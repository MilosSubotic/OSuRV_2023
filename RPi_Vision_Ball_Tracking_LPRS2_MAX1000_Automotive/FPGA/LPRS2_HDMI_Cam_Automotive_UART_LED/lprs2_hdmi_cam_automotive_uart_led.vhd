
library ieee;
use ieee.std_logic_1164.all;
use ieee.math_real.all;

library work;
use work.gpu_types.all;
use work.automotive_types.all;


entity lprs2_hdmi_cam_automotive_uart_led is
	generic(
		-- Default frequency used in synthesis.
		constant CLK_FREQ : positive := 12000000
	);
	port (
		i_clk           : in  std_logic;
		in_rst          : in  std_logic;
		
		op_hdmi_clk     : out std_logic;
		on_hdmi_clk     : out std_logic;
		op_hdmi_data    : out std_logic_vector(2 downto 0);
		on_hdmi_data    : out std_logic_vector(2 downto 0);
		
		
		o_cam_xclk      : out   std_logic;
		on_cam_rst      : out   std_logic;
		o_cam_pwdn      : out   std_logic;
		o_cam_sioc      : out   std_logic;
		io_cam_siod     : inout std_logic;
		i_cam_pclk      : in    std_logic;
		i_cam_vsync     : in    std_logic;
		i_cam_href      : in    std_logic;
		i_cam_data      : in    std_logic_vector(7 downto 0);

		-- Motors.
		o_l_mot_in1     : out std_logic;
		o_l_mot_in2     : out std_logic;
		o_r_mot_in1     : out std_logic;
		o_r_mot_in2     : out std_logic;
		
		-- UART.
		o_uart_tx       : out std_logic;
		o_uart_rx       : in  std_logic;
		
		o_led           : out std_logic_vector(7 downto 0)
	);
end entity lprs2_hdmi_cam_automotive_uart_led;

architecture arch of lprs2_hdmi_cam_automotive_uart_led is

	-- Config.
	constant BAUD_RATE : natural := 115200;
	
	-------------

	signal pll_rst : std_logic;
	signal n_rst : std_logic;
	signal hdmi_clk : std_logic;
	signal gpu_clk : std_logic;
	
	signal pix_phase : t_pix_phase;
	signal pix_x : t_pix_x;
	signal pix_y : t_pix_y;
	signal pix_rgb : t_rgb888;
	
	signal sync : std_logic;
	signal cam_rgb656 : t_rgb565;
	signal cam_rgb : t_rgb888;
	
	signal chassis : t_chassis;
	
		
	constant CLKS_PER_BIT : natural := 
		integer(round(real(CLK_FREQ)/real(BAUD_RATE)));
	
	signal uart_rx_d : std_logic_vector(7 downto 0);
	signal uart_rx_dv : std_logic;
	signal uart_tx_d : std_logic_vector(7 downto 0);
	signal uart_tx_dv : std_logic;

	
begin
	
	pll_rst <= not in_rst;
	pll_inst: entity work.PLL
	port map (
		inclk0 => i_clk,
		areset => pll_rst,
		c0 => hdmi_clk,
		c1 => gpu_clk,
		locked => n_rst
	);
	
	pix_ctrl_inst: entity work.pix_ctrl
	generic map (
		DELAY => 4
	)
	port map (
		i_gpu_clk_100MHz => gpu_clk,
		i_hdmi_clk_250MHz => hdmi_clk,
		in_rst => n_rst,
		
		i_pix_sync => sync,
		o_pix_phase => pix_phase,
		o_pix_x => pix_x,
		o_pix_y => pix_y,
		i_pix_r => pix_rgb( 7 downto  0),
		i_pix_g => pix_rgb(15 downto  8),
		i_pix_b => pix_rgb(23 downto 16),
		
		op_hdmi_clk => op_hdmi_clk,
		on_hdmi_clk => on_hdmi_clk,
		op_hdmi_data => op_hdmi_data,
		on_hdmi_data => on_hdmi_data
	);

	
	cam_inst: entity work.cam_ov7670
	port map(
		i_gpu_clk     => gpu_clk,
		in_rst        => n_rst,
		
		i_reconfigure => '0',
		
		-- To camera.
		o_cam_xclk    => o_cam_xclk,
		on_cam_rst    => on_cam_rst,
		o_cam_pwdn    => o_cam_pwdn,
		o_cam_sioc    => o_cam_sioc,
		io_cam_siod   => io_cam_siod,
		i_cam_pclk    => i_cam_pclk,
		i_cam_vsync   => i_cam_vsync,
		i_cam_href    => i_cam_href,
		i_cam_data    => i_cam_data,
		
		o_cfg_done    => open,
		
		-- To GPU.
		o_gpu_sync    => sync,
		o_gpu_rgb     => cam_rgb656
	);
	
	cam_rgb <= 
		cam_rgb656(15 downto 11) & "000" & -- Blue.
		cam_rgb656(10 downto  5) &  "00" & -- Green.
		cam_rgb656( 4 downto  0) & "000"; -- Red.
	
	chassis_drive_inst: entity work.chassis_drive
	port map (
		i_chassis   => chassis,
		o_l_mot_in1 => o_l_mot_in1,
		o_l_mot_in2 => o_l_mot_in2,
		o_r_mot_in1 => o_r_mot_in1,
		o_r_mot_in2 => o_r_mot_in2
	);
	
	main_inst: entity work.main
	port map(
		i_clk           => i_clk,
		i_gpu_clk       => gpu_clk,
		in_rst          => n_rst,
		
		i_cam_rgb       => cam_rgb,
		i_pix_phase     => pix_phase,
		i_pix_x         => pix_x,
		i_pix_y         => pix_y,
		o_pix_rgb       => pix_rgb,
		
		o_chassis       => chassis,
		
		i_uart_rx_dv    => uart_rx_dv,
		i_uart_rx_d     => uart_rx_d,
		o_uart_tx_dv    => uart_tx_dv,
		o_uart_tx_d     => uart_tx_d,
		
		o_led           => o_led
	);

	uart_rx_inst : entity work.uart_rx
	generic map (
		g_CLKS_PER_BIT => CLKS_PER_BIT
	)
	port map (
		i_clk       => i_clk,
		i_rx_serial => o_uart_rx,
		o_rx_dv     => uart_rx_dv,
		o_rx_byte   => uart_rx_d
	);
	
	-- Instantiate UART transmitter
	uart_tx_i : entity work.uart_tx
	generic map (
		g_CLKS_PER_BIT => CLKS_PER_BIT
	)
	port map (
		i_clk       => i_clk,
		i_tx_dv     => uart_tx_dv,
		i_tx_byte   => uart_tx_d,
		o_tx_active => open,
		o_tx_serial => o_uart_tx,
		o_tx_done   => open
	);


end architecture arch;
