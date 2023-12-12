
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

library work;
use work.gpu_types.all;
use work.automotive_types.all;

entity main is
	port (
		i_clk           : in    std_logic; -- 12 MHz
		i_gpu_clk       : in    std_logic; -- 100 MHz
		in_rst          : in    std_logic;
		
		i_cam_rgb       : in    t_rgb888;
		i_pix_phase     : in    t_pix_phase;
		i_pix_x         : in    t_pix_x;
		i_pix_y         : in    t_pix_y;
		o_pix_rgb       : out   t_rgb888;
		
		o_chassis       : out   t_chassis;
		
		-- UART.
		i_uart_rx_d  : in  std_logic_vector(7 downto 0);
		i_uart_rx_dv : in  std_logic;
		o_uart_tx_d  : out std_logic_vector(7 downto 0);
		o_uart_tx_dv : out std_logic;
		
		o_led        : out std_logic_vector(7 downto 0)
	);
end entity main;

architecture arch of main is
	
	
	signal chassis_cmd : std_logic_vector(5 downto 0);
	signal chassis_speed : std_logic_vector(1 downto 0);
	signal chassis_cmd_decoded : t_chassis;
	signal chassis_pwm : std_logic;
	signal chassis_cmd_idle : t_chassis := (CM_IDLE, CT_STRAIGHT, CTH_FLOAT);
	
	-- ~12.5Hz, yo work on lowest speed, bcs need on time at least 10ms.
	constant PWM_CNT_BITS : natural := 20;
	signal pwm_cnt : std_logic_vector(PWM_CNT_BITS-1 downto 0);
	signal pwm_cnt_upper : std_logic_vector(2 downto 0);
	signal pwm_cnt_threshold : std_logic_vector(3 downto 0);
begin
	o_pix_rgb <= 
		x"0000ff" when i_pix_x < 100 else
		x"00ff00" when i_pix_x < 200 else
		x"ff0000" when i_pix_x < 300 else
		i_cam_rgb;
	
	-- Loop it back.
	o_uart_tx_d <= i_uart_rx_d;
	o_uart_tx_dv <= i_uart_rx_dv;
	
	o_led <= i_uart_rx_d;
	
	chassis_cmd <= i_uart_rx_d(5 downto 0);
	chassis_speed <= i_uart_rx_d(7 downto 6);
	-- For testing.
	--chassis_cmd <= "000010";
	--chassis_speed <= "00";
	
	
	chassis_cmd_decoded <= conv_t_chassis(chassis_cmd);
	
	
	process(i_clk, in_rst)
	begin
		if in_rst = '0' then
			pwm_cnt <= (others => '0');
		elsif rising_edge(i_clk) then
			pwm_cnt <= pwm_cnt + 1;
		end if;
	end process;
	pwm_cnt_upper <= pwm_cnt(pwm_cnt'high downto pwm_cnt'high-2);
	with chassis_speed select pwm_cnt_threshold <=
		"0001" when "00",
		"0010" when "01",
		"0100" when "10",
		"1000" when others;
	
	chassis_pwm <= '1' when pwm_cnt_upper < pwm_cnt_threshold else '0';
	
	
	with chassis_pwm select o_chassis <=
		chassis_cmd_decoded when '1',
		chassis_cmd_idle when others;
	
end architecture arch;
