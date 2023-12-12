
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.automotive_types.all;

entity motor_drive is
	port(
		i_track   : in  t_track;
		o_mot_in1 : out std_logic;
		o_mot_in2 : out std_logic
	);
end entity;

architecture arch of motor_drive is
	
begin

	process(i_track)
	begin
		case i_track is
			when T_COAST =>
				o_mot_in1 <= '0';
				o_mot_in2 <= '0';
			when T_REVERSE =>
				o_mot_in1 <= '0';
				o_mot_in2 <= '1';
			when T_FORWARD =>
				o_mot_in1 <= '1';
				o_mot_in2 <= '0';
			when T_BRAKE =>
				o_mot_in1 <= '1';
				o_mot_in2 <= '1';
		end case;
	end process;
	
end architecture;
