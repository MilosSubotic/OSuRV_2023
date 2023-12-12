
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.automotive_types.all;

entity chassis_drive is
	port(
		i_chassis : in  t_chassis;
		o_l_mot_in1 : out std_logic;
		o_l_mot_in2 : out std_logic;
		o_r_mot_in1 : out std_logic;
		o_r_mot_in2 : out std_logic
	);
end entity;

architecture arch of chassis_drive is
	
	signal l_track : t_track;
	signal r_track : t_track;
	
begin
	
	process(i_chassis)
	begin
		if i_chassis.move = CM_FULL_BRAKE then
			-- 1
			l_track <= T_BRAKE;
			r_track <= T_BRAKE;
		elsif i_chassis.turn = CT_INVALID
			or i_chassis.turn_hardness = CTH_INVALID
		then
			-- NA
			l_track <= T_COAST;
			r_track <= T_COAST;
		else
			case i_chassis.turn is
				when CT_STRAIGHT =>
					case i_chassis.move is
						when CM_IDLE =>
							-- 0
							l_track <= T_COAST;
							r_track <= T_COAST;
						when CM_FULL_BRAKE => null;
						when CM_FORWARD =>
							-- 2
							l_track <= T_FORWARD;
							r_track <= T_FORWARD;
						when CM_REVERSE =>
							-- 3
							l_track <= T_REVERSE;
							r_track <= T_REVERSE;
					end case;
				when CT_RIGHT =>
					case i_chassis.move is
						when CM_IDLE =>
							-- 4
							l_track <= T_FORWARD;
							r_track <= T_REVERSE;
						when CM_FULL_BRAKE => null;
						when CM_FORWARD =>
							case i_chassis.turn_hardness is
								when CTH_FLOAT =>
									-- 10
									r_track <= T_COAST;
								when CTH_BRAKE =>
									-- 20
									r_track <= T_BRAKE;
								when CTH_IN_PLACE =>
									-- 30
									r_track <= T_REVERSE;
								when CTH_INVALID => null;
							end case;
							l_track <= T_FORWARD;
						when CM_REVERSE =>
							case i_chassis.turn_hardness is
								when CTH_FLOAT =>
									-- 11
									r_track <= T_COAST;
								when CTH_BRAKE =>
									-- 21
									r_track <= T_BRAKE;
								when CTH_IN_PLACE =>
									-- 31
									r_track <= T_FORWARD;
								when CTH_INVALID => null;
							end case;
							l_track <= T_REVERSE;
					end case;
				when CT_LEFT =>
					case i_chassis.move is
						when CM_IDLE =>
							-- 5
							l_track <= T_REVERSE;
							r_track <= T_FORWARD;
						when CM_FULL_BRAKE => null;
						when CM_FORWARD =>
							case i_chassis.turn_hardness is
								when CTH_FLOAT =>
									-- 12
									l_track <= T_COAST;
								when CTH_BRAKE =>
									-- 22
									l_track <= T_BRAKE;
								when CTH_IN_PLACE =>
									-- 32
									l_track <= T_REVERSE;
								when CTH_INVALID => null;
							end case;
							r_track <= T_FORWARD;
						when CM_REVERSE =>
							case i_chassis.turn_hardness is
								when CTH_FLOAT =>
									-- 13
									l_track <= T_COAST;
								when CTH_BRAKE =>
									-- 23
									l_track <= T_BRAKE;
								when CTH_IN_PLACE =>
									-- 33
									l_track <= T_FORWARD;
								when CTH_INVALID => null;
							end case;
							r_track <= T_REVERSE;
					end case;
				when CT_INVALID => null;
			end case;
		end if;
	end process;
	
	l_motor_drive_inst: entity work.motor_drive
	port map(
		i_track   => l_track,
		o_mot_in1 => o_l_mot_in1,
		o_mot_in2 => o_l_mot_in2
	);
	r_motor_drive_inst: entity work.motor_drive
	port map(
		i_track   => r_track,
		o_mot_in1 => o_r_mot_in1,
		o_mot_in2 => o_r_mot_in2
	);
	
end architecture;
