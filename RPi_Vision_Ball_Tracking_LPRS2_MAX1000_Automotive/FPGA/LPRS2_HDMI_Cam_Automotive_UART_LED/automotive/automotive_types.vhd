
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

package automotive_types is
	
	
	type t_chassis_move is (
		CM_IDLE,
		CM_FULL_BRAKE,
		CM_FORWARD,
		CM_REVERSE
	);
	type t_chassis_turn is (
		CT_STRAIGHT,
		CT_RIGHT,
		CT_LEFT,
		CT_INVALID
	);
	type t_chassis_turn_hardness is (
		CTH_FLOAT,
		CTH_BRAKE,
		CTH_IN_PLACE,
		CTH_INVALID
	);
	
	type t_chassis is record
		move : t_chassis_move;
		turn : t_chassis_turn;
		turn_hardness : t_chassis_turn_hardness;
	end record t_chassis;
	
	function conv_t_chassis(
		s : std_logic_vector(5 downto 0)
	) return t_chassis;
	
	
	type t_track is (
		T_COAST,
		T_REVERSE,
		T_FORWARD,
		T_BRAKE
	);

end package automotive_types;

package body automotive_types is
	function conv_t_chassis(
		s : std_logic_vector(5 downto 0)
	) return t_chassis
	is
	begin
		return (
			t_chassis_move'val(conv_integer(s(1 downto 0))),
			t_chassis_turn'val(conv_integer(s(3 downto 2))),
			t_chassis_turn_hardness'val(conv_integer(s(5 downto 4)))
		);
	end;
end package body;
