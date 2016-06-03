library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package i2c_master_pkg is

  type t_i2c_instr is (I2C_START, I2C_STOP, I2C_ADDR, I2C_WR_BYTE, I2C_RD_BYTE);

  type t_frequency is range 0 to 2147483647 units kHz;
    MHz = 1000 kHz;
    GHz = 1000 MHz;
  end units;

  function "/"(a : integer; b : t_frequency) return time;

end package i2c_master_pkg;

package body i2c_master_pkg is
  function "/"(a : integer; b : t_frequency) return time is
  begin
    return a * 1 ms / (b / 1 kHz);
  end function;

end package body i2c_master_pkg;
