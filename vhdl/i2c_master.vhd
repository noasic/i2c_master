-------------------------------------------------------------------------------
--
--  I2C Master 
--
--  Author(s):
--    Guy Eschemann, Guy.Eschemann@gmail.com
--
-------------------------------------------------------------------------------
--
-- Copyright (c) 2016, Guy Eschemann
-- All rights reserved.
-- 
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
-- 
-- 1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation
--    and/or other materials provided with the distribution.
-- 
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
-- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-- 
-- The views and conclusions contained in the software and documentation are those
-- of the authors and should not be interpreted as representing official policies,
-- either expressed or implied, of the FreeBSD Project.
--
-------------------------------------------------------------------------------

-- TODO: data setup time
-- TODO: spike filter
-- TODO: clock stretching (currently only for RD_BYTE)
-- TODO: arbitration
-- TODO: may need gray encoding for FSM

library ieee;

use ieee.std_logic_1164.all;
use work.i2c_master_pkg.all;

entity i2c_master is
    generic(
        g_sys_clk_freq : t_frequency := 100 MHz; -- system clock frequency
        g_i2c_clk_freq : t_frequency := 400 kHz -- i2c clock frequency
    );
    port(
        i_sys_clk  : in  std_logic;     -- system clock
        i_sys_rst  : in  std_logic;     -- system reset (synchronous, active-high)
        --
        i_scl      : in  std_logic;     -- SCL input input
        o_scl_tri  : out std_logic;     -- SCL tristate enable
        i_sda      : in  std_logic;     -- SDA input
        o_sda_tri  : out std_logic;     -- SDA tristate enable
        --
        i_tx_data  : in  std_logic_vector(7 downto 0); -- Tx interface, data word
        i_tx_nack  : in  std_logic;     -- Tx interface, NACK (1) or ACK (0)
        i_tx_instr : in  t_i2c_instr;   -- Tx interface, instruction word
        i_tx_valid : in  std_logic;     -- Tx interface, data/control word valid
        o_tx_ready : out std_logic;     -- Tx interface, ready to accept data/control word
        --
        o_rx_data  : out std_logic_vector(7 downto 0); -- Rx interface, data word
        o_rx_nack  : out std_logic;     -- Rx interface, NACK (1) or ACK (0)
        o_rx_valid : out std_logic;     -- Rx interface, data/control word valid
        i_rx_ready : in  std_logic      -- Rx interface, ready to accept data/control word
    );
end entity i2c_master;

architecture RTL of i2c_master is

    ------------------------------------------------------------------------------
    -- Types
    --
    type t_state is (IDLE, I2C_START_PHASE_1, I2C_START_PHASE_2, I2C_START_PHASE_3, --
                     I2C_START_PHASE_4, I2C_WR_BYTE_PHASE_1, I2C_WR_BYTE_PHASE_2, I2C_STOP_PHASE_1, --
                     I2C_STOP_PHASE_2, I2C_STOP_PHASE_3, I2C_SLAVE_ACK_PHASE_1, I2C_SLAVE_ACK_PHASE_2, --
                     WAIT_RX_READY, I2C_RD_BYTE_PHASE_1, I2C_RD_BYTE_PHASE_1a, I2C_RD_BYTE_PHASE_1b, --
                     I2C_RD_BYTE_PHASE_2, I2C_MASTER_ACK_PHASE_1, I2C_MASTER_ACK_PHASE_2);

    ------------------------------------------------------------------------------
    -- Registered signals
    --
    signal s_scl_r          : std_logic                    := '1';
    signal s_sda_tx_shreg_r : std_logic_vector(7 downto 0) := (others => '1');
    signal s_sda_rx_shreg_r : std_logic_vector(7 downto 0) := (others => '0');
    signal s_rx_nack_r      : std_logic                    := 'X';
    signal s_rx_valid_r     : std_logic                    := '0';
    signal s_scl_2x_ce_r    : std_logic                    := '0'; -- clock-enable at 2x the SCL frequency
    signal s_state_r        : t_state                      := IDLE;
    signal s_tx_nack_r      : std_logic                    := '0'; -- NACK/ACK value to transmit after reading a byte
    signal s_rx_data_r      : std_logic_vector(7 downto 0) := (others => 'X'); -- received byte

    ------------------------------------------------------------------------------
    -- Aliases
    --
    alias a_sda_tx is s_sda_tx_shreg_r(7);

begin

    ------------------------------------------------------------------------------
    -- Prescaler
    --
    prescaler : process(i_sys_clk) is
        constant RATIO     : natural                      := g_sys_clk_freq / (g_i2c_clk_freq * 2);
        variable v_count_r : natural range 0 to RATIO - 1 := 0;
    begin
        if rising_edge(i_sys_clk) then
            if i_sys_rst = '1' then
                v_count_r     := 0;
                s_scl_2x_ce_r <= '0';
            else
                if v_count_r = RATIO - 1 then
                    v_count_r     := 0;
                    s_scl_2x_ce_r <= '1';
                else
                    v_count_r     := v_count_r + 1;
                    s_scl_2x_ce_r <= '0';
                end if;
            end if;
        end if;
    end process prescaler;

    ------------------------------------------------------------------------------
    -- Control FSM
    --
    fsm_reg : process(i_sys_clk) is
        variable v_bit_idx_r : natural range 0 to 7 := 0;
        variable v_started_r : boolean              := false; -- whether a transfer has already started, or not
    begin
        if rising_edge(i_sys_clk) then
            if i_sys_rst = '1' then
                s_scl_r          <= '1';
                s_sda_tx_shreg_r <= (others => '1');
                s_sda_rx_shreg_r <= (others => '0'); -- don't care
                s_rx_nack_r      <= 'X'; -- don't care
                s_rx_data_r      <= (others => 'X'); -- don't care
                s_tx_nack_r      <= '0'; -- don't care
                s_rx_valid_r     <= '0';
                v_bit_idx_r      := 0;
                v_started_r      := false;
                s_state_r        <= IDLE;
            else
                case s_state_r is
                    -- Idle: wait for user instruction
                    when IDLE =>
                        v_bit_idx_r := 0;
                        if i_tx_valid = '1' then
                            if i_tx_instr = I2C_START then
                                if v_started_r then
                                    s_state_r <= I2C_START_PHASE_1; -- re-start
                                else
                                    v_started_r := true;
                                    s_state_r   <= I2C_START_PHASE_3; -- start 
                                end if;
                            elsif i_tx_instr = I2C_ADDR or i_tx_instr = I2C_WR_BYTE then
                                s_sda_tx_shreg_r <= i_tx_data;
                                s_state_r        <= I2C_WR_BYTE_PHASE_1;
                            elsif i_tx_instr = I2C_RD_BYTE then
                                s_tx_nack_r <= i_tx_nack;
                                s_state_r   <= I2C_RD_BYTE_PHASE_1;
                            elsif i_tx_instr = I2C_STOP then
                                v_started_r := false;
                                s_state_r   <= I2C_STOP_PHASE_1;
                            else
                                report "unsupported instruction" severity failure;
                            end if;
                        end if;

                    -- START, phase #1 (set SDA to 1)
                    when I2C_START_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_sda_tx_shreg_r <= (others => '1'); -- set SDA to 1
                            s_state_r        <= I2C_START_PHASE_2;
                        end if;

                    -- START, phase #2 (set SCL to 1)
                    when I2C_START_PHASE_2 =>
                        assert a_sda_tx = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '1';
                            s_state_r <= I2C_START_PHASE_3;
                        end if;

                    -- START, phase #1 (set SDA to 0)
                    when I2C_START_PHASE_3 =>
                        assert a_sda_tx = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_sda_tx_shreg_r <= (others => '0'); -- set SDA to 0
                            s_state_r        <= I2C_START_PHASE_4;
                        end if;

                    -- START, phase #2 (set SCL to 0)
                    when I2C_START_PHASE_4 =>
                        assert a_sda_tx = '0' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '0';
                            s_state_r <= IDLE;
                        end if;

                    -- Write byte to slave, phase #1 (generate rising SCL edge)
                    when I2C_WR_BYTE_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '1';
                            s_state_r <= I2C_WR_BYTE_PHASE_2;
                        end if;

                    -- Write byte to slave, phase #2 (generate falling SCL edge)
                    when I2C_WR_BYTE_PHASE_2 =>
                        assert s_scl_r = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r <= '0';
                            if v_bit_idx_r = 7 then
                                s_state_r <= I2C_SLAVE_ACK_PHASE_1;
                            else
                                v_bit_idx_r      := v_bit_idx_r + 1;
                                s_sda_tx_shreg_r <= s_sda_tx_shreg_r(6 downto 0) & '0';
                                s_state_r        <= I2C_WR_BYTE_PHASE_1;
                            end if;
                        end if;

                    -- Read one byte from the slave, phase #1 (generate rising SCL edge)
                    when I2C_RD_BYTE_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        s_sda_tx_shreg_r(7) <= '1'; -- SDA = 1
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '1';
                            s_state_r <= I2C_RD_BYTE_PHASE_1a;
                        end if;

                    -- Read one byte from the slave, phase #1a (clock stretching)
                    when I2C_RD_BYTE_PHASE_1a =>
                        if to_X01(i_scl) = '1' then
                            s_sda_rx_shreg_r <= s_sda_rx_shreg_r(6 downto 0) & i_sda; -- read SDA level, TODO: may need synchronizer
                            s_state_r        <= I2C_RD_BYTE_PHASE_1b;
                        end if;

                    -- Read one byte from the slave, phase #1b (re-sychronize to prescaler)
                    when I2C_RD_BYTE_PHASE_1b =>
                        if s_scl_2x_ce_r = '1' then
                            s_state_r <= I2C_RD_BYTE_PHASE_2;
                        end if;

                    -- Read one byte from the slave, phase #2 (generate falling SCL edge)
                    when I2C_RD_BYTE_PHASE_2 =>
                        assert s_scl_r = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r <= '0';
                            if v_bit_idx_r = 7 then
                                s_state_r <= I2C_MASTER_ACK_PHASE_1;
                            else
                                v_bit_idx_r := v_bit_idx_r + 1;
                                s_state_r   <= I2C_RD_BYTE_PHASE_1;
                            end if;
                        end if;

                    -- Master ACK, phase #1 (generate rising SCL edge) 
                    when I2C_MASTER_ACK_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        s_sda_tx_shreg_r(7) <= s_tx_nack_r; -- SDA = ACK/NACK
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '1';
                            s_state_r <= I2C_MASTER_ACK_PHASE_2;
                        end if;

                    -- Master ACK, phase #2 (generate falling SCL edge) 
                    when I2C_MASTER_ACK_PHASE_2 =>
                        assert s_scl_r = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_sda_tx_shreg_r(7) <= '1'; -- SDA = 1
                            s_scl_r             <= '0';
                            s_rx_valid_r        <= '1';
                            s_rx_data_r         <= s_sda_rx_shreg_r;
                            s_rx_nack_r         <= s_tx_nack_r; -- output the transmitted ACK
                            s_state_r           <= WAIT_RX_READY;
                        end if;

                    -- Slave ACK, phase #1 (generate rising SCL edge) 
                    when I2C_SLAVE_ACK_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        s_sda_tx_shreg_r <= (others => '1'); -- SDA = 1
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r          <= '1';
                            s_sda_rx_shreg_r <= s_sda_rx_shreg_r(6 downto 0) & i_sda; -- read ACK level
                            s_state_r        <= I2C_SLAVE_ACK_PHASE_2;
                        end if;

                    -- Slave ACK, phase #2 (generate falling SCL edge) 
                    when I2C_SLAVE_ACK_PHASE_2 =>
                        assert s_scl_r = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r      <= '0';
                            s_rx_valid_r <= '1';
                            s_rx_nack_r  <= s_sda_rx_shreg_r(0);
                            s_state_r    <= WAIT_RX_READY;
                        end if;

                    -- Wait for RX stream ready
                    when WAIT_RX_READY =>
                        if i_rx_ready = '1' then
                            s_rx_valid_r <= '0';
                            s_rx_nack_r  <= 'X';
                            s_rx_data_r  <= (others => 'X');
                            s_state_r    <= IDLE;
                        end if;

                    -- STOP, phase #1 (clear SDA)
                    when I2C_STOP_PHASE_1 =>
                        assert s_scl_r = '0' severity failure;
                        s_sda_tx_shreg_r <= (others => '0');
                        if s_scl_2x_ce_r = '1' then
                            s_state_r <= I2C_STOP_PHASE_2;
                        end if;

                    -- STOP, phase #2 (set SCL)
                    when I2C_STOP_PHASE_2 =>
                        assert a_sda_tx = '0' severity failure;
                        assert s_scl_r = '0' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_scl_r   <= '1';
                            s_state_r <= I2C_STOP_PHASE_3;
                        end if;

                    -- STOP, phase #2 (set SDA)
                    when I2C_STOP_PHASE_3 =>
                        assert a_sda_tx = '0' severity failure;
                        assert s_scl_r = '1' severity failure;
                        if s_scl_2x_ce_r = '1' then
                            s_sda_tx_shreg_r <= (others => '1'); -- SDA = 1
                            s_state_r        <= IDLE;
                        end if;

                end case;
            end if;
        end if;
    end process fsm_reg;

    fsm_comb : process(s_state_r) is
    begin
        case s_state_r is
            when IDLE =>
                o_tx_ready <= '1';
            when others =>
                o_tx_ready <= '0';
        end case;
    end process fsm_comb;

    ------------------------------------------------------------------------------
    -- Outputs
    --
    o_scl_tri  <= s_scl_r;
    o_sda_tri  <= a_sda_tx;
    --
    o_rx_nack  <= to_X01(s_rx_nack_r);
    o_rx_data  <= to_X01(s_rx_data_r);
    o_rx_valid <= s_rx_valid_r;

end architecture RTL;