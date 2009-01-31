-----------------------------------------------------------------------------
-- Title    : ADNS 6010 Control Unit
-- Project  : UNIOC_NG Optic Encoders
-----------------------------------------------------------------------------
-- File     : adns6010_controlunit.vhd
-- Author   : JD (jd@robotter.fr)
-- Company  : Rob'Otter
-- 
-- Creation date : 24/01/2009
-- Platform : Altera Cyclone
-----------------------------------------------------------------------------
-- Description : Control unit for the automated mode of optical sensors.
-- This entity shall :
-- * access sequentialy each ADNS / using SPI MOTION BURST.
-- * store and output sum of delta_x, delta_y for each ADNS
-- * output immediate value of squal for each ADNS
-----------------------------------------------------------------------------
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2, or (at your option)
-- any later version.
-- 
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
-- 
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, write to the Free Software
-----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

ENTITY latch_nbits IS
  
  GENERIC (
    CONSTANT data_width_c : natural RANGE 0 TO 127 := 32;  -- width of the data bus latched
		CONSTANT squal_width_c : natural := 8								 -- width of the squal bus latched
		); 
		

  PORT (
    clk_i            : IN  std_ulogic;
    reset_ni         : IN  std_ulogic;
    deltax_i         : IN  std_logic_vector(data_width_c-1 DOWNTO 0);  -- data to be latched
    deltay_i         : IN  std_logic_vector(data_width_c-1 DOWNTO 0);  -- data to be latched
    squal_i          : IN  std_logic_vector(squal_width_c-1 DOWNTO 0);  -- data to be latched
    deltax_latched_o : OUT std_logic_vector(data_width_c-1 DOWNTO 0);  -- data latched
    deltay_latched_o : OUT std_logic_vector(data_width_c-1 DOWNTO 0);  -- data latched
    squal_latched_o  : OUT std_logic_vector(squal_width_c-1 DOWNTO 0);  -- data latched
    latch_data_i     : IN  std_ulogic);  -- latches data_i while it is active (i.e. '1')
END latch_nbits;

ARCHITECTURE latch_n_bits_1 OF latch_nbits IS

  BEGIN  -- latch_n_bits_1
    
    latch_deltax: PROCESS (clk_i)
        VARIABLE last_latch_data_v : std_ulogic;  -- memory of the last sample of latch_data_i
    BEGIN  -- PROCESS latch
      IF rising_edge(clk_i) THEN  -- rising clock edge   

          IF reset_ni = '0' THEN
          last_latch_data_v := latch_data_i;
          
        ELSIF latch_data_i = '1' AND last_latch_data_v = '0' THEN
          deltax_latched_o <= deltax_i;
        END IF;

        last_latch_data_v := latch_data_i;
            
      END IF;
    END PROCESS latch_deltax;


    latch_deltay: PROCESS (clk_i)
        VARIABLE last_latch_data_v : std_ulogic;  -- memory of the last sample of latch_data_i
    BEGIN  -- PROCESS latch
      IF rising_edge(clk_i) THEN  -- rising clock edge   
          IF reset_ni = '0' THEN
          last_latch_data_v := latch_data_i;
          
        ELSIF latch_data_i = '1' AND last_latch_data_v = '0' THEN
          deltay_latched_o <= deltay_i;
        END IF;

        last_latch_data_v := latch_data_i;
            
      END IF;
    END PROCESS latch_deltay;



    latch_squal: PROCESS (clk_i)
        VARIABLE last_latch_data_v : std_ulogic;  -- memory of the last sample of latch_data_i
    BEGIN  -- PROCESS latch
      IF rising_edge(clk_i) THEN  -- rising clock edge   
        IF reset_ni = '0' THEN
          last_latch_data_v := latch_data_i;
          
        ELSIF latch_data_i = '1' AND last_latch_data_v = '0' THEN
          squal_latched_o <=  squal_i;
        END IF;

        last_latch_data_v := latch_data_i;
            
      END IF;
    END PROCESS latch_squal;


  END latch_n_bits_1;
