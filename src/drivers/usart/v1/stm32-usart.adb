------------------------------------------------------------------------------
--                                                                          --
--                    Copyright (C) 2015, AdaCore                           --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of STMicroelectronics nor the names of its       --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
--                                                                          --
--  This file is based on:                                                  --
--                                                                          --
--   @file    stm32f4xx_hal_spi.c                                           --
--   @author  MCD Application Team                                          --
--   @version V1.1.0                                                        --
--   @date    19-June-2014                                                  --
--   @brief   SPI HAL module driver.                                        --
--                                                                          --
--   COPYRIGHT(c) 2014 STMicroelectronics                                   --
------------------------------------------------------------------------------

with Ada.Unchecked_Conversion;

with STM32_SVD.USART; use STM32_SVD.USART;

package body STM32.USART is

   use type HAL.UART.UART_Data_Size;

   Baud_Rate_Value : constant array (USART_Baud_Rate_Prescaler) of UInt3 :=
     (BRP_2   => 2#000#,
      BRP_4   => 2#001#,
      BRP_8   => 2#010#,
      BRP_16  => 2#011#,
      BRP_32  => 2#100#,
      BRP_64  => 2#101#,
      BRP_128 => 2#110#,
      BRP_256 => 2#111#);

   type Half_Word_Pointer is access all UInt16
     with Storage_Size => 0;

   function As_Half_Word_Pointer is new Ada.Unchecked_Conversion
     (Source => System.Address, Target => Half_Word_Pointer);
   --  So that we can treat the address of a UInt8 as a pointer to a two-UInt8
   --  sequence representing a Half_Word quantity

   ---------------
   -- APB_Clock --
   ---------------

   function APB_Clock (This : in out USART_Port) return UInt32 is
   begin
      if This.Periph = USART1_Periph or This.Periph = USART6_Periph
      then
         return Clocks.PCLK2;
      else
         return Clocks.PCLK1;
      end if;
   end APB_Clock;

   ---------------
   -- Configure --
   ---------------

   procedure Configure (This : in out USART_Port; Conf : USART_Configuration) is
      Clock       : constant UInt32 := APB_Clock (This);
      Int_Scale   : constant UInt32 := (if Conf.Oversampling = Oversampling_8x then 2 else 4);
      Int_Divider : constant UInt32 := (25 * Clock) / (Int_Scale * Conf.Baud_Rate);
      Frac_Divider : constant UInt32 := Int_Divider rem 100;
   begin
      case Conf.Mode is
         when Syncrhonous =>
            null; -- TODO: Implement difference for this, or remove if determined externally
         when Asyncrhonous =>
            null;
      end case;

      case Conf.Direction is
         when RX_TX =>
            This.Periph.CR1.RE := True;
            This.Periph.CR1.TE := True;
         when RX =>
            This.Periph.CR1.RE := True;
            This.Periph.CR1.TE := False;
         when TX =>
            This.Periph.CR1.RE := False;
            This.Periph.CR1.TE := True;
      end case;

      case Conf.Flow_Control is
         when No_Flow_Control =>
            This.Periph.CR3.RTSE := False;
            This.Periph.CR3.CTSE := False;
         when RTS_Flow_Control =>
            This.Periph.CR3.RTSE := True;
            This.Periph.CR3.CTSE := False;
         when CTS_Flow_Control =>
            This.Periph.CR3.RTSE := False;
            This.Periph.CR3.CTSE := True;
         when RTS_CTS_Flow_Control =>
            This.Periph.CR3.RTSE := True;
            This.Periph.CR3.CTSE := True;
      end case;

      case Conf.Parity is
         when No_Parity =>
            This.Periph.CR1.PCE := False;
         when Even_Parity =>
            This.Periph.CR1.PCE := True;
            This.Periph.CR1.PS := False;
         when Odd_Parity =>
            This.Periph.CR1.PCE := True;
            This.Periph.CR1.PS := True;
      end case;

      case Conf.Oversampling is
         when Oversampling_16x =>
            This.Periph.CR1.OVER8 := False;
            This.Periph.BRR.DIV_Fraction :=
               BRR_DIV_Fraction_Field (((Frac_Divider * 16) + 50) / 100 mod 8);
         when Oversampling_8x =>
            This.Periph.CR1.OVER8 := True;
            This.Periph.BRR.DIV_Fraction :=
               BRR_DIV_Fraction_Field (((Frac_Divider * 8) + 50) / 100 mod 8);
      end case;
      This.Periph.BRR.DIV_Mantissa :=
         BRR_DIV_Mantissa_Field (Int_Divider / 100);

      
      This.Perioh.CR1.M0    := False; -- If true, data_size is 7 bits.
      This.Periph.CR1.M1    := Conf.Data_Size = HAL.UART.Data_Size_9b;


      -- Interrupt configuration also exists in CR1
      

      --  This.Periph.CR1.DFF      := Conf.Data_Size = HAL.SPI.Data_Size_16b;
      --  This.Periph.CR1.CPOL     := Conf.Clock_Polarity = High;
      --  This.Periph.CR1.CPHA     := Conf.Clock_Phase = P2Edge;
      --  This.Periph.CR1.SSM      := Conf.Slave_Management = Software_Managed;
      --  This.Periph.CR1.BR       := Baud_Rate_Value (Conf.Baud_Rate_Prescaler);
      --  This.Periph.CR1.LSBFIRST := Conf.First_Bit = LSB;

      This.Periph.CR2.STOP := Conf.Stop_Bits'Enum_Rep;

      --  Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register)
      --  This.Periph.I2SCFGR.I2SMOD := False;

      This.Periph.CRCPR.CRCPOLY := Conf.CRC_Poly;
   end Configure;

   ------------
   -- Enable --
   ------------

   procedure Enable (This : in out USART_Port) is
   begin
      This.Periph.CR1.UE := True;
   end Enable;

   -------------
   -- Disable --
   -------------

   procedure Disable (This : in out USART_Port) is
   begin
      This.Periph.CR1.UE := False;
   end Disable;

   -------------
   -- Enabled --
   -------------

   function Enabled (This : USART_Port) return Boolean is
   begin
      return This.Periph.CR1.UE;
   end Enabled;

   ----------
   -- Send --
   ----------

   procedure Send (This : in out USART_Port; Data : UInt9) is
   begin
      This.Periph.TDR.TDR := Data;
   end Send;

   ----------
   -- Data --
   ----------

   function Data (This : USART_Port) return UInt9 is
   begin
      return This.Periph.RDR.RDR;
   end Data;

   ----------
   -- Send --
   ----------

   procedure Send (This : in out USART_Port; Data : UInt8) is
   begin
      Send (This, UInt9 (Data));
   end Send;

   ----------
   -- Data --
   ----------

   function Data (This : USART_Port) return UInt8 is
   begin
      return UInt8 (UInt9'(Data (This)));
   end Data;

   -------------
   -- Is_Busy --
   -------------

   function Is_Busy (This : USART_Port) return Boolean is
   begin
      return (Rx_Is_Empty (This)
              and then not Tx_Is_Empty (This))
        or else Busy (This);
   end Is_Busy;

   -----------------
   -- Tx_Is_Empty --
   -----------------

   function Tx_Is_Empty (This : USART_Port) return Boolean is
   begin
      return This.Periph.ISR.TXE;
   end Tx_Is_Empty;

   -----------------
   -- Rx_Is_Empty --
   -----------------

   function Rx_Is_Empty (This : USART_Port) return Boolean is
   begin
      return not This.Periph.ISR.RXNE;
   end Rx_Is_Empty;

   ----------
   -- Busy --
   ----------

   function Busy (This : USART_Port) return Boolean is
   begin
      return This.Periph.ISR.BUSY;
   end Busy;

   ------------------
   -- Current_Mode --
   ------------------

   --I changed this to sync and async mode if it's determined externally just remove this
   function Current_Mode (This : USART_Port) return USART_Mode is
   begin
      if This.Periph.CR1.Syncrhonous and This.Periph.CR1.SSI then
         return Syncrhonous;
      else
         return Asyncrhonous;
      end if;
   end Current_Mode;

   ----------------------------
   -- Current_Data_Direction --
   ----------------------------

   function Current_Data_Direction (This : USART_Port) return USART_Data_Direction
   is
   begin
      if This.Periph.CR1.RE then
         if THIS.Periph.CR1.TE then
            return RX_TX;
         else
            return RX;
         end if;
      else
         return TX;
      end if;
   end Current_Data_Direction;

   -----------------
   -- CRC_Enabled --
   -----------------

   function CRC_Enabled (This : USART_Port) return Boolean is
      (This.Periph.CR1.CRCEN);

   ----------------------------
   -- Channel_Side_Indicated --
   ----------------------------

   function Channel_Side_Indicated (This : USART_Port) return Boolean is
     (This.Periph.ISR.CHSIDE);

   ------------------------
   -- Underrun_Indicated --
   ------------------------

   function Underrun_Indicated (This : USART_Port) return Boolean is
     (This.Periph.ISR.UDR);

   -------------------------
   -- CRC_Error_Indicated --
   -------------------------

   function CRC_Error_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.CRCERR);

   --------------------------
   -- Mode_Fault_Indicated --
   --------------------------

   function Mode_Fault_Indicated (This : USART_Port) return Boolean is
     (This.Periph.ISR.MODF);

   -----------------------
   -- Overrun_Indicated --
   -----------------------

   function Overrun_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.OVR);

   -------------------------------
   -- Frame_Fmt_Error_Indicated --
   -------------------------------

   function Frame_Fmt_Error_Indicated (This : SPI_Port) return Boolean is
   begin
      return This.Periph.ISR.TIFRFE;
   end Frame_Fmt_Error_Indicated;

   -------------------
   -- Clear_Overrun --
   -------------------

   procedure Clear_Overrun (This : SPI_Port) is
      Dummy1 : UInt16;
      Dummy2 : SR_Register;
   begin
      Dummy1 := This.Periph.DR.DR;
      Dummy2 := This.Periph.ISR;
   end Clear_Overrun;

   ---------------
   -- Reset_CRC --
   ---------------

   procedure Reset_CRC (This : in out USART_Port) is
   begin
      This.Periph.CR1.CRCEN := False;
      This.Periph.CR1.CRCEN := True;
   end Reset_CRC;

   -------------------------
   -- Is_Data_Frame_16bit --
   -------------------------

   function Is_Data_Frame_16bit (This : SPI_Port) return Boolean is
      (This.Periph.CR1.DFF);

   ---------------
   -- Data_Size --
   ---------------

   overriding
   function Data_Size (This : SPI_Port) return HAL.SPI.SPI_Data_Size is
   begin
      if This.Periph.CR1.DFF then
         return HAL.SPI.Data_Size_16b;
      else
         return HAL.SPI.Data_Size_8b;
      end if;
   end Data_Size;

   --------------
   -- Transmit --
   --------------

   overriding
   procedure Transmit
     (This    : in out SPI_Port;
      Data    : HAL.SPI.SPI_Data_8b;
      Status  : out HAL.SPI.SPI_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Timeout);
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      --  ??? right value to compare???
      if Current_Data_Direction (This) = D1Line_Tx  then
         This.Periph.CR1.BIDIOE := True;
      end if;

      Clear_Overrun (This);

      if not Enabled (This) then
         Enable (This);
      end if;

      Send_8bit_Mode (This, Data);

      --  Wait until TXE flag is set to send data
      while not Tx_Is_Empty (This) loop
         null;
      end loop;

      --  Wait until Busy flag is reset before disabling SPI
      while Busy (This) loop
         null;
      end loop;

      --  Clear OVERUN flag in 2-Line communication mode because received UInt8
      --  is not read.
      if Current_Data_Direction (This) in D2Lines_RxOnly | D2Lines_FullDuplex
      then  -- right comparison ???
         Clear_Overrun (This);
      end if;
      Status := HAL.SPI.Ok;
   end Transmit;

   --------------
   -- Transmit --
   --------------

   overriding
   procedure Transmit
     (This    : in out SPI_Port;
      Data    : HAL.SPI.SPI_Data_16b;
      Status  : out HAL.SPI.SPI_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Timeout);
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      --  ??? right value to compare???
      if Current_Data_Direction (This) = D1Line_Tx then
         This.Periph.CR1.BIDIOE := True;
      end if;

      Clear_Overrun (This);

      if not Enabled (This) then
         Enable (This);
      end if;

      Send_16bit_Mode (This, Data);

      --  Wait until TXE flag is set to send data
      while not Tx_Is_Empty (This) loop
         null;
      end loop;

      --  Wait until Busy flag is reset before disabling SPI
      while Busy (This) loop
         null;
      end loop;

      --  Clear OVERUN flag in 2-Line communication mode because received UInt8
      --  is not read.
      if Current_Data_Direction (This) in D2Lines_RxOnly | D2Lines_FullDuplex
      then  -- right comparison ???
         Clear_Overrun (This);
         Status := HAL.SPI.Err_Error;
      end if;
      Status := HAL.SPI.Ok;
   end Transmit;

   --------------
   -- Transmit --
   --------------

   procedure Transmit
     (This     : in out SPI_Port;
      Outgoing : UInt8)
   is
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      --  ??? right value to compare???
      if Current_Data_Direction (This) = D1Line_Tx  then
         This.Periph.CR1.BIDIOE := True;
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      This.Periph.DR.DR := UInt16 (Outgoing);

      while not Tx_Is_Empty (This) loop
         null;
      end loop;

      while Busy (This) loop
         null;
      end loop;

      --  Clear OVERUN flag in 2-Line communication mode because received UInt8
      --  is not read.
      if Current_Data_Direction (This) in D2Lines_RxOnly | D2Lines_FullDuplex
      then  -- right comparison ???
         Clear_Overrun (This);
      end if;
   end Transmit;

   -------------
   -- Receive --
   -------------

   overriding
   procedure Receive
     (This    : in out SPI_Port;
      Data    : out HAL.SPI.SPI_Data_8b;
      Status  : out HAL.SPI.SPI_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Timeout);
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      Receive_8bit_Mode (This, Data);

      while Busy (This) loop
         null;
      end loop;

      if CRC_Enabled (This) and CRC_Error_Indicated (This) then
         Reset_CRC (This);
         Status := HAL.SPI.Err_Error;
      end if;
      Status := HAL.SPI.Ok;
   end Receive;

   -------------
   -- Receive --
   -------------

   overriding
   procedure Receive
     (This    : in out SPI_Port;
      Data    : out HAL.SPI.SPI_Data_16b;
      Status  : out HAL.SPI.SPI_Status;
      Timeout : Natural := 1000)
   is
      pragma Unreferenced (Timeout);
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      Receive_16bit_Mode (This, Data);

      while Busy (This) loop
         null;
      end loop;

      if CRC_Enabled (This) and CRC_Error_Indicated (This) then
         Reset_CRC (This);
         Status := HAL.SPI.Err_Error;
      end if;
      Status := HAL.SPI.Ok;
   end Receive;

   -------------
   -- Receive --
   -------------

   procedure Receive
     (This     : in out USART_Port;
      Incoming : out UInt8)
   is
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      This.Periph.DR.DR := 0; -- generate clock

      while Rx_Is_Empty (This) loop
         null;
      end loop;

      Incoming := UInt8 (This.Periph.DR.DR);

      if CRC_Enabled (This) then
         while Rx_Is_Empty (This) loop
            null;
         end loop;
         Read_CRC : declare
            Dummy : UInt16;
         begin
            Dummy := This.Periph.DR.DR;
         end Read_CRC;
      end if;

      while Busy (This) loop
         null;
      end loop;

      if CRC_Enabled (This) and CRC_Error_Indicated (This) then
         Reset_CRC (This);
      end if;
   end Receive;

   ----------------------
   -- Transmit_Receive --
   ----------------------

   procedure Transmit_Receive
     (This     : in out SPI_Port;
      Outgoing : UInt8_Buffer;
      Incoming : out UInt8_Buffer;
      Size     : Positive)
   is
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      if Is_Data_Frame_16bit (This) then
         Send_Receive_16bit_Mode (This, Outgoing, Incoming, Size);
      else
         Send_Receive_8bit_Mode (This, Outgoing, Incoming, Size);
      end if;

      --  Read CRC to close CRC calculation process
      if CRC_Enabled (This) then
         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;
         Read_CRC : declare
            Dummy : UInt16;
         begin
            Dummy := This.Periph.DR.DR;
         end Read_CRC;
      end if;

      while Busy (This) loop
         null;
      end loop;

      if CRC_Enabled (This) and CRC_Error_Indicated (This) then
         Reset_CRC (This);
      end if;
   end Transmit_Receive;

   ----------------------
   -- Transmit_Receive --
   ----------------------

   procedure Transmit_Receive
     (This     : in out SPI_Port;
      Outgoing : UInt8;
      Incoming : out UInt8)
   is
   begin
      if CRC_Enabled (This) then
         Reset_CRC (This);
      end if;

      if not Enabled (This) then
         Enable (This);
      end if;

      if Is_Data_Frame_16bit (This) then
         raise Program_Error;
      end if;

      This.Periph.DR.DR := UInt16 (Outgoing);

      --  enable CRC transmission
      if CRC_Enabled (This) then
         This.Periph.CR1.CRCNEXT := True;
      end if;

      --  wait until data is received
      while Rx_Is_Empty (This) loop
         null;
      end loop;

      Incoming := UInt8 (This.Periph.DR.DR);

      --  Read CRC UInt8 to close CRC calculation
      if CRC_Enabled (This) then
         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;
         Read_CRC : declare
            Dummy : UInt16;
         begin
            Dummy := This.Periph.DR.DR;
         end Read_CRC;
      end if;

      while Busy (This) loop
         null;
      end loop;

      if CRC_Enabled (This) and CRC_Error_Indicated (This) then
         Reset_CRC (This);
      end if;
   end Transmit_Receive;

   ---------------------------
   -- Data_Register_Address --
   ---------------------------

   function Data_Register_Address
     (This : SPI_Port)
      return System.Address
   is
   begin
      return This.Periph.DR'Address;
   end Data_Register_Address;

   -----------------------------
   -- Send_Receive_16bit_Mode --
   -----------------------------

   procedure Send_Receive_16bit_Mode
     (This     : in out SPI_Port;
      Outgoing : UInt8_Buffer;
      Incoming : out UInt8_Buffer;
      Size     : Positive)
   is
      Tx_Count : Natural := Size;
      Outgoing_Index : Natural := Outgoing'First;
      Incoming_Index : Natural := Incoming'First;
   begin
      if Current_Mode (This) = Slave or else Tx_Count = 1 then
         This.Periph.DR.DR :=
           As_Half_Word_Pointer (Outgoing (Outgoing_Index)'Address).all;
         Outgoing_Index := Outgoing_Index + 2;
         Tx_Count := Tx_Count - 1;
      end if;

      if Tx_Count = 0 then

         --  enable CRC transmission
         if CRC_Enabled (This) then
            This.Periph.CR1.CRCNEXT := True;
         end if;

         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         As_Half_Word_Pointer (Incoming (Incoming_Index)'Address).all :=
           This.Periph.DR.DR;

         return;
      end if;

      while Tx_Count > 0 loop
         --  wait until we can send data
         while not Tx_Is_Empty (This) loop
            null;
         end loop;

         This.Periph.DR.DR :=
           As_Half_Word_Pointer (Outgoing (Outgoing_Index)'Address).all;
         Outgoing_Index := Outgoing_Index + 2;
         Tx_Count := Tx_Count - 1;

         --  enable CRC transmission
         if Tx_Count = 0 and CRC_Enabled (This) then
            This.Periph.CR1.CRCNEXT := True;
         end if;

         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         As_Half_Word_Pointer (Incoming (Incoming_Index)'Address).all :=
           This.Periph.DR.DR;
         Incoming_Index := Incoming_Index + 2;
      end loop;

      --  receive the last UInt8
      if Current_Mode (This) = Slave then
         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         As_Half_Word_Pointer (Incoming (Incoming_Index)'Address).all :=
           This.Periph.DR.DR;
      end if;
   end Send_Receive_16bit_Mode;

   ----------------------------
   -- Send_Receive_8bit_Mode --
   ----------------------------

   procedure Send_Receive_8bit_Mode
     (This     : in out SPI_Port;
      Outgoing : UInt8_Buffer;
      Incoming : out UInt8_Buffer;
      Size     : Positive)
   is
      Tx_Count : Natural := Size;
      Outgoing_Index : Natural := Outgoing'First;
      Incoming_Index : Natural := Incoming'First;
   begin
      if Current_Mode (This) = Slave or else Tx_Count = 1 then
         This.Periph.DR.DR := UInt16 (Outgoing (Outgoing_Index));
         Outgoing_Index := Outgoing_Index + 1;
         Tx_Count := Tx_Count - 1;
      end if;

      if Tx_Count = 0 then

         --  enable CRC transmission
         if CRC_Enabled (This) then
            This.Periph.CR1.CRCNEXT := True;
         end if;

         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         Incoming (Incoming_Index) := UInt8 (This.Periph.DR.DR);

         return;

      end if;

      while Tx_Count > 0 loop
         --  wait until we can send data
         while not Tx_Is_Empty (This) loop
            null;
         end loop;

         This.Periph.DR.DR := UInt16 (Outgoing (Outgoing_Index));
         Outgoing_Index := Outgoing_Index + 1;
         Tx_Count := Tx_Count - 1;

         --  enable CRC transmission
         if Tx_Count = 0 and CRC_Enabled (This) then
            This.Periph.CR1.CRCNEXT := True;
         end if;

         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         Incoming (Incoming_Index) := UInt8 (This.Periph.DR.DR);
         Incoming_Index := Incoming_Index + 1;
      end loop;

      if Current_Mode (This) = Slave then
         --  wait until data is received
         while Rx_Is_Empty (This) loop
            null;
         end loop;

         Incoming (Incoming_Index) := Data (This);
      end if;
   end Send_Receive_8bit_Mode;

   ---------------------
   -- Send_16bit_Mode --
   ---------------------

   procedure Send_16bit_Mode
     (This     : in out SPI_Port;
      Outgoing : HAL.SPI.SPI_Data_16b)
   is
      Tx_Count : Natural := Outgoing'Length;
      Index    : Natural := Outgoing'First;
   begin
      if Current_Mode (This) = Slave or else Tx_Count = 1 then
         This.Periph.DR.DR :=
           As_Half_Word_Pointer (Outgoing (Index)'Address).all;
         Index := Index + 2;
         Tx_Count := Tx_Count - 1;
      end if;

      while Tx_Count > 0 loop
         --  wait until we can send data
         while not Tx_Is_Empty (This) loop
            null;
         end loop;

         This.Periph.DR.DR :=
           As_Half_Word_Pointer (Outgoing (Index)'Address).all;
         Index := Index + 2;
         Tx_Count := Tx_Count - 1;
      end loop;

      if CRC_Enabled (This) then
         This.Periph.CR1.CRCNEXT := True;
      end if;
   end Send_16bit_Mode;

   --------------------
   -- Send_8bit_Mode --
   --------------------

   procedure Send_8bit_Mode
     (This     : in out SPI_Port;
      Outgoing : HAL.SPI.SPI_Data_8b)
   is
      Tx_Count : Natural := Outgoing'Length;
      Index    : Natural := Outgoing'First;
   begin
      if Current_Mode (This) = Slave or else Tx_Count = 1 then
         This.Periph.DR.DR := UInt16 (Outgoing (Index));
         Index := Index + 1;
         Tx_Count := Tx_Count - 1;
      end if;

      while Tx_Count > 0 loop
         --  wait until we can send data
         while not Tx_Is_Empty (This) loop
            null;
         end loop;

         This.Periph.DR.DR := UInt16 (Outgoing (Index));
         Index := Index + 1;
         Tx_Count := Tx_Count - 1;
      end loop;

      if CRC_Enabled (This) then
         This.Periph.CR1.CRCNEXT := True;
      end if;
   end Send_8bit_Mode;

   ------------------------
   -- Receive_16bit_Mode --
   ------------------------

   procedure Receive_16bit_Mode
     (This     : in out SPI_Port;
      Incoming : out HAL.SPI.SPI_Data_16b)
   is
      Generate_Clock : constant Boolean := Current_Mode (This) = Master;
   begin
      for K of Incoming loop
         if Generate_Clock then
            This.Periph.DR.DR := 0;
         end if;
         while Rx_Is_Empty (This) loop
            null;
         end loop;
         K := This.Periph.DR.DR;
      end loop;
   end Receive_16bit_Mode;

   -----------------------
   -- Receive_8bit_Mode --
   -----------------------

   procedure Receive_8bit_Mode
     (This     : in out SPI_Port;
      Incoming : out HAL.SPI.SPI_Data_8b)
   is
      Generate_Clock : constant Boolean := Current_Mode (This) = Master;
   begin
      for K of Incoming loop
         if Generate_Clock then
            This.Periph.DR.DR := 0;
         end if;
         while Rx_Is_Empty (This) loop
            null;
         end loop;
         K := UInt8 (This.Periph.DR.DR);
      end loop;
   end Receive_8bit_Mode;

end STM32.USART;
