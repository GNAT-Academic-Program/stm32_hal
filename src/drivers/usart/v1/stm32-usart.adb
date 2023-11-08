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

with System; use System;
with STM32_SVD.USART; use STM32_SVD.USART;
with STM32.Device; use STM32.Device;

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
      if This.Periph.all'Address = USART1_Periph'Address
        or else
         This.Periph.all'Address = USART6_Periph'Address
      then
         return Clocks.PCLK2;
      else
         return Clocks.PCLK1;
      end if;
   end APB_Clock;

   ---------------
   -- Configure --
   ---------------

   procedure Configure (
      This : in out USART_Port;
      Conf : USART_Configuration
   ) is
      Clock        : constant UInt32 := APB_Clock (This);
      Int_Scale    : constant UInt32 := (
         if Conf.Oversampling = Oversampling_8x then 2 else 4
      );
      Int_Divider  : constant UInt32 := (
         (25 * Clock) / (Int_Scale * Conf.Baud_Rate)
      );
      Frac_Divider : constant UInt32 := Int_Divider rem 100;
   begin

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

      This.Periph.CR1.M0    := False; -- If true, data_size is 7 bits.
      This.Periph.CR1.M1    := Conf.Data_Size = HAL.UART.Data_Size_9b;
      -- Interrupt CR1 also contains interrupt configuration

      This.Periph.CR2.CLKEN := Conf.Mode = Syncrhonous;
      This.Periph.CR2.STOP := Conf.Stop_Bits'Enum_Rep;
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
              and not Transmission_Is_Complete (This))
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
   -- Tx_Is_Empty --
   -----------------

   function Tx_Is_Complete (This : USART_Port) return Boolean is
   begin
      return This.Periph.ISR.TC;
   end Tx_Is_Complete;

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

   function Current_Mode (This : USART_Port) return USART_Mode is
   begin
      if This.Periph.CR2.CLKEN then
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
      if This.Periph.CR1.TE and This.Periph.CR1.RE then
         return RX_TX;
      elsif This.Periph.CR1.TE then
         return TX;
      else
         return RX;
      end if;
   end Current_Data_Direction;

   ----------------------------
   -- Parity_Error_Indicated --
   ----------------------------

   function Parity_Error_Indicated (This : USART_Port) return Boolean is
     (This.Periph.ISR.PE);

   -----------------------------
   -- Framing_Error_Indicated --
   -----------------------------

   function Framing_Error_Indicated (This : USART_Port) return Boolean is
     (This.Periph.ISR.FE);

   ------------------------------
   -- Start_Bit_Noise_Detected --
   ------------------------------

   function Start_Bit_Noise_Detected (This : USART_Port) return Boolean is
      (This.Periph.ISR.NF);

   -----------------------
   -- Overrun_Indicated --
   -----------------------

   function Overrun_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.ORE);

   -------------------------
   -- Idle_Line_Indicated --
   -------------------------

   function Idle_Line_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.IDLE);

   ------------------------
   -- LIN_Break_Detected --
   ------------------------

   function LIN_Break_Detected (This : USART_Port) return Boolean is
      (This.Periph.ISR.LBDF);

   -----------------------------
   -- CTS_Interrupt_Indicated --
   -----------------------------

   function CTS_Interrupt_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.CTSIF);

   -------------------
   -- CTS_Indicated --
   -------------------

   function CTS_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.CTS);

   --------------------------------
   -- Reciever_Timeout_Indicated --
   --------------------------------

   function Reciever_Timeout_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.RTOF);

   ----------------------------
   -- End_Of_Block_Indicated --
   ----------------------------

   function End_Of_Block_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.EOBF);

   ---------------------------
   -- Auto_Baud_Rate_Failed --
   ---------------------------

   function Auto_Baud_Rate_Failed (This : USART_Port) return Boolean is
      (This.Periph.ISR.ABRE);

   -------------------------------
   -- Auto_Baud_Rate_Successful --
   -------------------------------

   function Auto_Baud_Rate_Successful (This : USART_Port) return Boolean is
      (This.Periph.ISR.ABRF);

   -------------------------------
   -- Character_Match_Indicated --
   -------------------------------

   function Character_Match_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.CMF);

   ----------------------------------
   -- Send_Break_Request_Indicated --
   ----------------------------------

   function Send_Break_Request_Indicated (This : USART_Port) return Boolean is
      (This.Periph.ISR.SBKF);

   -----------------------------
   -- Reciever_Pending_Wakeup --
   -----------------------------

   function Reciever_Pending_Wakeup (This : USART_Port) return Boolean is
      (This.Periph.ISR.RWU);

   ----------------------------------
   -- Transmit_Enable_Acknowledged --
   ----------------------------------

   function Transmit_Enable_Acknowledged (This : USART_Port) return Boolean is
      (This.Periph.ISR.TEACK);

   -------------------
   -- Clear_Overrun --
   -------------------

   procedure Clear_Overrun (This : USART_Port) is
   begin
      This.Periph.ICR.ORECF := True;
   end Clear_Overrun;

   -------------------------
   -- Is_Data_Frame_9bit --
   -------------------------

   function Is_Data_Frame_9bit (This : USART_Port) return Boolean is
      (This.Periph.CR1.M1);

   ---------------
   -- Data_Size --
   ---------------

   overriding
   function Data_Size (This : USART_Port) return HAL.UART.UART_Data_Size is
   begin
      if This.Periph.CR1.M1 then
         return HAL.UART.Data_Size_9b;
      else
         return HAL.UART.Data_Size_8b;
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
      if not Enabled (This) then
         Enable (This);
      end if;

      Receive_8bit_Mode (This, Data);

      while Busy (This) loop
         null;
      end loop;

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

      if not Enabled (This) then
         Enable (This);
      end if;

      Receive_16bit_Mode (This, Data);

      while Busy (This) loop
         null;
      end loop;

      Status := HAL.SPI.Ok;
   end Receive;

   -------------
   -- Receive --
   -------------

   procedure Receive
     (This     : in out SPI_Port;
      Incoming : out UInt8)
   is
   begin

      if not Enabled (This) then
         Enable (This);
      end if;

      This.Periph.DR.DR := 0; -- generate clock

      while Rx_Is_Empty (This) loop
         null;
      end loop;

      Incoming := UInt8 (This.Periph.DR.DR);

      while Busy (This) loop
         null;
      end loop;
   end Receive;

   --  ----------------------
   --  -- Transmit_Receive --
   --  ----------------------

   --  procedure Transmit_Receive
   --    (This     : in out SPI_Port;
   --     Outgoing : UInt8_Buffer;
   --     Incoming : out UInt8_Buffer;
   --     Size     : Positive)
   --  is
   --  begin
   --     if not Enabled (This) then
   --        Enable (This);
   --     end if;

   --     if Is_Data_Frame_16bit (This) then
   --        Send_Receive_16bit_Mode (This, Outgoing, Incoming, Size);
   --     else
   --        Send_Receive_8bit_Mode (This, Outgoing, Incoming, Size);
   --     end if;

   --     while Busy (This) loop
   --        null;
   --     end loop;
   --  end Transmit_Receive;

   --  ----------------------
   --  -- Transmit_Receive --
   --  ----------------------

   --  procedure Transmit_Receive
   --    (This     : in out SPI_Port;
   --     Outgoing : UInt8;
   --     Incoming : out UInt8)
   --  is
   --  begin

   --     if not Enabled (This) then
   --        Enable (This);
   --     end if;

   --     if Is_Data_Frame_16bit (This) then
   --        raise Program_Error;
   --     end if;

   --     This.Periph.DR.DR := UInt16 (Outgoing);

   --     --  wait until data is received
   --     while Rx_Is_Empty (This) loop
   --        null;
   --     end loop;

   --     Incoming := UInt8 (This.Periph.DR.DR);

   --     while Busy (This) loop
   --        null;
   --     end loop;
   --  end Transmit_Receive;

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

   procedure Send_Receive_9bit_Mode
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
   end Send_Receive_9bit_Mode;

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

   procedure Send_9bit_Mode
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
   end Send_9bit_Mode;

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
   end Send_8bit_Mode;

   ------------------------
   -- Receive_16bit_Mode --
   ------------------------

   procedure Receive_9bit_Mode
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
   end Receive_9bit_Mode;

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
