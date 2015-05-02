/*
 * Copyright (c) 2005-2006 Arch Rock Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Arch Rock Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * ARCHED ROCK OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */

/**
 * Implementation of the receive path for the ChipCon CC2420 radio.
 *
 * @author Jonathan Hui <jhui@archrock.com>
 * @version $Revision: 1.4 $ $Date: 2009/08/14 20:33:43 $
 */

configuration QuickTransmitC {

  provides interface StdControl;
  provides interface CC2420Receive;
  provides interface Receive;
  provides interface ReceiveIndicator as PacketIndicator;

}

implementation {
  components MainC;
  components QuickTransmitP;
  components CC2420PacketC;
  components new CC2420SpiC() as Spi;
  components CC2420ControlC;
  
  components HplCC2420PinsC as Pins;
  components HplCC2420InterruptsC as InterruptsC;

  components LedsC as Leds;
  QuickTransmitP.Leds -> Leds;

  StdControl = QuickTransmitP;
  CC2420Receive = QuickTransmitP;
  Receive = QuickTransmitP;
  PacketIndicator = QuickTransmitP.PacketIndicator;

  MainC.SoftwareInit -> QuickTransmitP;
  
  QuickTransmitP.CSN -> Pins.CSN;
  QuickTransmitP.FIFO -> Pins.FIFO;
  QuickTransmitP.FIFOP -> Pins.FIFOP;
  QuickTransmitP.InterruptFIFOP -> InterruptsC.InterruptFIFOP;
  QuickTransmitP.SpiResource -> Spi;
  QuickTransmitP.RXFIFO -> Spi.RXFIFO;
  QuickTransmitP.SFLUSHRX -> Spi.SFLUSHRX;
  QuickTransmitP.SACK -> Spi.SACK;
  QuickTransmitP.CC2420Packet -> CC2420PacketC;
  QuickTransmitP.CC2420PacketBody -> CC2420PacketC;
  QuickTransmitP.PacketTimeStamp -> CC2420PacketC;
  QuickTransmitP.CC2420Config -> CC2420ControlC;

  QuickTransmitP.SECCTRL0 -> Spi.SECCTRL0;
  QuickTransmitP.SECCTRL1 -> Spi.SECCTRL1;
  QuickTransmitP.SRXDEC -> Spi.SRXDEC;
  QuickTransmitP.RXNONCE -> Spi.RXNONCE;
  QuickTransmitP.KEY0 -> Spi.KEY0;
  QuickTransmitP.KEY1 -> Spi.KEY1;
  QuickTransmitP.RXFIFO_RAM -> Spi.RXFIFO_RAM;
  QuickTransmitP.SNOP -> Spi.SNOP;

  components CC2420TransmitC;
  QuickTransmitP.QuickSend -> CC2420TransmitC;
  components new AlarmMicro32C();
  QuickTransmitP.Timeout -> AlarmMicro32C;
}