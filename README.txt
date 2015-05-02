***DESCRIPTION***

This software stack is motivated to implement and utilize the constructive interference for bulk data dissemination process.
The modification relates to three parts:
1. The timer subsystem of MCU. Using high frequency DCO timer rather than the 32khz timer.
2. The rx/tx of the radio stack. Fulfilling the timeing requirement of the constructive interference.
3. Add the sender selection and loss repair control module into the Deluge 2.x.

***NOTICE***

1. Timer Subsystem (tos/chips/msp430/timer)
  a. Virtual High-resolution Time (VHT) to ensure the reliability of the DCO timer.
  b. The influence of SPI speed incured by change the SMCLK frequecy? It seems the influence is not considerable currently.
  c. The synchronization skew compensation of VHT.

2. Radio Stack (tos/chips/cc2420)
  Based on the compensation concern of Glossy, there are three main aspects should be considered :
  a. Software execution jitter. The software executions should be minimize as much as possible.
  b. Timer fired interrupt compensation. Insert certain NOP instructions.
  c. The jitter of interaction between MCU and radio. Insert constant NOP instructions. (Acctually, I do not understand this
     point very clearly.)

3. Protocol Design

***OBSERVATION***

5/28,2012
1. Based on the current CC2420 radio stack (./receive ./transmit ./quick_transmit), the difference of the SFD intterupt from 
   two transmiters could be reduced to 4us. More specifically, two trasmiters forward the received packet from a initiator 
   immediately as the Glossy does (without sync skew). The data is read out from RXFIFO buffer after the FIFOP Interrupt (the 
   falling edge interrupt of SFD). We could not strobe the STXON and then load the data in to TXFIFO buffer. The underflow 
   is often happened. This motivates us to implement the rx/tx files.

7/19,2012
2. Finish the basic Glossy function implementation in current TinyOS 2.1.1 system (./tos/chips/cc2420/x-glossyrtx). Some exper-
   iences are seen the first time. 1) "atomic" keyword is actually to disable all interrupt. 2) the function excution by "call"
   could incur tens of ticks delay. DO NOT use "call" or signal "event" when the timing reqirement is strict. It is better to
   control the register and interrupt vector directly. 3) It seems (a > b) could incur compiler error rather than ((signed)(a - 
   b) > 0).
   In the next, I mainly focus on merging a normal radio rx/tx function into the current gRTx stack. Then I will implement the
   PRR test application.
