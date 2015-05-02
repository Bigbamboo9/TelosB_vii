COMPONENT=TestPacketCIAppC

CFLAGS += -DUSING_4MHZ_UART
#CFLAGS += -DENABLE_PR
CFLAGS += -DNEW_PRINTF_SEMANTICS
#CFLAGS += -DCONSTRUCTIVE_INTERFERENCE
CFLAGS += -DTOSH_DATA_LENGTH=110
#CFLAGS += -DCC2420_DEF_POWER=2
CFLAGS += -DCC2420_DEF_CHANNEL=20
#PFLAGS += -DENABLE_SPI0_DMA
#CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION

CFLAGS += -I../../tos/chips/msp430/timer
#CFLAGS += -I../../tos/chips/cc2420
#CFLAGS += -I../../tos/chips/cc2420/packet
#CFLAGS += -I../../tos/chips/cc2420/transmit
#CFLAGS += -I../../tos/chips/cc2420/receive
#CFLAGS += -I../../tos/chips/cc2420/csma
#CFLAGS += -I../../tos/chips/cc2420/quick_transmit
#CFLAGS += -I../../tos/platforms/telosa
#CFLAGS += -I../../tos/lib/timer
CFLAGS += -I../../tos/include
CFLAGS += -I$(TOSROOT)/tos/lib/printf

include $(MAKERULES)
