COMPONENT=TestGlossyAppC

CFLAGS += -DUSING_4MHZ_UART
CFLAGS += -DENABLE_PR
CFLAGS += -DENABLE_GLOSSY
CFLAGS += -DENABLE_X
CFLAGS += -DDATA_LENGTH=40
CFLAGS += -DTOSH_DATA_LENGTH=110
CFLAGS += -DCC2420_DEF_RFPOWER=1
CFLAGS += -DCC2420_DEF_CHANNEL=26
#CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION

CFLAGS += -I../../tos/platforms/telosa
#CFLAGS += -I../../tos/chips/cc2420/x-glossyrtx/bckup/workedVer
CFLAGS += -I../../tos/chips/cc2420/x-glossyrtx
CFLAGS += -I../../tos/chips/cc2420/alarm
CFLAGS += -I../../tos/chips/cc2420/spi
CFLAGS += -I../../tos/chips/cc2420/control
CFLAGS += -I../../tos/chips/msp430/timer
CFLAGS += -I../../tos/chips/msp430/usart
CFLAGS += -I../../tos/chips/msp430/pins
CFLAGS += -I../../tos/system
CFLAGS += -I../../tos/lib/timer
CFLAGS += -I../../tos/include
CFLAGS += -I$(TOSROOT)/tos/lib/printf

include $(MAKERULES)
