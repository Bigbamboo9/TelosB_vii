COMPONENT=RadioCountToLedsAppC

CFLAGS += -DENABLE_X
CFLAGS += -DRADIO_X_DEBUG
CFLAGS += -DCC2420_X_DEF_RFPOWER=31

# TOSH_DATA_LENGTH and CC2420_X_PACKET_SIZE should be the same to avoid memory operation error!
CFLAGS += -DTOSH_DATA_LENGTH=77
CFLAGS += -DCC2420_X_PACKET_SIZE=77

PFLAGS += -I../../Dropbox/Telosb_vii/tos/chips/msp430
PFLAGS += -I../../tos/chips/msp430/timer
PFLAGS += -I../../tos/chips/cc2420/x-spi
PFLAGS += -I../../tos/chips/cc2420/x-control
PFLAGS += -I../../tos/chips/cc2420/x-timer
PFLAGS += -I../../tos/chips/cc2420/x-rtx
PFLAGS += -I../../tos/chips/cc2420/x-packet
PFLAGS += -I../../tos/chips/cc2420/x-lpl
PFLAGS += -I../../tos/chips/cc2420/interfaces
PFLAGS += -I../../tos/chips/cc2420

PFLAGS += -I../../tos/platforms/telosa
PFLAGS += -I../../tos/lib/timer
PFLAGS += -I../../tos/system
PFLAGS += -I../../tos/printf

#CFLAGS += -I../../tos/chips/msp430
#CFLAGS += -I../../tos/chips/msp430/timer
#CFLAGS += -I../../tos/chips/cc2420/x-spi
#CFLAGS += -I../../tos/chips/cc2420/x-control
#CFLAGS += -I../../tos/chips/cc2420/x-timer
#CFLAGS += -I../../tos/chips/cc2420/x-rtx
#CFLAGS += -I../../tos/chips/cc2420/x-packet
#CFLAGS += -I../../tos/chips/cc2420/x-lpl
#CFLAGS += -I../../tos/chips/cc2420/interfaces
#CFLAGS += -I../../tos/chips/cc2420

#CFLAGS += -I../../tos/platforms/telosa
#CFLAGS += -I../../tos/lib/timer
#CFLAGS += -I../../tos/system
#CFLAGS += -I../../tos/printf

include $(MAKERULES)
#TINYOS_ROOT_DIR?=/Users/ZhichaoCao/Documents/czc/tinyos-main
#include $(TINYOS_ROOT_DIR)/Makefile.include
