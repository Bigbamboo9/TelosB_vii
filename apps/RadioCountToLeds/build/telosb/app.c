#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 303
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 347
static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 42 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memcpy(void *arg_0x2ac07467fbf0, const void *arg_0x2ac07467d020, size_t arg_0x2ac07467d2c8);

extern void *memset(void *arg_0x2ac07467c980, int arg_0x2ac07467cbe8, size_t arg_0x2ac074683020);
#line 65
extern void *memset(void *arg_0x2ac074695b10, int arg_0x2ac074695d78, size_t arg_0x2ac074694060);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2ac0746d8290);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2ac0746dd300);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/opt/tinyos-main-read-only/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/opt/tinyos-main-read-only/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 145 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char IE1 __asm ("__""IE1");









extern volatile unsigned char IFG1 __asm ("__""IFG1");








extern volatile unsigned char ME1 __asm ("__""ME1");






extern volatile unsigned char IE2 __asm ("__""IE2");





extern volatile unsigned char IFG2 __asm ("__""IFG2");





extern volatile unsigned char ME2 __asm ("__""ME2");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 265
extern const volatile unsigned char P1IN __asm ("__""P1IN");

extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");

extern volatile unsigned char P1IFG __asm ("__""P1IFG");

extern volatile unsigned char P1IES __asm ("__""P1IES");

extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");





extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");


extern const volatile unsigned char P4IN __asm ("__""P4IN");

extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 382
extern volatile unsigned char U0CTL __asm ("__""U0CTL");

extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");



extern volatile unsigned char U0MCTL __asm ("__""U0MCTL");

extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");

extern volatile unsigned char U0TXBUF __asm ("__""U0TXBUF");
#line 439
extern volatile unsigned char U1CTL __asm ("__""U1CTL");

extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");



extern volatile unsigned char U1MCTL __asm ("__""U1MCTL");

extern volatile unsigned char U1BR0 __asm ("__""U1BR0");

extern volatile unsigned char U1BR1 __asm ("__""U1BR1");



extern volatile unsigned char U1TXBUF __asm ("__""U1TXBUF");
#line 595
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");





extern volatile unsigned int TACCR2 __asm ("__""TACCR2");
#line 716
extern const volatile unsigned int TBIV __asm ("__""TBIV");

extern volatile unsigned int TBCTL __asm ("__""TBCTL");

extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");

extern volatile unsigned int TBCCTL1 __asm ("__""TBCCTL1");

extern volatile unsigned int TBCCTL2 __asm ("__""TBCCTL2");

extern volatile unsigned int TBCCTL3 __asm ("__""TBCCTL3");

extern volatile unsigned int TBCCTL4 __asm ("__""TBCCTL4");

extern volatile unsigned int TBCCTL5 __asm ("__""TBCCTL5");

extern volatile unsigned int TBCCTL6 __asm ("__""TBCCTL6");

extern volatile unsigned int TBR __asm ("__""TBR");



extern volatile unsigned int TBCCR1 __asm ("__""TBCCR1");

extern volatile unsigned int TBCCR2 __asm ("__""TBCCR2");

extern volatile unsigned int TBCCR3 __asm ("__""TBCCR3");



extern volatile unsigned int TBCCR5 __asm ("__""TBCCR5");

extern volatile unsigned int TBCCR6 __asm ("__""TBCCR6");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 977
extern volatile unsigned char CACTL1 __asm ("__""CACTL1");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
#line 1395
extern volatile unsigned int DMA0CTL __asm ("__""DMA0CTL");

extern volatile unsigned int DMA1CTL __asm ("__""DMA1CTL");

extern volatile unsigned int DMA2CTL __asm ("__""DMA2CTL");
# 343 "/opt/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/opt/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4251 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4252 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 36 "RadioCountToLeds.h"
#line 33
typedef nx_struct radio_count_msg {
  nx_uint16_t counter;
  nx_uint32_t time;
} __attribute__((packed)) radio_count_msg_t;

enum __nesc_unnamed4253 {
  AM_RADIO_COUNT_MSG = 6
};
# 28 "../../tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4254 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4255 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4256 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4257 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 43 "/opt/tinyos-main-read-only/tos/types/Leds.h"
enum __nesc_unnamed4258 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 6 "/opt/tinyos-main-read-only/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4259 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4260 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 39 "../../tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4261 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4262 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 83 "/opt/tinyos-main-read-only/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4263 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4264 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4265 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/opt/tinyos-main-read-only/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-main-read-only/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 67 "../../tos/chips/cc2420/x-rtx/cc2420_x_rtx.h"
#line 54
typedef enum __nesc_unnamed4266 {
  S_RTX_IDLE, 

  S_TX_DETECT, 
  S_TX_SFD, 
  S_TX_ACK, 

  S_RX_DETECT, 
  S_RX_RECEIVE, 
  S_RX_ACK, 

  S_CI_SFD, 
  S_CI_ACK
} cc2420_rtx_state_t;
#line 85
#line 69
typedef nx_struct __nesc_unnamed4267 {

  nx_bool batched;

  nx_bool ack;

  nx_bool priority;

  nx_uint8_t size;
  nx_uint16_t addr;
  nx_uint16_t metric;
  nx_uint16_t progress;
  nx_bool ci;

  nx_uint8_t hop;
  nx_uint8_t preamble_dsn;
} __attribute__((packed)) rtx_setting_t;







#line 87
typedef struct __nesc_unnamed4268 {
  bool received;
  uint8_t max_size;
  uint8_t occ_size;
  uint8_t pos_buf;
  message_t *p_rx_buf;
} rx_buffer_t;
#line 117
#line 95
typedef struct __nesc_unnamed4269 {

  uint16_t calibration_factor;

  uint16_t pkt_rtx_time;
  uint16_t ack_time;
  uint16_t turnaround_time;

  uint8_t pkt_recv;
  uint8_t pkt_send;

  uint32_t channel_detection;

  uint8_t pkt_ack;

  uint8_t pkt_turnaround;

  uint32_t radio_on_time;
  uint32_t tail_total_time;
  uint32_t rtx_total_time;
  uint32_t ack_total_time;
  uint32_t turnaround_total_time;
} rtx_time_compensation_t;

static __inline void timer_initialization();
#line 143
static __inline void msp430_sync_dco();
# 15 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static inline void cc2420_spi_init();
#line 37
static __inline void fast_read_one(uint8_t *buf);
#line 53
static __inline void fast_continue_read_one(uint8_t *buf);






static __inline void fast_read_any(uint8_t *buf, uint8_t len);
#line 81
static __inline void fast_write_any(uint8_t *buf, uint8_t len);
#line 103
static __inline uint8_t strobe(uint8_t reg);
#line 118
static __inline uint16_t get_register(uint8_t reg);
#line 141
static __inline void set_register(uint8_t reg, uint16_t val);
#line 188
static __inline void write_ram(uint16_t addr, uint16_t offset, uint8_t *data, uint8_t len);
# 17 "../../tos/chips/cc2420/x-timer/cc2420_x_timer.h"
static inline void clock_delay(unsigned int i);




static __inline void tx_time_update(rtx_time_compensation_t *rtc, uint16_t time);



static __inline void rx_time_update(rtx_time_compensation_t *rtc, uint16_t time);



static __inline void ack_time_update(rtx_time_compensation_t *rtc, uint16_t time);



static __inline void turnaround_time_update(rtx_time_compensation_t *rtc, uint16_t time);
# 46 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
enum cc2420_status_byte {
  CC2420_XOSC16M_STABLE = 6, 
  CC2420_TX_UNDERFLOW = 5, 
  CC2420_ENC_BUSY = 4, 
  CC2420_TX_ACTIVE = 3, 
  CC2420_LOCK = 2, 
  CC2420_RSSI_VALID = 1
};







#line 56
typedef struct __nesc_unnamed4270 {
  uint16_t ie1;
  uint16_t ie2;
  uint16_t p1ie;
  uint16_t p2ie;
} interrupt_status_t;

static __inline void disable_other_interrupts(interrupt_status_t *status);
#line 96
static __inline void enable_other_interrupts(interrupt_status_t *status);
#line 123
static __inline void cc2420_tx_setting();






static __inline void cc2420_channel_setting();






static __inline void cc2420_mod_setting();
#line 149
static __inline void cc2420_rx_setting();






static __inline void cc2420_sec_setting();






static __inline void cc2420_io_setting();







static __inline int cc2420_get_rssi();









static __inline void radio_flush_rx();






static __inline void radio_flush_tx();



static inline void cc2420_init();
#line 231
static __inline void cc2420_rx_start();



static __inline void cc2420_rx_stop();
# 40 "/opt/tinyos-main-read-only/tos/types/IeeeEui64.h"
enum __nesc_unnamed4271 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/opt/tinyos-main-read-only/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4272 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4273 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4274 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4275 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 86
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum ieee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 9 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t *get_packet_header(message_t *m);



static __inline uint8_t *get_packet_setting(message_t *m);



static __inline uint8_t *get_packet_payload(message_t *m);
#line 29
static __inline uint8_t get_packet_payloadLen(message_t *m);



static __inline uint8_t get_packet_maxPayloadLen();



static __inline uint8_t get_packet_bulk(message_t *m);






static __inline void set_packet_header(message_t *m, uint8_t dsn);
#line 62
static __inline void set_packet_dest(message_t *m, am_addr_t dest);
#line 75
static __inline void set_packet_bulk(message_t *m, uint8_t size);
#line 90
static __inline void set_payload_length(message_t *m, uint8_t len);



static __inline void set_tx_setting(message_t *m, rtx_setting_t *ts);
# 7 "../../tos/printf/serial_fast_print.h"
static inline void uart_init();
#line 30
static __inline void uart_fast_tx(uint8_t byte);
#line 45
static __inline void printf_u16(uint8_t var, uint16_t *word);
# 41 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4276 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4277 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4278 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4279 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 20 "../../tos/chips/cc2420/x-lpl/cc2420_x_lpl.h"
#line 9
typedef enum __nesc_unnamed4280 {

  LPL_X_CLOSE, 

  LPL_X_IDLE, 



  LPL_X_RX, 

  LPL_X_TX
} lpl_x_state_t;
typedef TMilli RadioCountToLedsC__LocalTime__precision_tag;
typedef TMilli RadioCountToLedsC__MilliTimer__precision_tag;
enum AMQueueP____nesc_unnamed4281 {
  AMQueueP__NUM_CLIENTS = 1U
};
typedef TMilli CC2420xLplP__SleepTimer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4282 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef uint16_t RandomMlcgC__SeedInit__parameter;
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 35 "../../tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ac074c8a458);
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ac074c8a458);
# 33 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 30 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t delta);
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t CC2420xRTxP__Init__init(void );
# 3 "../../tos/chips/cc2420/interfaces/LplSend.nc"
static error_t CC2420xRTxP__LplSend__send(message_t *msg, rtx_setting_t *ts);
# 5 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
static error_t CC2420xRTxP__LplReceive__rxInit(void );
#line 3
static void CC2420xRTxP__LplReceive__rxOn(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 61 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );





static void LedsP__Leds__led0Toggle(void );




static void LedsP__Leds__led1On(void );




static void LedsP__Leds__led1Off(void );
#line 94
static void LedsP__Leds__led2Off(void );
#line 56
static void LedsP__Leds__led0On(void );
#line 89
static void LedsP__Leds__led2On(void );
# 58 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
# 42 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 76 "/opt/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ac074ba7d50);
# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ac074ba7d50);
# 57 "/opt/tinyos-main-read-only/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 60 "/opt/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void RadioCountToLedsC__Boot__booted(void );
# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void RadioCountToLedsC__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



RadioCountToLedsC__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void RadioCountToLedsC__MilliTimer__fired(void );
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 48 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2ac0752df020, 
# 103 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 46 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ac0752e1e18, 
# 67 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 46 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ac0752e1e18, 
# 96 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 75
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP__AMSend__send(
# 6 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac075324818, 
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__default__receive(
# 8 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac07535b670, 
# 71 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



CC2420ActiveMessageP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__default__receive(
# 7 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac07535ca90, 
# 71 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 4 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
static void CC2420ActiveMessageP__BulkSend__sendDone(message_t *msg, error_t err);
# 5 "../../tos/chips/cc2420/interfaces/LplSend.nc"
static void CC2420xLplP__SubSend__sendDone(message_t *msg, rtx_setting_t *ts, error_t error);
# 9 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
static void CC2420xLplP__SubReceive__receive(message_t *msg, uint8_t size);
# 3 "../../tos/chips/cc2420/x-timer/RadioTimerUpdate.nc"
static void CC2420xLplP__RadioTimerUpdate__default__counterUpdate(uint32_t count, uint16_t factor);
#line 2
static void CC2420xLplP__RadioTimerUpdate__default__triggerUpdate(void );
# 95 "/opt/tinyos-main-read-only/tos/interfaces/StdControl.nc"
static error_t CC2420xLplP__RadioControl__start(void );
# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t CC2420xLplP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t CC2420xLplP__Init__init(void );
# 2 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
static error_t CC2420xLplP__BulkSend__send(message_t *msg, uint8_t len);
# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void CC2420xLplP__SleepTimer__fired(void );
# 2 "../../tos/chips/cc2420/interfaces/LplTime.nc"
static void CC2420xLplP__LplTime__timeRadio(rtx_time_compensation_t *p_rtx_time);
static void CC2420xLplP__LplTime__timeCompensated(uint16_t time, rtx_time_compensation_t *p_rtx_time);
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "../../tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ac0755542f8);
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "../../tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ac0755542f8, 
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "../../tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ac0755542f8, 
# 73 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 48 "../../tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ac0755542f8);
# 61 "/opt/tinyos-main-read-only/tos/lib/timer/LocalTime.nc"
static uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void );
# 82 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 4 "../../tos/chips/cc2420/interfaces/LplxPacket.nc"
static void CC2420xPacketP__LplxPacket__setPacketBulk(message_t *m, uint8_t size);
static uint8_t CC2420xPacketP__LplxPacket__getPacketBulk(message_t *m);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
static uint8_t CC2420xPacketP__Packet__payloadLength(
#line 74
message_t * msg);
#line 126
static 
#line 123
void * 


CC2420xPacketP__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
#line 106
static uint8_t CC2420xPacketP__Packet__maxPayloadLength(void );
#line 94
static void CC2420xPacketP__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 68 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
static am_addr_t CC2420xPacketP__AMPacket__address(void );









static am_addr_t CC2420xPacketP__AMPacket__destination(
#line 74
message_t * amsg);
#line 103
static void CC2420xPacketP__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t CC2420xPacketP__AMPacket__type(
#line 143
message_t * amsg);
#line 162
static void CC2420xPacketP__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
#line 136
static bool CC2420xPacketP__AMPacket__isForMe(
#line 133
message_t * amsg);
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/opt/tinyos-main-read-only/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/opt/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "../../tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 43 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4283 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
#line 74
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 97
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 114
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 137
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 165
static inline void Msp430ClockP__startTimerB(void );
#line 236
static inline void Msp430ClockP__msp430_init_dco(void );
#line 284
static inline error_t Msp430ClockP__Init__init(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ac074c8a458);
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 51 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x2ac074c8a458);
# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 115 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void );
# 44 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x);
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 44 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;


static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 14 "../../tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void CC2420xRTxP__VectorTimerB1__fired(void );
# 5 "../../tos/chips/cc2420/interfaces/LplSend.nc"
static void CC2420xRTxP__LplSend__sendDone(message_t *msg, rtx_setting_t *ts, error_t error);
# 2 "../../tos/chips/cc2420/interfaces/LplTime.nc"
static void CC2420xRTxP__LplTime__timeRadio(rtx_time_compensation_t *p_rtx_time);
static void CC2420xRTxP__LplTime__timeCompensated(uint16_t time, rtx_time_compensation_t *p_rtx_time);
# 9 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
static void CC2420xRTxP__LplReceive__receive(message_t *msg, uint8_t size);
# 17 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
cc2420_rtx_state_t CC2420xRTxP__rtx_status;
uint8_t CC2420xRTxP__pkt_length;
uint16_t CC2420xRTxP__tbiv;



uint16_t CC2420xRTxP__rx_duration;

uint16_t CC2420xRTxP__tx_duration;

uint16_t CC2420xRTxP__ack_duration;

uint16_t CC2420xRTxP__turn_around;

uint16_t CC2420xRTxP__detect_duration;

uint16_t CC2420xRTxP__abortion_duration;
rtx_time_compensation_t CC2420xRTxP__rtx_time;

uint16_t CC2420xRTxP__tb1_buffer;


int CC2420xRTxP__noise_floor;








message_t *CC2420xRTxP__p_tx_buf;

message_t *CC2420xRTxP__swap_tx_buf;

uint8_t CC2420xRTxP__tx_counter;

rtx_setting_t *CC2420xRTxP__tx_setting;

message_t CC2420xRTxP__rx_buf[7];
rx_buffer_t CC2420xRTxP__m_rx_buf;

uint8_t CC2420xRTxP__rx_readbytes;

rtx_setting_t CC2420xRTxP__rx_setting;

uint16_t CC2420xRTxP__local_metric;




static __inline void CC2420xRTxP__cc2420_signal_detect(uint16_t time);

static __inline void CC2420xRTxP__cc2420_begin_rx(void );

static __inline void CC2420xRTxP__cc2420_end_rx(void );

static __inline void CC2420xRTxP__cc2420_ack_strobe_rx(void );



static __inline void CC2420xRTxP__cc2420_load_tx(void );

static __inline void CC2420xRTxP__cc2420_strobe_tx(void );

static __inline void CC2420xRTxP__cc2420_ack_wait_tx(void );

static __inline void CC2420xRTxP__cc2420_ack_rx(void );

static __inline void CC2420xRTxP__cc2420_ack_rx_except(void );

static inline error_t CC2420xRTxP__Init__init(void );
#line 114
static error_t CC2420xRTxP__LplSend__send(message_t *msg, rtx_setting_t *ts);
#line 135
static inline void CC2420xRTxP__LplReceive__rxOn(void );
#line 151
static error_t CC2420xRTxP__LplReceive__rxInit(void );
#line 170
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
#line 692
static __inline void CC2420xRTxP__cc2420_signal_detect(uint16_t time);
#line 785
static __inline void CC2420xRTxP__cc2420_begin_rx(void );
#line 824
static __inline void CC2420xRTxP__cc2420_end_rx(void );
#line 875
static __inline void CC2420xRTxP__cc2420_ack_strobe_rx(void );








static __inline void CC2420xRTxP__cc2420_load_tx(void );



static __inline void CC2420xRTxP__cc2420_strobe_tx(void );








static __inline void CC2420xRTxP__cc2420_ack_wait_tx(void );










static __inline void CC2420xRTxP__cc2420_ack_rx(void );
#line 977
static __inline void CC2420xRTxP__cc2420_ack_rx_except(void );
# 42 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 56 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 74
static inline void LedsP__Leds__led0On(void );




static inline void LedsP__Leds__led0Off(void );




static inline void LedsP__Leds__led0Toggle(void );




static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );









static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );
# 56 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 58 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "../../tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 111
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/opt/tinyos-main-read-only/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/opt/tinyos-main-read-only/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/opt/tinyos-main-read-only/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ac074ba7d50);
# 76 "/opt/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4284 {

  SchedulerBasicP__NUM_TASKS = 4U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 95 "/opt/tinyos-main-read-only/tos/interfaces/StdControl.nc"
static error_t RadioCountToLedsC__AMControl__start(void );
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t RadioCountToLedsC__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 61 "/opt/tinyos-main-read-only/tos/lib/timer/LocalTime.nc"
static uint32_t RadioCountToLedsC__LocalTime__get(void );
# 126 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
static 
#line 123
void * 


RadioCountToLedsC__Packet__getPayload(
#line 121
message_t * msg, 




uint8_t len);
# 4 "../../tos/chips/cc2420/interfaces/LplxPacket.nc"
static void RadioCountToLedsC__LplxPacket__setPacketBulk(message_t *m, uint8_t size);
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void RadioCountToLedsC__MilliTimer__startPeriodic(uint32_t dt);
# 61 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
static void RadioCountToLedsC__Leds__led0Off(void );





static void RadioCountToLedsC__Leds__led0Toggle(void );




static void RadioCountToLedsC__Leds__led1On(void );




static void RadioCountToLedsC__Leds__led1Off(void );
#line 94
static void RadioCountToLedsC__Leds__led2Off(void );
#line 56
static void RadioCountToLedsC__Leds__led0On(void );
#line 89
static void RadioCountToLedsC__Leds__led2On(void );
# 62 "RadioCountToLedsC.nc"
message_t RadioCountToLedsC__packet;

bool RadioCountToLedsC__locked;
uint16_t RadioCountToLedsC__counter = 0;

static inline void RadioCountToLedsC__Boot__booted(void );








static inline void RadioCountToLedsC__MilliTimer__fired(void );
#line 94
static inline message_t *RadioCountToLedsC__Receive__receive(message_t *bufPtr, void *payload, uint8_t len);
#line 118
static inline void RadioCountToLedsC__AMSend__sendDone(message_t *bufPtr, error_t error);
# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 103 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 162
static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
# 53 "/opt/tinyos-main-read-only/tos/system/AMQueueEntryP.nc"
static inline error_t /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 48 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2ac0752df020, 
# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 46 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ac0752e1e18, 
# 96 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 78 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 147
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 143
message_t * amsg);
# 126 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4285 {
#line 126
  AMQueueImplP__0__CancelTask = 0U
};
#line 126
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 169
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4286 {
#line 169
  AMQueueImplP__0__errorTask = 1U
};
#line 169
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 57
#line 55
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4287 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 90
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 163
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 189
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP__AMSend__sendDone(
# 6 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac075324818, 
# 103 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Snoop__receive(
# 8 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac07535b670, 
# 71 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 5 "../../tos/chips/cc2420/interfaces/LplxPacket.nc"
static uint8_t CC2420ActiveMessageP__LplxPacket__getPacketBulk(message_t *m);
# 106 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP__Receive__receive(
# 7 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2ac07535ca90, 
# 71 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 2 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
static error_t CC2420ActiveMessageP__BulkSend__send(message_t *msg, uint8_t len);
# 103 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
static void CC2420ActiveMessageP__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t CC2420ActiveMessageP__AMPacket__type(
#line 143
message_t * amsg);
#line 162
static void CC2420ActiveMessageP__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
#line 136
static bool CC2420ActiveMessageP__AMPacket__isForMe(
#line 133
message_t * amsg);
# 25 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len);
#line 57
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);




static inline void CC2420ActiveMessageP__BulkSend__sendDone(message_t *msg, error_t result);




static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 81
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);


static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);
# 3 "../../tos/chips/cc2420/interfaces/LplSend.nc"
static error_t CC2420xLplP__SubSend__send(message_t *msg, rtx_setting_t *ts);
# 5 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
static error_t CC2420xLplP__SubReceive__rxInit(void );
#line 3
static void CC2420xLplP__SubReceive__rxOn(void );
# 3 "../../tos/chips/cc2420/x-timer/RadioTimerUpdate.nc"
static void CC2420xLplP__RadioTimerUpdate__counterUpdate(uint32_t count, uint16_t factor);
#line 2
static void CC2420xLplP__RadioTimerUpdate__triggerUpdate(void );
# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
static void CC2420xLplP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420xLplP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 4 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
static void CC2420xLplP__BulkSend__sendDone(message_t *msg, error_t err);
# 73 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void CC2420xLplP__SleepTimer__startOneShot(uint32_t dt);




static void CC2420xLplP__SleepTimer__stop(void );
# 24 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
lpl_x_state_t CC2420xLplP__lpl_status;
interrupt_status_t CC2420xLplP__ie_status;
rtx_setting_t CC2420xLplP__tx_status;
uint8_t CC2420xLplP__lpl_dsn;
uint32_t CC2420xLplP__radio_time_perround;
uint32_t CC2420xLplP__radio_start_time;

uint16_t CC2420xLplP__print_low;
uint16_t CC2420xLplP__print_high;

static inline error_t CC2420xLplP__Init__init(void );





static inline error_t CC2420xLplP__RadioControl__start(void );
#line 59
static inline void CC2420xLplP__SleepTimer__fired(void );
#line 74
static inline error_t CC2420xLplP__Send__send(message_t *msg, uint8_t len);
#line 117
static inline error_t CC2420xLplP__BulkSend__send(message_t *msg, uint8_t len);
#line 154
static void CC2420xLplP__SubSend__sendDone(message_t *msg, rtx_setting_t *ts, error_t error);
#line 170
static void CC2420xLplP__SubReceive__receive(message_t *msg, uint8_t size);
#line 185
static void CC2420xLplP__LplTime__timeRadio(rtx_time_compensation_t *rtx_time);
#line 208
static void CC2420xLplP__LplTime__timeCompensated(uint16_t time, rtx_time_compensation_t *rtx_time);
#line 222
static inline void CC2420xLplP__RadioTimerUpdate__default__triggerUpdate(void );
static inline void CC2420xLplP__RadioTimerUpdate__default__counterUpdate(uint32_t count, uint16_t factor);
# 30 "../../tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 43 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 55
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 104
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "../../tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 69 "../../tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4288 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 135
static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 78 "../../tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4289 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );









static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 142
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 157
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 172
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 76 "../../tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4290 {
#line 76
  AlarmToTimerC__0__fired = 2U
};
#line 76
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 73
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 97
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);




static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "../../tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2ac0755542f8);
#line 72
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4291 {
#line 72
  VirtualizeTimerC__0__updateFromTimer = 3U
};
#line 72
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 54
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4292 {

  VirtualizeTimerC__0__NUM_TIMERS = 2U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 60
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4293 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 101
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 141
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);
#line 213
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get(void );
# 53 "/opt/tinyos-main-read-only/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void );




static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 52 "/opt/tinyos-main-read-only/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
# 18 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void CC2420xPacketP__LplxPacket__setPacketBulk(message_t *m, uint8_t size);



static inline uint8_t CC2420xPacketP__LplxPacket__getPacketBulk(message_t *m);
#line 44
static inline am_addr_t CC2420xPacketP__AMPacket__address(void );



static inline am_addr_t CC2420xPacketP__AMPacket__destination(message_t *amsg);









static inline void CC2420xPacketP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);








static inline bool CC2420xPacketP__AMPacket__isForMe(message_t *amsg);









static inline am_id_t CC2420xPacketP__AMPacket__type(message_t *amsg);




static inline void CC2420xPacketP__AMPacket__setType(message_t *amsg, am_id_t type);
#line 105
static inline uint8_t CC2420xPacketP__Packet__payloadLength(message_t *msg);



static inline void CC2420xPacketP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420xPacketP__Packet__maxPayloadLength(void );



static inline void *CC2420xPacketP__Packet__getPayload(message_t *msg, uint8_t len);
# 397 "/opt/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 185 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 104 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 82 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 53 "../../tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2ac074c8a458){
#line 28
  switch (arg_0x2ac074c8a458) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2ac074c8a458);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 58 "/opt/tinyos-main-read-only/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 172 "../../tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
#line 172
{
}

# 82 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 47 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4294 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 83 "../../tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 157 "../../tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 124 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void )
{
  * (volatile uint16_t * )354U &= ~0x0010;
}

# 47 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents();
#line 47
}
#line 47
# 60 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 97 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "../../tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 70 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )352U & 1U;
}

# 35 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "../../tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 119 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void )
{
  * (volatile uint16_t * )354U |= 0x0010;
}

# 46 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents();
#line 46
}
#line 46
# 84 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )354U &= ~0x0001;
}

# 33 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 30 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )370U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get() + x;
}

# 32 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 71 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 77
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 84
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 87
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 89
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 47 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4295 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4296 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 185 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 37 "../../tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
}
#line 37
# 126 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2ac074c8a458){
#line 28
  switch (arg_0x2ac074c8a458) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2ac074c8a458);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4297 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "../../tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void )
{
}

# 34 "../../tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired();
#line 34
}
#line 34
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

#line 303
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 303
{
#line 303
  return __nesc_ntoh_uint8(source);
}

#line 303
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 303
{
#line 303
  __nesc_hton_uint8(target, value);
#line 303
  return value;
}

#line 292
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

#line 322
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

# 188 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void write_ram(uint16_t addr, uint16_t offset, uint8_t *data, uint8_t len)
#line 188
{
  uint8_t idx;
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  addr += offset;
  U0TXBUF = (addr & 0x7f) | 0x80;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = (addr >> 1) & 0xc0;

  for (idx = 0; idx < len; idx++) {
      while (!(IFG1 & 0x40)) ;
      tmp = U0RXBUF;
      while (!(IFG1 & 0x80)) ;
      U0TXBUF = data[idx];
    }

  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  P4OUT |= 1 << 2;
}

#line 103
static __inline uint8_t strobe(uint8_t reg)
#line 103
{
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  U0TXBUF = reg;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  P4OUT |= 1 << 2;

  return tmp;
}

# 188 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void radio_flush_tx()
#line 188
{
  strobe(CC2420_SFLUSHTX);
}

# 3 "../../tos/chips/cc2420/interfaces/LplTime.nc"
inline static void CC2420xRTxP__LplTime__timeCompensated(uint16_t time, rtx_time_compensation_t *p_rtx_time){
#line 3
  CC2420xLplP__LplTime__timeCompensated(time, p_rtx_time);
#line 3
}
#line 3
# 5 "../../tos/chips/cc2420/interfaces/LplSend.nc"
inline static void CC2420xRTxP__LplSend__sendDone(message_t *msg, rtx_setting_t *ts, error_t error){
#line 5
  CC2420xLplP__SubSend__sendDone(msg, ts, error);
#line 5
}
#line 5
# 9 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
inline static void CC2420xRTxP__LplReceive__receive(message_t *msg, uint8_t size){
#line 9
  CC2420xLplP__SubReceive__receive(msg, size);
#line 9
}
#line 9
# 2 "../../tos/chips/cc2420/interfaces/LplTime.nc"
inline static void CC2420xRTxP__LplTime__timeRadio(rtx_time_compensation_t *p_rtx_time){
#line 2
  CC2420xLplP__LplTime__timeRadio(p_rtx_time);
#line 2
}
#line 2
# 9 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t *get_packet_header(message_t *m)
#line 9
{
  return (uint8_t *)((uint8_t *)m + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 81 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void fast_write_any(uint8_t *buf, uint8_t len)
#line 81
{
  uint8_t idx;
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  U0TXBUF = CC2420_TXFIFO;

  for (idx = 0; idx < len; idx++) {
      while (!(IFG1 & 0x40)) ;
      tmp = U0RXBUF;
      while (!(IFG1 & 0x80)) ;
      U0TXBUF = buf[idx];
    }

  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  P4OUT |= 1 << 2;
}

# 884 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_load_tx(void )
#line 884
{
  fast_write_any(get_packet_header(CC2420xRTxP__p_tx_buf), CC2420xRTxP__pkt_length);
}

# 37 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void fast_read_one(uint8_t *buf)
#line 37
{
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  U0TXBUF = CC2420_RXFIFO | 0x40;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  while (!(IFG1 & 0x80)) ;
  U0TXBUF = 0;
  while (!(IFG1 & 0x40)) ;
  buf[0] = U0RXBUF;
}

# 181 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void radio_flush_rx()
#line 181
{
  uint8_t dummy;

#line 183
  fast_read_one(&dummy);
  strobe(CC2420_SFLUSHRX);
  strobe(CC2420_SFLUSHRX);
}

# 888 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_strobe_tx(void )
#line 888
{
  radio_flush_rx();
  TBCCTL1 &= ~0x0001;
  CC2420xRTxP__detect_duration = TBR - CC2420xRTxP__detect_duration;

  strobe(CC2420_STXON);
  CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
}

# 118 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline uint16_t get_register(uint8_t reg)
#line 118
{
  uint8_t tmp;
  uint16_t val;

  P4OUT &= ~(1 << 2);

  U0TXBUF = reg | 0x40;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = 0;
  while (!(IFG1 & 0x40)) ;
  val = U0RXBUF << 8;
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = 0;
  while (!(IFG1 & 0x40)) ;
  val |= U0RXBUF;

  P4OUT |= 1 << 2;

  return val;
}

# 171 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline int cc2420_get_rssi()
#line 171
{
  int rssi = 0xff & get_register(CC2420_RSSI);

#line 173
  if (rssi > 128) {
      rssi = rssi - 256 - 45;
    }
  else 
#line 175
    {
      rssi = rssi - 45;
    }
  return rssi;
}

# 692 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_signal_detect(uint16_t time)
#line 692
{
  uint8_t i;
  uint8_t pos_number = 0;
  uint8_t noise_number = 0;
  int rssi_max = CC2420xRTxP__noise_floor;
  int noise_floor_sum = 0;
  int pos_sum = 0;
  int rssi_list[22];

  CC2420xRTxP__detect_duration = time;

  while ((strobe(CC2420_SNOP) & CC2420_STATUS_RSSI_VALID) == 0) ;

  for (i = 0; i < 22 && ! CC2420xRTxP__m_rx_buf.received; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 706
        {
#line 706
          rssi_list[i] = cc2420_get_rssi();
        }
#line 707
        __nesc_atomic_end(__nesc_atomic); }
#line 707
      if (rssi_list[i] > CC2420xRTxP__noise_floor + 3) {
          pos_number++;
          pos_sum += rssi_list[i];
          if (rssi_list[i] > rssi_max) {
              rssi_max = rssi_list[i];
            }
        }
      else 
#line 713
        {
          noise_number++;
          noise_floor_sum += rssi_list[i];
        }
    }

  if (pos_number > 7 && ! CC2420xRTxP__m_rx_buf.received) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 720
        {
          int pos_avr = pos_sum / pos_number;

          if (rssi_max - pos_avr < 2) {

              TBCCR5 = TBR + 288;
              TBCCTL5 = 0x0010;
              {
#line 727
                __nesc_atomic_end(__nesc_atomic); 
#line 727
                return;
              }
            }
#line 729
          if (CC2420xRTxP__rtx_status == S_TX_DETECT) {
              CC2420xRTxP__rtx_status = S_TX_SFD;
              CC2420xRTxP__cc2420_strobe_tx();
              CC2420xRTxP__cc2420_load_tx();
              {
#line 733
                __nesc_atomic_end(__nesc_atomic); 
#line 733
                return;
              }
            }
#line 735
          if (CC2420xRTxP__rtx_status == S_RX_DETECT) {

              CC2420xRTxP__detect_duration = TBR - CC2420xRTxP__detect_duration;
              CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
              CC2420xRTxP__rtx_status = S_RTX_IDLE;
              TBCCTL1 = 0x0100;
              CC2420xRTxP__LplTime__timeRadio(&CC2420xRTxP__rtx_time);
              if (CC2420xRTxP__m_rx_buf.occ_size > 0) {
                  CC2420xRTxP__LplReceive__receive(&CC2420xRTxP__rx_buf[CC2420xRTxP__m_rx_buf.pos_buf], CC2420xRTxP__m_rx_buf.occ_size);
                }
              if (CC2420xRTxP__p_tx_buf != (void *)0) {
                  CC2420xRTxP__LplSend__sendDone(CC2420xRTxP__p_tx_buf, CC2420xRTxP__tx_setting, SUCCESS);
                  CC2420xRTxP__p_tx_buf = (void *)0;
                }
              CC2420xRTxP__LplTime__timeCompensated(TBR - TBCCR1, &CC2420xRTxP__rtx_time);
              {
#line 750
                __nesc_atomic_end(__nesc_atomic); 
#line 750
                return;
              }
            }
        }
#line 753
        __nesc_atomic_end(__nesc_atomic); }
    }
  if (! CC2420xRTxP__m_rx_buf.received) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 756
        {
          noise_floor_sum = noise_floor_sum / noise_number;
          CC2420xRTxP__noise_floor = (8 * CC2420xRTxP__noise_floor + 2 * noise_floor_sum) / 10;
          if (CC2420xRTxP__rtx_status == S_TX_DETECT) {
              CC2420xRTxP__rtx_status = S_TX_SFD;
              CC2420xRTxP__cc2420_strobe_tx();
              CC2420xRTxP__cc2420_load_tx();
              {
#line 763
                __nesc_atomic_end(__nesc_atomic); 
#line 763
                return;
              }
            }
#line 765
          if (CC2420xRTxP__rtx_status == S_RX_DETECT) {

              CC2420xRTxP__detect_duration = TBR - CC2420xRTxP__detect_duration;
              CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
              CC2420xRTxP__rtx_status = S_RTX_IDLE;
              TBCCTL1 = 0x0100;
              CC2420xRTxP__LplTime__timeRadio(&CC2420xRTxP__rtx_time);
              if (CC2420xRTxP__m_rx_buf.occ_size > 0) {
                  CC2420xRTxP__LplReceive__receive(&CC2420xRTxP__rx_buf[CC2420xRTxP__m_rx_buf.pos_buf], CC2420xRTxP__m_rx_buf.occ_size);
                }
              if (CC2420xRTxP__p_tx_buf != (void *)0) {
                  CC2420xRTxP__LplSend__sendDone(CC2420xRTxP__p_tx_buf, CC2420xRTxP__tx_setting, SUCCESS);
                  CC2420xRTxP__p_tx_buf = (void *)0;
                }
              CC2420xRTxP__LplTime__timeCompensated(TBR - TBCCR1, &CC2420xRTxP__rtx_time);
            }
        }
#line 781
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 60 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void fast_read_any(uint8_t *buf, uint8_t len)
#line 60
{
  uint8_t idx;
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  U0TXBUF = CC2420_RXFIFO | 0x40;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  for (idx = 0; idx < len; idx++) {
      while (!(IFG1 & 0x80)) ;
      U0TXBUF = 0;
      while (!(IFG1 & 0x40)) ;
      buf[idx] = U0RXBUF;
    }

  P4OUT |= 1 << 2;
}

# 908 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_ack_rx(void )
#line 908
{
  unsigned char __nesc_temp53;
  unsigned char *__nesc_temp52;
  unsigned char __nesc_temp51;
  unsigned char *__nesc_temp50;
#line 909
  uint8_t type = 0xff;
  cc2420_header_t *m_rx_header = (cc2420_header_t *)get_packet_header(CC2420xRTxP__m_rx_buf.p_rx_buf);
  cc2420_header_t *m_tx_header = (cc2420_header_t *)get_packet_header(CC2420xRTxP__p_tx_buf);

  fast_read_any((uint8_t *)m_rx_header, 5);

  type = (__nesc_ntoh_leuint16(m_rx_header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7;

  if (CC2420xRTxP__rtx_status == S_TX_ACK) {

      if (
#line 918
      type == IEEE154_TYPE_ACK
       && __nesc_ntoh_leuint8(m_rx_header->dsn.nxdata) == __nesc_ntoh_leuint8(m_tx_header->dsn.nxdata)) {
          __nesc_hton_int8(((cc2420_metadata_t *)CC2420xRTxP__p_tx_buf->metadata)->ack.nxdata, TRUE);
          (__nesc_temp50 = CC2420xRTxP__tx_setting->size.nxdata, __nesc_hton_uint8(__nesc_temp50, (__nesc_temp51 = __nesc_ntoh_uint8(__nesc_temp50)) - 1), __nesc_temp51);
          if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
              CC2420xRTxP__rtx_status = S_TX_SFD;
              CC2420xRTxP__tx_counter = 0;
              CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
              CC2420xRTxP__cc2420_load_tx();
            }
          else {
#line 927
            if (__nesc_ntoh_int8(CC2420xRTxP__tx_setting->ci.nxdata) && CC2420xRTxP__tx_counter != 64) {
                CC2420xRTxP__rtx_status = S_TX_SFD;
                CC2420xRTxP__tx_counter++;
              }
            else 
#line 930
              {
                radio_flush_tx();
                CC2420xRTxP__rtx_status = S_RX_DETECT;
                TBCCR5 = TBCCR1 + 288;
                CC2420xRTxP__detect_duration = TBCCR1;
                TBCCTL5 = 0x0010;
              }
            }
        }
      else 
#line 937
        {
          CC2420xRTxP__tx_counter++;
          if (CC2420xRTxP__tx_counter == 64) {
              (__nesc_temp52 = CC2420xRTxP__tx_setting->size.nxdata, __nesc_hton_uint8(__nesc_temp52, (__nesc_temp53 = __nesc_ntoh_uint8(__nesc_temp52)) - 1), __nesc_temp53);
              if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
                  CC2420xRTxP__rtx_status = S_TX_SFD;
                  CC2420xRTxP__tx_counter = 0;
                  CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
                  CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                }
              else 
#line 946
                {
                  radio_flush_tx();
                  CC2420xRTxP__rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + 288;
                  CC2420xRTxP__detect_duration = TBCCR1;
                  TBCCTL5 = 0x0010;
                }
            }
          else 
#line 953
            {
              CC2420xRTxP__rtx_status = S_TX_SFD;

              write_ram(CC2420_TXFIFO, sizeof(cc2420_header_t ) + sizeof(rtx_setting_t ), &CC2420xRTxP__tx_counter, 1);
            }
        }
    }

  if (CC2420xRTxP__rtx_status == S_CI_ACK) {

      if (CC2420xRTxP__tx_counter == 64) {
          radio_flush_tx();
          CC2420xRTxP__rtx_status = S_RX_DETECT;
          TBCCR5 = TBCCR1 + 288;
          CC2420xRTxP__detect_duration = TBCCR1;
          TBCCTL5 = 0x0010;
        }
      else 
#line 969
        {
          CC2420xRTxP__rtx_status = S_CI_SFD;
          CC2420xRTxP__tx_counter++;
          write_ram(CC2420_TXFIFO, sizeof(cc2420_header_t ) + sizeof(rtx_setting_t ), &CC2420xRTxP__tx_counter, 1);
        }
    }
}

# 30 "../../tos/printf/serial_fast_print.h"
static __inline void uart_fast_tx(uint8_t byte)
#line 30
{
  while (!(IFG2 & 0x20)) ;
  U1TXBUF = byte;
}











static __inline void printf_u16(uint8_t var, uint16_t *word)
#line 45
{
  uint8_t high;
  uint8_t low;
  uint8_t idx;

  uart_fast_tx(0x77);
  uart_fast_tx(var);
  for (idx = 0; idx < var; idx++) {
      low = word[idx] & 0x00FF;
      high = word[idx] >> 8;
      uart_fast_tx(high);
      uart_fast_tx(low);
    }
}

# 77 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline am_id_t CC2420xPacketP__AMPacket__type(message_t *amsg)
#line 77
{
  cc2420_header_t *header = (cc2420_header_t *)get_packet_header(amsg);

#line 79
  return __nesc_ntoh_leuint8(header->type.nxdata);
}

# 147 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static am_id_t CC2420ActiveMessageP__AMPacket__type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = CC2420xPacketP__AMPacket__type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 84 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 84
{
  return msg;
}

# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Snoop__receive(am_id_t arg_0x2ac07535b670, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = CC2420ActiveMessageP__Snoop__default__receive(arg_0x2ac07535b670, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 310 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 310
{
  const uint8_t *base = source;

#line 312
  return ((uint16_t )base[0] << 8) | base[1];
}

# 56 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 109 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 109
{
  LedsP__Led2__set();
  ;
#line 111
  ;
}

# 94 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led2Off(void ){
#line 94
  LedsP__Leds__led2Off();
#line 94
}
#line 94
# 57 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 6);
}

# 53 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 41 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 41
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 41
}
#line 41
# 104 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 104
{
  LedsP__Led2__clr();
  ;
#line 106
  ;
}

# 89 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led2On(void ){
#line 89
  LedsP__Leds__led2On();
#line 89
}
#line 89
# 56 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 94 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 94
{
  LedsP__Led1__set();
  ;
#line 96
  ;
}

# 77 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led1Off(void ){
#line 77
  LedsP__Leds__led1Off();
#line 77
}
#line 77
# 57 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 5);
}

# 53 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 41 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 41
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 41
}
#line 41
# 89 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 89
{
  LedsP__Led1__clr();
  ;
#line 91
  ;
}

# 72 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led1On(void ){
#line 72
  LedsP__Leds__led1On();
#line 72
}
#line 72
# 56 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 79 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Off(void )
#line 79
{
  LedsP__Led0__set();
  ;
#line 81
  ;
}

# 61 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led0Off(void ){
#line 61
  LedsP__Leds__led0Off();
#line 61
}
#line 61
# 57 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 4);
}

# 53 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 41 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 41
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 41
}
#line 41
# 74 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 74
{
  LedsP__Led0__clr();
  ;
#line 76
  ;
}

# 56 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led0On(void ){
#line 56
  LedsP__Leds__led0On();
#line 56
}
#line 56
# 94 "RadioCountToLedsC.nc"
static inline message_t *RadioCountToLedsC__Receive__receive(message_t *bufPtr, void *payload, uint8_t len)
#line 94
{
  if (len != sizeof(radio_count_msg_t )) {
    return bufPtr;
    }
  else 
#line 97
    {
      radio_count_msg_t *rcm = (radio_count_msg_t *)payload;

#line 99
      if (__nesc_ntoh_uint16(rcm->counter.nxdata) & 0x1) {
          RadioCountToLedsC__Leds__led0On();
        }
      else 
#line 101
        {
          RadioCountToLedsC__Leds__led0Off();
        }
      if (__nesc_ntoh_uint16(rcm->counter.nxdata) & 0x2) {
          RadioCountToLedsC__Leds__led1On();
        }
      else 
#line 106
        {
          RadioCountToLedsC__Leds__led1Off();
        }
      if (__nesc_ntoh_uint16(rcm->counter.nxdata) & 0x4) {
          RadioCountToLedsC__Leds__led2On();
        }
      else 
#line 111
        {
          RadioCountToLedsC__Leds__led2Off();
        }
      return bufPtr;
    }
}

# 81 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 81
{
  return msg;
}

# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Receive__receive(am_id_t arg_0x2ac07535ca90, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x2ac07535ca90) {
#line 78
    case 6:
#line 78
      __nesc_result = RadioCountToLedsC__Receive__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = CC2420ActiveMessageP__Receive__default__receive(arg_0x2ac07535ca90, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 44 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline am_addr_t CC2420xPacketP__AMPacket__address(void )
#line 44
{
  return TOS_AM_ADDRESS;
}

static inline am_addr_t CC2420xPacketP__AMPacket__destination(message_t *amsg)
#line 48
{
  cc2420_header_t *header = (cc2420_header_t *)get_packet_header(amsg);

#line 50
  return __nesc_ntoh_leuint16(header->dest.nxdata);
}

# 13 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t *get_packet_setting(message_t *m)
#line 13
{
  return (uint8_t *)((uint8_t *)m + (unsigned short )& ((message_t *)0)->data + 1);
}

# 67 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline bool CC2420xPacketP__AMPacket__isForMe(message_t *amsg)
#line 67
{
  rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(amsg);

  return (((CC2420xPacketP__AMPacket__destination(amsg) == CC2420xPacketP__AMPacket__address()
   || CC2420xPacketP__AMPacket__destination(amsg) == AM_BROADCAST_ADDR)
   || __nesc_ntoh_uint16(p_ts->addr.nxdata) == CC2420xPacketP__AMPacket__address())
   || __nesc_ntoh_uint16(p_ts->addr.nxdata) == AM_BROADCAST_ADDR)
   || __nesc_ntoh_uint16(p_ts->addr.nxdata) == 0xFFFE;
}

# 136 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static bool CC2420ActiveMessageP__AMPacket__isForMe(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420xPacketP__AMPacket__isForMe(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 67 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 67
{
  if (CC2420ActiveMessageP__AMPacket__isForMe(msg)) {
      return CC2420ActiveMessageP__Receive__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else 
#line 70
    {
      return CC2420ActiveMessageP__Snoop__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 78 "/opt/tinyos-main-read-only/tos/interfaces/Receive.nc"
inline static message_t * CC2420xLplP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP__AMSend__sendDone(am_id_t arg_0x2ac075324818, message_t * msg, error_t error){
#line 110
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x2ac075324818, msg, error);
#line 110
}
#line 110
# 57 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 57
{
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static void CC2420xLplP__Send__sendDone(message_t * msg, error_t error){
#line 100
  CC2420ActiveMessageP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 118 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__AMSend__sendDone(message_t *bufPtr, error_t error)
#line 118
{
  if (&RadioCountToLedsC__packet == bufPtr) {
      RadioCountToLedsC__locked = FALSE;
    }
}

# 110 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  RadioCountToLedsC__AMSend__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/opt/tinyos-main-read-only/tos/system/AMQueueEntryP.nc"
static inline void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 215 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x2ac0752e1e18, message_t * msg, error_t error){
#line 100
  switch (arg_0x2ac0752e1e18) {
#line 100
    case 0U:
#line 100
      /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x2ac0752e1e18, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 163 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

#line 65
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

# 29 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t get_packet_payloadLen(message_t *m)
#line 29
{
  return *(get_packet_setting(m) - 1);
}

# 105 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline uint8_t CC2420xPacketP__Packet__payloadLength(message_t *msg)
#line 105
{
  return get_packet_payloadLen(msg);
}

# 78 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420xPacketP__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 37 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t get_packet_bulk(message_t *m)
#line 37
{
  rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(m);

#line 39
  if (__nesc_ntoh_int8(p_ts->batched.nxdata)) {
    return __nesc_ntoh_uint8(p_ts->size.nxdata);
    }
#line 41
  return 0;
}

# 22 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline uint8_t CC2420xPacketP__LplxPacket__getPacketBulk(message_t *m)
#line 22
{
  return get_packet_bulk(m);
}

# 5 "../../tos/chips/cc2420/interfaces/LplxPacket.nc"
inline static uint8_t CC2420ActiveMessageP__LplxPacket__getPacketBulk(message_t *m){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = CC2420xPacketP__LplxPacket__getPacketBulk(m);
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 33 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t get_packet_maxPayloadLen()
#line 33
{
  return 77 - 1 - sizeof(rtx_setting_t );
}

# 113 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline uint8_t CC2420xPacketP__Packet__maxPayloadLength(void )
#line 113
{
  return get_packet_maxPayloadLen();
}

# 106 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
inline static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void ){
#line 106
  unsigned char __nesc_result;
#line 106

#line 106
  __nesc_result = CC2420xPacketP__Packet__maxPayloadLength();
#line 106

#line 106
  return __nesc_result;
#line 106
}
#line 106
# 297 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

# 82 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void CC2420xPacketP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 82
{
  cc2420_header_t *header = (cc2420_header_t *)get_packet_header(amsg);

#line 84
  __nesc_hton_leuint8(header->type.nxdata, type);
}

# 162 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static void CC2420ActiveMessageP__AMPacket__setType(message_t * amsg, am_id_t t){
#line 162
  CC2420xPacketP__AMPacket__setType(amsg, t);
#line 162
}
#line 162
# 315 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 315
{
  uint8_t *base = target;

#line 317
  base[1] = value;
  base[0] = value >> 8;
  return value;
}






static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 62 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline void set_packet_dest(message_t *m, am_addr_t dest)
#line 62
{
  rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(m);
  cc2420_header_t *p_header = (cc2420_header_t *)get_packet_header(m);

#line 65
  __nesc_hton_leuint16(p_header->dest.nxdata, dest);
  __nesc_hton_uint16(p_ts->addr.nxdata, dest);
}

# 58 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void CC2420xPacketP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 58
{
  set_packet_dest(amsg, addr);
}

# 103 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static void CC2420ActiveMessageP__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 103
  CC2420xPacketP__AMPacket__setDestination(amsg, addr);
#line 103
}
#line 103
# 3 "../../tos/chips/cc2420/interfaces/LplSend.nc"
inline static error_t CC2420xLplP__SubSend__send(message_t *msg, rtx_setting_t *ts){
#line 3
  unsigned char __nesc_result;
#line 3

#line 3
  __nesc_result = CC2420xRTxP__LplSend__send(msg, ts);
#line 3

#line 3
  return __nesc_result;
#line 3
}
#line 3
# 231 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void cc2420_rx_start()
#line 231
{
  strobe(CC2420_SRXON);
}

# 5 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
inline static error_t CC2420xLplP__SubReceive__rxInit(void ){
#line 5
  unsigned char __nesc_result;
#line 5

#line 5
  __nesc_result = CC2420xRTxP__LplReceive__rxInit();
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 63 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void disable_other_interrupts(interrupt_status_t *status)
#line 63
{
  status->ie1 = IE1;
  status->ie2 = IE2;
  status->p1ie = P1IE;
  status->p2ie = P2IE;

  IE1 = 0;
  IE2 = 0;

  P1IE = 0;
  P2IE = 0;

  DMA0CTL &= ~0x0004;
  DMA1CTL &= ~0x0004;
  DMA2CTL &= ~0x0004;

  CACTL1 &= ~0x02;

  TACTL &= ~0x0002;
  TACCTL0 &= ~0x0010;
  TACCTL1 &= ~0x0010;
  TACCTL2 &= ~0x0010;

  TBCCTL0 &= ~0x0010;

  TBCTL &= ~0x0002;

  P4SEL |= 1 << 1;

  TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
  TBCCTL1 &= ~0x0001;
}

# 94 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline void set_tx_setting(message_t *m, rtx_setting_t *ts)
#line 94
{
  memcpy((uint8_t *)ts, get_packet_setting(m), sizeof(rtx_setting_t ));
}

#line 90
static __inline void set_payload_length(message_t *m, uint8_t len)
#line 90
{
  *(get_packet_setting(m) - 1) = len;
}

#line 44
static __inline void set_packet_header(message_t *m, uint8_t dsn)
#line 44
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 45
  cc2420_header_t *p_header = (cc2420_header_t *)get_packet_header(m);
  rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(m);

  (__nesc_temp42 = p_header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) | (((1 << IEEE154_FCF_INTRAPAN)
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));
  __nesc_hton_leuint16(p_header->destpan.nxdata, 0);
  __nesc_hton_leuint16(p_header->src.nxdata, TOS_NODE_ID);
  if (__nesc_ntoh_leuint16(p_header->dest.nxdata) != AM_BROADCAST_ADDR) {
      __nesc_hton_int8(p_ts->ack.nxdata, TRUE);

      (__nesc_temp43 = p_header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | (1 << IEEE154_FCF_ACK_REQ)));
    }
  __nesc_hton_leuint8(p_header->length.nxdata, 77 + CC2420_SIZE);
  __nesc_hton_leuint8(p_header->dsn.nxdata, dsn);
}

# 166 "../../tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void CC2420xLplP__SleepTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(0U);
#line 78
}
#line 78
# 117 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline error_t CC2420xLplP__BulkSend__send(message_t *msg, uint8_t len)
#line 117
{
  uint8_t i;

  if (CC2420xLplP__lpl_status != LPL_X_IDLE) {
      return EBUSY;
    }
  CC2420xLplP__SleepTimer__stop();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 124
    {
      rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(msg);
      uint8_t size = __nesc_ntoh_uint8(p_ts->size.nxdata);

#line 127
      CC2420xLplP__lpl_status = LPL_X_TX;
      for (i = 0; i < size; i++) {
          set_packet_header(msg + i, CC2420xLplP__lpl_dsn);
          set_payload_length(msg + i, len);
          CC2420xLplP__lpl_dsn++;
        }
      set_tx_setting(msg, &CC2420xLplP__tx_status);
      disable_other_interrupts(&CC2420xLplP__ie_status);
      CC2420xLplP__SubReceive__rxInit();
      cc2420_rx_start();
    }
#line 137
    __nesc_atomic_end(__nesc_atomic); }
  return CC2420xLplP__SubSend__send(msg, &CC2420xLplP__tx_status);
}

# 2 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
inline static error_t CC2420ActiveMessageP__BulkSend__send(message_t *msg, uint8_t len){
#line 2
  unsigned char __nesc_result;
#line 2

#line 2
  __nesc_result = CC2420xLplP__BulkSend__send(msg, len);
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 143 "../../tos/chips/cc2420/x-rtx/cc2420_x_rtx.h"
static __inline void msp430_sync_dco()
#line 143
{
  uint16_t last;
  uint16_t diff;


  TBCCTL6 = ((0x1000 | 0x4000) | 0x0100) | 0x0800;

  while (1) {

      TBCCTL6 &= ~0x0001;
      while (!(TBCCTL6 & 0x0001)) ;
      last = TBCCR6;

      TBCCTL6 &= ~0x0001;

      while (!(TBCCTL6 & 0x0001)) ;
      diff = TBCCR6 - last;


      if (4194304UL / 32768 < diff) {
          DCOCTL--;
          if (DCOCTL == 0xFF) {
              BCSCTL1--;
            }
        }
      else {
#line 167
        if (4194304UL / 32768 > diff) {
            DCOCTL++;
            if (DCOCTL == 0x00) {
                BCSCTL1++;
              }
          }
        else 
#line 172
          {
            break;
          }
        }
    }
}

# 74 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline error_t CC2420xLplP__Send__send(message_t *msg, uint8_t len)
#line 74
{
  if (CC2420xLplP__lpl_status != LPL_X_IDLE) {
      return EBUSY;
    }
  CC2420xLplP__SleepTimer__stop();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      CC2420xLplP__lpl_status = LPL_X_TX;
      set_packet_header(msg, CC2420xLplP__lpl_dsn);
      set_payload_length(msg, len);
      set_tx_setting(msg, &CC2420xLplP__tx_status);
      disable_other_interrupts(&CC2420xLplP__ie_status);

      msp430_sync_dco();
      CC2420xLplP__SubReceive__rxInit();
      cc2420_rx_start();
    }
#line 89
    __nesc_atomic_end(__nesc_atomic); }
  return CC2420xLplP__SubSend__send(msg, &CC2420xLplP__tx_status);
}

# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420xLplP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 62 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__BulkSend__sendDone(message_t *msg, error_t result)
#line 62
{
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 4 "../../tos/chips/cc2420/interfaces/BulkSend.nc"
inline static void CC2420xLplP__BulkSend__sendDone(message_t *msg, error_t err){
#line 4
  CC2420ActiveMessageP__BulkSend__sendDone(msg, err);
#line 4
}
#line 4
# 223 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline void CC2420xLplP__RadioTimerUpdate__default__counterUpdate(uint32_t count, uint16_t factor)
#line 223
{
}

# 3 "../../tos/chips/cc2420/x-timer/RadioTimerUpdate.nc"
inline static void CC2420xLplP__RadioTimerUpdate__counterUpdate(uint32_t count, uint16_t factor){
#line 3
  CC2420xLplP__RadioTimerUpdate__default__counterUpdate(count, factor);
#line 3
}
#line 3
# 222 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline void CC2420xLplP__RadioTimerUpdate__default__triggerUpdate(void )
#line 222
{
}

# 2 "../../tos/chips/cc2420/x-timer/RadioTimerUpdate.nc"
inline static void CC2420xLplP__RadioTimerUpdate__triggerUpdate(void ){
#line 2
  CC2420xLplP__RadioTimerUpdate__default__triggerUpdate();
#line 2
}
#line 2
# 67 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 96 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void enable_other_interrupts(interrupt_status_t *status)
#line 96
{
  IE1 = status->ie1;
  IE2 = status->ie2;
  P1IE = status->p1ie;
  P2IE = status->p2ie;
  DMA0CTL &= ~0x0008;
  DMA0CTL |= 0x0004;
  DMA1CTL &= ~0x0008;
  DMA1CTL |= 0x0004;
  DMA2CTL &= ~0x0008;
  DMA2CTL |= 0x0004;
  CACTL1 &= ~0x01;
  CACTL1 |= 0x02;
  TACTL &= ~0x0001;
  TACTL |= 0x0002;
  TACCTL0 &= ~0x0001;
  TACCTL0 |= 0x0010;
  TACCTL1 &= ~0x0001;
  TACCTL1 |= 0x0010;
  TACCTL2 &= ~0x0001;
  TACCTL2 |= 0x0010;
  TBCTL &= ~0x0001;
  TBCTL |= 0x0002;
  TBCCTL0 &= ~0x0001;
  TBCCTL0 |= 0x0010;
}

# 977 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_ack_rx_except(void )
#line 977
{
  unsigned char __nesc_temp55;
  unsigned char *__nesc_temp54;
#line 978
  uint8_t type = 0xff;
  cc2420_header_t *m_rx_header = (cc2420_header_t *)get_packet_header(CC2420xRTxP__m_rx_buf.p_rx_buf);
  cc2420_header_t *m_tx_header = (cc2420_header_t *)get_packet_header(CC2420xRTxP__p_tx_buf);

  fast_read_any((uint8_t *)m_rx_header, 5);

  type = (__nesc_ntoh_leuint16(m_rx_header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7;

  if (CC2420xRTxP__rtx_status == S_TX_ACK) {

      if (
#line 987
      type == IEEE154_TYPE_ACK
       && __nesc_ntoh_leuint8(m_rx_header->dsn.nxdata) == __nesc_ntoh_leuint8(m_tx_header->dsn.nxdata)) {
          __nesc_hton_int8(((cc2420_metadata_t *)CC2420xRTxP__p_tx_buf->metadata)->ack.nxdata, TRUE);
          (__nesc_temp54 = CC2420xRTxP__tx_setting->size.nxdata, __nesc_hton_uint8(__nesc_temp54, (__nesc_temp55 = __nesc_ntoh_uint8(__nesc_temp54)) - 1), __nesc_temp55);
          if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
              CC2420xRTxP__rtx_status = S_TX_DETECT;
              CC2420xRTxP__tx_counter = 0;
              CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
              CC2420xRTxP__cc2420_signal_detect(TBCCR1);
            }
          else 
#line 996
            {
              radio_flush_tx();
              CC2420xRTxP__rtx_status = S_RX_DETECT;
              TBCCR5 = TBCCR1 + 288;
              CC2420xRTxP__detect_duration = TBCCR1;
              TBCCTL5 = 0x0010;
            }
        }
      else 
#line 1003
        {
          CC2420xRTxP__tx_counter++;
          write_ram(CC2420_TXFIFO, sizeof(cc2420_header_t ) + sizeof(rtx_setting_t ), &CC2420xRTxP__tx_counter, 1);
          if (CC2420xRTxP__tx_counter == 64) {
              if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
                  CC2420xRTxP__rtx_status = S_TX_DETECT;
                  CC2420xRTxP__tx_counter = 0;
                  CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
                  CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                }
              else 
#line 1012
                {
                  radio_flush_tx();
                  CC2420xRTxP__rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + 288;
                  CC2420xRTxP__detect_duration = TBCCR1;
                  TBCCTL5 = 0x0010;
                }
            }
          else 
#line 1019
            {
              CC2420xRTxP__rtx_status = S_TX_DETECT;
              CC2420xRTxP__cc2420_signal_detect(TBCCR1);
            }
        }
    }

  if (CC2420xRTxP__rtx_status == S_CI_ACK) {
      radio_flush_tx();
      CC2420xRTxP__rtx_status = S_RX_DETECT;
      TBCCR5 = TBCCR1 + 288;
      CC2420xRTxP__detect_duration = TBCCR1;
      TBCCTL5 = 0x0010;
    }
}

# 30 "../../tos/chips/cc2420/x-timer/cc2420_x_timer.h"
static __inline void ack_time_update(rtx_time_compensation_t *rtc, uint16_t time)
#line 30
{
  rtc->ack_time = time;
}

# 875 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_ack_strobe_rx(void )
#line 875
{
  radio_flush_rx();
  TBCCTL1 &= ~0x0001;
  CC2420xRTxP__detect_duration = TBR - CC2420xRTxP__detect_duration;
  CC2420xRTxP__turn_around = TBR;
  strobe(CC2420_SACK);
  CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
}

#line 897
static __inline void CC2420xRTxP__cc2420_ack_wait_tx(void )
#line 897
{
  radio_flush_rx();
  TBCCTL1 &= ~0x0001;
  CC2420xRTxP__detect_duration = TBR - CC2420xRTxP__detect_duration;


  TBCCR2 = TBR + CC2420xRTxP__rtx_time.turnaround_time + CC2420xRTxP__rtx_time.ack_time;
  TBCCTL2 = 0x0010;
  CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
}

#line 824
static __inline void CC2420xRTxP__cc2420_end_rx(void )
#line 824
{
  unsigned char __nesc_temp49;
  unsigned char *__nesc_temp48;
#line 825
  uint8_t i;
  bool duplicate = FALSE;
  uint8_t *p_buf = get_packet_header(CC2420xRTxP__m_rx_buf.p_rx_buf);

  CC2420xRTxP__rtx_status = S_RX_ACK;

  fast_read_any(p_buf + CC2420xRTxP__rx_readbytes, CC2420xRTxP__pkt_length - CC2420xRTxP__rx_readbytes + 1);
  CC2420xRTxP__rx_readbytes = CC2420xRTxP__pkt_length + 1;

  if (p_buf[CC2420xRTxP__pkt_length] >> 7) {
      rtx_setting_t *m_ptr_setting = (rtx_setting_t *)get_packet_setting(CC2420xRTxP__m_rx_buf.p_rx_buf);





      if ((
#line 837
      ! __nesc_ntoh_int8(m_ptr_setting->ack.nxdata)
       || (__nesc_ntoh_uint16(m_ptr_setting->addr.nxdata) != TOS_NODE_ID
       && (__nesc_ntoh_uint16(m_ptr_setting->addr.nxdata) != 0xFFFE
       || __nesc_ntoh_uint16(m_ptr_setting->metric.nxdata) - CC2420xRTxP__local_metric < __nesc_ntoh_uint16(m_ptr_setting->progress.nxdata))))
       || !(__nesc_ntoh_int8(m_ptr_setting->ci.nxdata) && __nesc_ntoh_uint8(m_ptr_setting->hop.nxdata) != 0)) {
          radio_flush_tx();
          radio_flush_rx();
          return;
        }


      for (i = 0; i < CC2420xRTxP__m_rx_buf.occ_size; i++) {
          cc2420_header_t *p_header = (cc2420_header_t *)get_packet_header(&CC2420xRTxP__rx_buf[(CC2420xRTxP__m_rx_buf.pos_buf + i) % 7]);

#line 850
          if (__nesc_ntoh_leuint8(((cc2420_header_t *)p_buf)->dsn.nxdata) == __nesc_ntoh_leuint8(p_header->dsn.nxdata) && __nesc_ntoh_leuint16(((cc2420_header_t *)p_buf)->src.nxdata) == __nesc_ntoh_leuint16(p_header->src.nxdata)) {
              duplicate = TRUE;
              break;
            }
        }
      if (!duplicate) {

          (__nesc_temp48 = m_ptr_setting->hop.nxdata, __nesc_hton_uint8(__nesc_temp48, (__nesc_temp49 = __nesc_ntoh_uint8(__nesc_temp48)) - 1), __nesc_temp49);
          memcpy(&CC2420xRTxP__rx_setting, m_ptr_setting, sizeof(rtx_setting_t ));

          CC2420xRTxP__m_rx_buf.occ_size++;
          if (CC2420xRTxP__m_rx_buf.occ_size != CC2420xRTxP__m_rx_buf.max_size) {
              CC2420xRTxP__m_rx_buf.p_rx_buf = &CC2420xRTxP__rx_buf[(CC2420xRTxP__m_rx_buf.pos_buf + CC2420xRTxP__m_rx_buf.occ_size) % 7];
            }
          else {
            }
        }
    }
  else {
      radio_flush_tx();
      radio_flush_rx();
    }
}

# 26 "../../tos/chips/cc2420/x-timer/cc2420_x_timer.h"
static __inline void rx_time_update(rtx_time_compensation_t *rtc, uint16_t time)
#line 26
{
  rtc->pkt_rtx_time = (rtc->pkt_rtx_time * 8 + time * 2) / 10;
}

#line 22
static __inline void tx_time_update(rtx_time_compensation_t *rtc, uint16_t time)
#line 22
{
  rtc->pkt_rtx_time = (rtc->pkt_rtx_time * 8 + time * 2) / 10;
}

# 53 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void fast_continue_read_one(uint8_t *buf)
#line 53
{
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = 0;
  while (!(IFG1 & 0x40)) ;
  buf[0] = U0RXBUF;
}

# 785 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static __inline void CC2420xRTxP__cc2420_begin_rx(void )
#line 785
{
  uint8_t *p_header = get_packet_header(CC2420xRTxP__m_rx_buf.p_rx_buf);

#line 787
  CC2420xRTxP__rtx_status = S_RX_RECEIVE;

  while ((P1IN & (1 << 3)) == 0) {
      if (TBCCTL3 & 0x0001) {
          radio_flush_rx();
          CC2420xRTxP__rtx_status = S_RTX_IDLE;
          TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
          return;
        }
    }

  fast_read_one(p_header);

  if (p_header[0] != CC2420xRTxP__pkt_length) {
      radio_flush_rx();
      TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
      return;
    }

  CC2420xRTxP__rx_readbytes = 1;

  while (CC2420xRTxP__rx_readbytes <= CC2420xRTxP__pkt_length - 8) {

      while ((P1IN & (1 << 3)) == 0) {
          if (TBCCTL3 & 0x0001) {
              radio_flush_rx();
              TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
              return;
            }
        }

      fast_continue_read_one(p_header + CC2420xRTxP__rx_readbytes);

      CC2420xRTxP__rx_readbytes++;
    }
}

# 34 "../../tos/chips/cc2420/x-timer/cc2420_x_timer.h"
static __inline void turnaround_time_update(rtx_time_compensation_t *rtc, uint16_t time)
#line 34
{
  rtc->turnaround_time = time;
}

# 120 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "../../tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void CC2420xRTxP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 124 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/opt/tinyos-main-read-only/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 63 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/opt/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/opt/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/opt/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 153 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}







static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 97
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;
#line 111
  Msp430ClockP__TACTL = 0x0100 | 0x0002;
}

#line 142
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "../../tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 114 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;
#line 129
  Msp430ClockP__TBCTL = 0x0200 | 0x0002;
}

#line 147
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "../../tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 74 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));









  BCSCTL2 = 0;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 137
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "../../tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 236 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__msp430_init_dco(void )
#line 236
{
  uint16_t compare = 0;
  uint16_t oldcapture = 0;
  uint16_t i = 0;

  WDTCTL = 0x5A00 | 0x0080;

  DCOCTL = 0;
  BCSCTL1 = 0xa6;
  BCSCTL2 = 0x00;

  for (i = 0xffff; i > 0; i--) {
       __asm ("nop");}


  TACCTL2 = 0x1000 + 0x4000 + 0x0100;
  Msp430ClockP__TACTL = 0x0200 + 0x0004 + 0x0020;

  while (1) {
      TACCTL2 &= ~0x0001;
      while ((TACCTL2 & 0x0001) != 0x0001) ;
      oldcapture = TACCR2;

      TACCTL2 &= ~0x0001;
      while ((TACCTL2 & 0x0001) != 0x0001) ;
      compare = TACCR2 - oldcapture;

      if (4194304UL / (32768 / 8) == compare) {
          break;
        }
      else {
#line 265
        if (4194304UL / (32768 / 8) < compare) {
            DCOCTL--;
            if (DCOCTL == 0xFF) {
                BCSCTL1--;
              }
          }
        else 
#line 270
          {
            DCOCTL++;
            if (DCOCTL == 0x00) {
                BCSCTL1++;
              }
          }
        }
    }
  TACCTL2 = 0;
  Msp430ClockP__TACTL = 0;

  BCSCTL1 &= ~(0x20 + 0x10);
}

static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {


    Msp430ClockP__msp430_init_dco();
    Msp430ClockP__Msp430ClockInit__initClocks();




    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__startTimerB();
    Msp430ClockP__startTimerA();
  }

  return SUCCESS;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/opt/tinyos-main-read-only/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/opt/tinyos-main-read-only/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/opt/tinyos-main-read-only/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 55 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "../../tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 73 "../../tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 74
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 135 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static inline void CC2420xRTxP__LplReceive__rxOn(void )
#line 135
{
  if ((CC2420xRTxP__rtx_status != S_RTX_IDLE) | (CC2420xRTxP__m_rx_buf.occ_size == CC2420xRTxP__m_rx_buf.max_size)) {
    return;
    }
#line 138
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 138
    {
      CC2420xRTxP__m_rx_buf.p_rx_buf = &CC2420xRTxP__rx_buf[(CC2420xRTxP__m_rx_buf.pos_buf + CC2420xRTxP__m_rx_buf.occ_size) % CC2420xRTxP__m_rx_buf.max_size];
      CC2420xRTxP__rtx_time.pkt_recv = 0;
      CC2420xRTxP__rtx_time.pkt_send = 0;
      CC2420xRTxP__rtx_time.pkt_ack = 0;
      CC2420xRTxP__rtx_time.pkt_turnaround = 0;
      CC2420xRTxP__rtx_time.channel_detection = 0;
      CC2420xRTxP__rtx_status = S_RX_DETECT;
      TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
    }
#line 147
    __nesc_atomic_end(__nesc_atomic); }
  CC2420xRTxP__cc2420_signal_detect(TBR);
}

# 3 "../../tos/chips/cc2420/interfaces/LplReceive.nc"
inline static void CC2420xLplP__SubReceive__rxOn(void ){
#line 3
  CC2420xRTxP__LplReceive__rxOn();
#line 3
}
#line 3
# 59 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline void CC2420xLplP__SleepTimer__fired(void )
#line 59
{
  if (CC2420xLplP__lpl_status == LPL_X_IDLE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
        {
          CC2420xLplP__lpl_status = LPL_X_RX;
          disable_other_interrupts(&CC2420xLplP__ie_status);

          msp430_sync_dco();
          CC2420xLplP__SubReceive__rxInit();
          CC2420xLplP__radio_start_time = TBR;
          cc2420_rx_start();
        }
#line 69
        __nesc_atomic_end(__nesc_atomic); }
      CC2420xLplP__SubReceive__rxOn();
    }
}

# 347 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 347
{
  uint8_t *base = target;

#line 349
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x2ac0752df020, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = CC2420ActiveMessageP__AMSend__send(arg_0x2ac0752df020, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 78 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420xPacketP__AMPacket__destination(amsg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
#line 147
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = CC2420xPacketP__AMPacket__type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 109 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void CC2420xPacketP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 109
{
  set_payload_length(msg, len);
}

# 94 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  CC2420xPacketP__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/opt/tinyos-main-read-only/tos/interfaces/Send.nc"
inline static error_t /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 162 "/opt/tinyos-main-read-only/tos/interfaces/AMPacket.nc"
inline static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 162
  CC2420xPacketP__AMPacket__setType(amsg, t);
#line 162
}
#line 162
#line 103
inline static void /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 103
  CC2420xPacketP__AMPacket__setDestination(amsg, addr);
#line 103
}
#line 103
# 53 "/opt/tinyos-main-read-only/tos/system/AMQueueEntryP.nc"
static inline error_t /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 6);
  return /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 80 "/opt/tinyos-main-read-only/tos/interfaces/AMSend.nc"
inline static error_t RadioCountToLedsC__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*RadioCountToLedsAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/opt/tinyos-main-read-only/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void )
{
  return /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get();
}

# 61 "/opt/tinyos-main-read-only/tos/lib/timer/LocalTime.nc"
inline static uint32_t RadioCountToLedsC__LocalTime__get(void ){
#line 61
  unsigned long __nesc_result;
#line 61

#line 61
  __nesc_result = /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 75 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline void set_packet_bulk(message_t *m, uint8_t size)
#line 75
{
  rtx_setting_t *p_ts = (rtx_setting_t *)get_packet_setting(m);

#line 77
  __nesc_hton_int8(p_ts->batched.nxdata, TRUE);
  __nesc_hton_uint8(p_ts->size.nxdata, size);
}

# 18 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void CC2420xPacketP__LplxPacket__setPacketBulk(message_t *m, uint8_t size)
#line 18
{
  set_packet_bulk(m, size);
}

# 4 "../../tos/chips/cc2420/interfaces/LplxPacket.nc"
inline static void RadioCountToLedsC__LplxPacket__setPacketBulk(message_t *m, uint8_t size){
#line 4
  CC2420xPacketP__LplxPacket__setPacketBulk(m, size);
#line 4
}
#line 4
# 17 "../../tos/chips/cc2420/x-packet/cc2420_x_packet.h"
static __inline uint8_t *get_packet_payload(message_t *m)
#line 17
{
  return (uint8_t *)((uint8_t *)m + (unsigned short )& ((message_t *)0)->data + 1 + sizeof(rtx_setting_t ));
}

# 117 "../../tos/chips/cc2420/x-packet/CC2420xPacketP.nc"
static inline void *CC2420xPacketP__Packet__getPayload(message_t *msg, uint8_t len)
#line 117
{
  if (len < get_packet_maxPayloadLen()) {
    return (void *)get_packet_payload(msg);
    }
#line 120
  return (void *)0;
}

# 126 "/opt/tinyos-main-read-only/tos/interfaces/Packet.nc"
inline static void * RadioCountToLedsC__Packet__getPayload(message_t * msg, uint8_t len){
#line 126
  void *__nesc_result;
#line 126

#line 126
  __nesc_result = CC2420xPacketP__Packet__getPayload(msg, len);
#line 126

#line 126
  return __nesc_result;
#line 126
}
#line 126
# 58 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 58
}
#line 58
# 50 "/opt/tinyos-main-read-only/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 42 "/opt/tinyos-main-read-only/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 42
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 42
}
#line 42
# 84 "/opt/tinyos-main-read-only/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 84
{
  LedsP__Led0__toggle();
  ;
#line 86
  ;
}

# 67 "/opt/tinyos-main-read-only/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led0Toggle(void ){
#line 67
  LedsP__Leds__led0Toggle();
#line 67
}
#line 67
# 76 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__MilliTimer__fired(void )
#line 76
{
  RadioCountToLedsC__Leds__led0Toggle();
  RadioCountToLedsC__counter++;
  if (RadioCountToLedsC__locked) {
    return;
    }
  else 
#line 81
    {
      radio_count_msg_t *rcm = (radio_count_msg_t *)RadioCountToLedsC__Packet__getPayload(&RadioCountToLedsC__packet, sizeof(radio_count_msg_t ));

#line 83
      if (rcm == (void *)0) {
        return;
        }
      RadioCountToLedsC__LplxPacket__setPacketBulk(&RadioCountToLedsC__packet, 1);
      __nesc_hton_uint16(rcm->counter.nxdata, RadioCountToLedsC__counter);
      __nesc_hton_uint32(rcm->time.nxdata, RadioCountToLedsC__LocalTime__get());
      if (RadioCountToLedsC__AMSend__send(AM_BROADCAST_ADDR, &RadioCountToLedsC__packet, sizeof(radio_count_msg_t )) == SUCCESS) {
        RadioCountToLedsC__locked = TRUE;
        }
    }
}

# 213 "../../tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x2ac0755542f8){
#line 83
  switch (arg_0x2ac0755542f8) {
#line 83
    case 0U:
#line 83
      CC2420xLplP__SleepTimer__fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      RadioCountToLedsC__MilliTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x2ac0755542f8);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 103 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "../../tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 97
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 92 "../../tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 94
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 94
        __nesc_atomic_end(__nesc_atomic); 
#line 94
        return __nesc_temp;
      }
    }
#line 96
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 87 "../../tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/opt/tinyos-main-read-only/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 102 "../../tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 103
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 141 "../../tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 69 "/opt/tinyos-main-read-only/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 119 "../../tos/chips/cc2420/x-rtx/cc2420_x_rtx.h"
static __inline void timer_initialization()
#line 119
{

  TBCCTL1 = 0x0100;

  TBCCTL2 = 0;

  TBCCTL3 = 0;

  TBCCTL4 = 0;

  TBCCTL6 = 0x0100;
  TACCTL2 = 0x0100;

  TBCCTL5 = 0;
}

# 88 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static inline error_t CC2420xRTxP__Init__init(void )
#line 88
{
  CC2420xRTxP__rtx_status = S_RTX_IDLE;
  CC2420xRTxP__pkt_length = 77 + CC2420_SIZE;
  CC2420xRTxP__noise_floor = -77;
  CC2420xRTxP__p_tx_buf = (void *)0;
  CC2420xRTxP__swap_tx_buf = (void *)0;
  CC2420xRTxP__tx_counter = 0;
  CC2420xRTxP__rx_readbytes = 0;
  CC2420xRTxP__m_rx_buf.received = FALSE;
  CC2420xRTxP__m_rx_buf.max_size = 7;
  CC2420xRTxP__m_rx_buf.occ_size = 0;
  CC2420xRTxP__m_rx_buf.pos_buf = 0;
  CC2420xRTxP__m_rx_buf.p_rx_buf = &CC2420xRTxP__rx_buf[0];
  timer_initialization();
  CC2420xRTxP__rtx_time.calibration_factor = 128;
  CC2420xRTxP__rtx_time.pkt_rtx_time = 10335;
  CC2420xRTxP__rtx_time.ack_time = 672;
  CC2420xRTxP__rtx_time.turnaround_time = 537;
  CC2420xRTxP__rtx_time.turnaround_total_time = 0;
  CC2420xRTxP__rtx_time.rtx_total_time = 0;
  CC2420xRTxP__rtx_time.ack_total_time = 0;
  CC2420xRTxP__rtx_time.radio_on_time = 0;
  CC2420xRTxP__rtx_time.tail_total_time = 0;
  return SUCCESS;
}

# 34 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline error_t CC2420xLplP__Init__init(void )
#line 34
{
  CC2420xLplP__lpl_status = LPL_X_CLOSE;
  CC2420xLplP__lpl_dsn = TOS_NODE_ID % 0xFF;
  return SUCCESS;
}

# 46 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4298 {
#line 46
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl();
}

# 36 "../../tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare();
#line 36
}
#line 36
# 43 "../../tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 55 "/opt/tinyos-main-read-only/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = RandomMlcgC__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420xLplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420xRTxP__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 156 "../../tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void RadioCountToLedsC__MilliTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(1U, dt);
#line 64
}
#line 64
# 161 "../../tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/opt/tinyos-main-read-only/tos/lib/timer/Timer.nc"
inline static void CC2420xLplP__SleepTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 73
}
#line 73
# 235 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void cc2420_rx_stop()
#line 235
{
  strobe(CC2420_SRFOFF);
}

# 141 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static __inline void set_register(uint8_t reg, uint16_t val)
#line 141
{
  uint8_t tmp;


  P4OUT &= ~(1 << 2);

  U0TXBUF = reg;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = val >> 8;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;
  while (!(IFG1 & 0x80)) ;
  U0TXBUF = val & 0xff;
  while (!(IFG1 & 0x40)) ;
  tmp = U0RXBUF;

  P4OUT |= 1 << 2;
}

# 163 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static __inline void cc2420_io_setting()
#line 163
{


  uint16_t setting = get_register(CC2420_IOCFG0);

#line 167
  setting |= (1 << CC2420_IOCFG0_CCA_POLARITY) | (127 << CC2420_IOCFG0_FIFOP_THR);
  set_register(CC2420_IOCFG0, setting);
}

#line 123
static __inline void cc2420_tx_setting()
#line 123
{

  uint16_t setting = get_register(CC2420_TXCTRL);

#line 126
  setting &= (31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL;
  set_register(CC2420_TXCTRL, setting);
}

#line 149
static __inline void cc2420_rx_setting()
#line 149
{

  uint16_t setting = get_register(CC2420_RXCTRL1);

#line 152
  setting |= 1 << CC2420_RXCTRL1_RXBPF_LOCUR;
  set_register(CC2420_RXCTRL1, setting);
}

#line 130
static __inline void cc2420_channel_setting()
#line 130
{
  uint16_t setting = get_register(CC2420_FSCTRL);

#line 132
  setting &= 0xFE00;
  setting |= 0x1FFF & (357 + 5 * (26 - 11));
  set_register(CC2420_FSCTRL, setting);
}

#line 156
static __inline void cc2420_sec_setting()
#line 156
{

  uint16_t setting = get_register(CC2420_SECCTRL0);

#line 159
  setting &= ~(1 << CC2420_SECCTRL0_RXFIFO_PROTECTION);
  set_register(CC2420_SECCTRL0, setting);
}

#line 137
static __inline void cc2420_mod_setting()
#line 137
{


  uint16_t setting = get_register(CC2420_MDMCTRL0);

#line 141
  setting &= ~(1 << CC2420_MDMCTRL0_ADR_DECODE);
  set_register(CC2420_MDMCTRL0, setting);

  setting = get_register(CC2420_MDMCTRL1);
  setting |= (20 & 0x1F) << CC2420_MDMCTRL1_CORR_THR;
  set_register(CC2420_MDMCTRL1, setting);
}

# 17 "../../tos/chips/cc2420/x-timer/cc2420_x_timer.h"
static inline void clock_delay(unsigned int i)
#line 17
{
   __asm ("add #-1, r15");
   __asm ("jnz $-2");}

# 15 "../../tos/chips/cc2420/x-spi/cc2420_x_spi.h"
static inline void cc2420_spi_init()
#line 15
{
  static unsigned char spi_inited = 0;

  if (spi_inited) {
    return;
    }


  U0CTL = 0x10 + 0x04 + 0x02 + 0x01;
  U0TCTL = 0x80 + 0x20 + 0x02;

  U0BR0 = 0x02;
  U0BR1 = 0;
  U0MCTL = 0;

  P3SEL |= ((1 << 1) | (1 << 2)) | (1 << 3);
  P3DIR |= (1 << 1) | (1 << 3);

  ME1 |= 0x40;
  U0CTL &= ~0x01;
}

# 192 "../../tos/chips/cc2420/x-control/cc2420_x_control.h"
static inline void cc2420_init()
#line 192
{

  cc2420_spi_init();

  P4DIR |= ((1 << 2) | (1 << 5)) | (1 << 6);

  P4DIR &= ~(1 << 1);
  P1DIR &= ~(((1 << 0) | (1 << 3)) | (1 << 4));

  P4OUT |= 1 << 2;

  P1IE &= ~(1 << 0);
  do {
#line 204
      P1IES &= ~(1 << 0);
#line 204
      P1IFG &= ~(1 << 0);
    }
  while (
#line 204
  0);

  P4OUT |= 1 << 5;
  P4OUT &= ~(1 << 6);

  clock_delay(1024);
  P4OUT |= 1 << 6;

  strobe(CC2420_SXOSCON);

  while (!(strobe(CC2420_SNOP) & (1 << CC2420_XOSC16M_STABLE))) ;

  cc2420_mod_setting();
  cc2420_sec_setting();
  cc2420_channel_setting();
  cc2420_rx_setting();
  cc2420_tx_setting();
  cc2420_io_setting();

  radio_flush_rx();
}

# 40 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static inline error_t CC2420xLplP__RadioControl__start(void )
#line 40
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 41
    {
      cc2420_init();
      cc2420_rx_stop();
      CC2420xLplP__lpl_status = LPL_X_IDLE;
      CC2420xLplP__SleepTimer__startOneShot(128);
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/opt/tinyos-main-read-only/tos/interfaces/StdControl.nc"
inline static error_t RadioCountToLedsC__AMControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420xLplP__RadioControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 7 "../../tos/printf/serial_fast_print.h"
static inline void uart_init()
#line 7
{
  static unsigned char uart_inited = 0;

  if (uart_inited) {
    return;
    }
  P3SEL |= 0xc0;
  U1CTL = 0x10 + 0x01;
  U1TCTL = 0x20;






  U1BR0 = 0x24;
  U1BR1 = 0x00;
  U1MCTL = 0x29;

  ME2 = 0x20;
  U1CTL &= ~0x01;
}

# 67 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__Boot__booted(void )
#line 67
{

  RadioCountToLedsC__locked = TRUE;
  memset((uint8_t *)&RadioCountToLedsC__packet, 0x0, sizeof(message_t ));
  uart_init();
  RadioCountToLedsC__AMControl__start();
  RadioCountToLedsC__MilliTimer__startPeriodic(1024);
}

# 60 "/opt/tinyos-main-read-only/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  RadioCountToLedsC__Boot__booted();
#line 60
}
#line 60
# 175 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/opt/tinyos-main-read-only/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2ac074ba7d50){
#line 75
  switch (arg_0x2ac074ba7d50) {
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2ac074ba7d50);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 391 "/opt/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 55 "../../tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 55
{
  return MSP430_POWER_LPM3;
}

# 62 "/opt/tinyos-main-read-only/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "../../tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;








  if ((((((((((
#line 78
  TBCCTL0 & 0x0010 || TBCCTL1 & 0x0010) || TBCCTL2 & 0x0010) || TBCCTL3 & 0x0010) || TBCCTL4 & 0x0010) || TBCCTL5 & 0x0010) || TBCCTL6 & 0x0010) && (
  TBCTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 98
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 99
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/opt/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 111 "../../tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 111
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 116
{
  uint16_t temp;

#line 118
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/opt/tinyos-main-read-only/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/opt/tinyos-main-read-only/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 411 "/opt/tinyos-main-read-only/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 14 "../../tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 135 "../../tos/lib/timer/TransformCounterC.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 169 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

# 170 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "../../tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else 
#line 125
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 130
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
#line 135
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 82 "../../tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 85
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 89
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 103
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 105
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "../../tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )368U;
    }
}

# 169 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 15 "../../tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 15
{
#line 15
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 16
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 16
{
#line 16
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 169 "../../tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 170 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 170
{
  unsigned char __nesc_temp47;
  unsigned char *__nesc_temp46;
  unsigned char __nesc_temp45;
  unsigned char *__nesc_temp44;
#line 172
  uint16_t TB1_irq = (TBR - TBCCR1 - 42) << 1;


  uint16_t TB2_irq = (TBR - TBCCR2 - 56) << 1;
  uint8_t debug;

#line 177
  CC2420xRTxP__tbiv = TBIV;
  CC2420xRTxP__tb1_buffer = TBCCR1;



  if (CC2420xRTxP__tbiv == 2) {


      if ((CC2420xRTxP__rtx_status == S_TX_ACK || CC2420xRTxP__rtx_status == S_CI_ACK) && !(P4IN & (1 << 1))) {



          CC2420xRTxP__ack_duration = TBCCR1 - CC2420xRTxP__ack_duration;
          CC2420xRTxP__detect_duration = TBCCR1;

          if (CC2420xRTxP__rtx_status == S_RX_ACK && !(P4IN & (1 << 1))) {
#line 192
            ;
            }
          TBCCTL3 &= ~(0x0010 | 0x0001);

          TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;

          if (TB1_irq <= 8) {

               __asm volatile ("add %[d], r0" :  : [d] "m"(TB1_irq));
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");

               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");

              CC2420xRTxP__cc2420_strobe_tx();

              CC2420xRTxP__cc2420_ack_rx();
            }
          else 
#line 219
            {
              CC2420xRTxP__cc2420_ack_rx_except();
            }

          CC2420xRTxP__rtx_time.pkt_ack++;
          ack_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__ack_duration);
        }
      else {
#line 225
        if (CC2420xRTxP__rtx_status == S_RX_ACK && !(P4IN & (1 << 1))) {



            CC2420xRTxP__ack_duration = TBCCR1 - CC2420xRTxP__ack_duration;
            CC2420xRTxP__detect_duration = TBCCR1;


            TBCCTL3 &= ~(0x0010 | 0x0001);

            TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;

            if (TB1_irq <= 8) {

                 __asm volatile ("add %[d], r0" :  : [d] "m"(TB1_irq));
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");

                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");
                 __asm volatile ("nop");

                CC2420xRTxP__cc2420_strobe_tx();

                if (CC2420xRTxP__p_tx_buf != (void *)0 && __nesc_ntoh_int8(CC2420xRTxP__tx_setting->priority.nxdata)) {

                    CC2420xRTxP__rtx_status = S_TX_SFD;
                    CC2420xRTxP__cc2420_load_tx();
                  }
                else {
#line 261
                  if (CC2420xRTxP__m_rx_buf.occ_size != 0 && CC2420xRTxP__m_rx_buf.occ_size != CC2420xRTxP__m_rx_buf.max_size) {

                      if (__nesc_ntoh_int8(CC2420xRTxP__rx_setting.ci.nxdata) && __nesc_ntoh_uint8(CC2420xRTxP__rx_setting.hop.nxdata) < 7) {

                          CC2420xRTxP__rtx_status = S_CI_SFD;
                          if (CC2420xRTxP__p_tx_buf == (void *)0) {
                              CC2420xRTxP__p_tx_buf = CC2420xRTxP__m_rx_buf.p_rx_buf;
                            }
                          else 
#line 268
                            {

                              CC2420xRTxP__swap_tx_buf = CC2420xRTxP__p_tx_buf;
                              CC2420xRTxP__p_tx_buf = CC2420xRTxP__m_rx_buf.p_rx_buf;
                            }
                          CC2420xRTxP__tx_counter = 0;
                          CC2420xRTxP__cc2420_load_tx();
                        }
                      else {
#line 275
                        if (__nesc_ntoh_int8(CC2420xRTxP__rx_setting.batched.nxdata)) {

                            radio_flush_tx();
                            CC2420xRTxP__rtx_status = S_RX_DETECT;
                            CC2420xRTxP__rtx_status = S_RX_DETECT;
                            TBCCR5 = TBCCR1 + 288;
                            CC2420xRTxP__detect_duration = TBCCR1;
                            TBCCTL5 = 0x0010;
                          }
                        else 
#line 283
                          {
                            radio_flush_tx();
                            CC2420xRTxP__rtx_status = S_RX_DETECT;
                            CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                          }
                        }
                    }
                  else {
#line 288
                    if (CC2420xRTxP__p_tx_buf != (void *)0) {

                        CC2420xRTxP__rtx_status = S_TX_SFD;
                        CC2420xRTxP__cc2420_load_tx();
                      }
                    else {
#line 292
                      if (CC2420xRTxP__m_rx_buf.occ_size == CC2420xRTxP__m_rx_buf.max_size) {

                          radio_flush_tx();
                          TBCCTL1 = 0x0100;
                          CC2420xRTxP__rtx_time.pkt_ack++;
                          ack_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__ack_duration);
                          CC2420xRTxP__LplTime__timeRadio(&CC2420xRTxP__rtx_time);
                          if (CC2420xRTxP__m_rx_buf.occ_size > 0) {
                              CC2420xRTxP__LplReceive__receive(&CC2420xRTxP__rx_buf[CC2420xRTxP__m_rx_buf.pos_buf], CC2420xRTxP__m_rx_buf.occ_size);
                            }
                          if (CC2420xRTxP__p_tx_buf != (void *)0) {
                              CC2420xRTxP__LplSend__sendDone(CC2420xRTxP__p_tx_buf, CC2420xRTxP__tx_setting, SUCCESS);
                              CC2420xRTxP__p_tx_buf = (void *)0;
                            }
                          CC2420xRTxP__LplTime__timeCompensated(TBR - TBCCR1, &CC2420xRTxP__rtx_time);
                          return;
                        }
                      else 
#line 308
                        {

                          radio_flush_tx();
                          CC2420xRTxP__rtx_status = S_RX_DETECT;
                          TBCCR5 = TBCCR1 + 288;
                          CC2420xRTxP__detect_duration = TBCCR1;
                          TBCCTL5 = 0x0010;
                        }
                      }
                    }
                  }
              }
            else 
#line 316
              {


                if (CC2420xRTxP__p_tx_buf != (void *)0 && __nesc_ntoh_int8(CC2420xRTxP__tx_setting->priority.nxdata)) {
                    CC2420xRTxP__rtx_status = S_TX_SFD;
                    CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                  }
                else {
#line 322
                  if (CC2420xRTxP__m_rx_buf.occ_size != 0 && CC2420xRTxP__m_rx_buf.occ_size != CC2420xRTxP__m_rx_buf.max_size) {
                      if (__nesc_ntoh_int8(CC2420xRTxP__rx_setting.batched.nxdata)) {
                          CC2420xRTxP__rtx_status = S_RX_DETECT;
                          TBCCR5 = TBCCR1 + 288;
                          CC2420xRTxP__detect_duration = TBCCR1;
                          TBCCTL5 = 0x0010;
                        }
                      else 
#line 328
                        {
                          radio_flush_tx();
                          CC2420xRTxP__rtx_status = S_RX_DETECT;
                          CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                        }
                    }
                  else {
#line 333
                    if (CC2420xRTxP__p_tx_buf != (void *)0) {
                        CC2420xRTxP__rtx_status = S_TX_SFD;
                        CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                      }
                    else {
#line 336
                      if (CC2420xRTxP__m_rx_buf.occ_size == CC2420xRTxP__m_rx_buf.max_size) {
                          TBCCTL1 = 0x0100;
                          CC2420xRTxP__rtx_time.pkt_ack++;
                          ack_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__ack_duration);
                          CC2420xRTxP__LplTime__timeRadio(&CC2420xRTxP__rtx_time);
                          if (CC2420xRTxP__m_rx_buf.received) {
                              CC2420xRTxP__LplReceive__receive(&CC2420xRTxP__rx_buf[CC2420xRTxP__m_rx_buf.pos_buf], CC2420xRTxP__m_rx_buf.occ_size);
                            }
                          if (CC2420xRTxP__p_tx_buf != (void *)0) {
                              CC2420xRTxP__LplSend__sendDone(CC2420xRTxP__p_tx_buf, CC2420xRTxP__tx_setting, SUCCESS);
                              CC2420xRTxP__p_tx_buf = (void *)0;
                            }
                          CC2420xRTxP__LplTime__timeCompensated(TBR - TBCCR1, &CC2420xRTxP__rtx_time);
                          return;
                        }
                      else 
#line 350
                        {
                          CC2420xRTxP__rtx_status = S_RX_DETECT;
                          TBCCR5 = TBCCR1 + 288;
                          CC2420xRTxP__detect_duration = TBCCR1;
                          TBCCTL5 = 0x0010;
                        }
                      }
                    }
                  }
              }
#line 358
            CC2420xRTxP__rtx_time.pkt_ack++;
            ack_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__ack_duration);
          }
        else {
#line 360
          if (CC2420xRTxP__rtx_status == S_RX_RECEIVE && !(P4IN & (1 << 1))) {




              CC2420xRTxP__rx_duration = TBCCR1 - CC2420xRTxP__rx_duration;
              TBCCTL3 &= ~(0x0010 | 0x0001);
              CC2420xRTxP__detect_duration = TBCCR1;
              if (CC2420xRTxP__rtx_status == S_TX_SFD && !(P4IN & (1 << 1))) {
#line 368
                ;
                }
              TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;

              if (TB1_irq <= 8) {

                   __asm volatile ("add %[d], r0" :  : [d] "m"(TB1_irq));
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");

                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");
                   __asm volatile ("nop");

                  CC2420xRTxP__cc2420_ack_strobe_rx();
                  CC2420xRTxP__cc2420_ack_wait_tx();
                }

              CC2420xRTxP__cc2420_end_rx();
              CC2420xRTxP__rtx_time.pkt_recv++;
              rx_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__rx_duration);
            }
          else {
#line 397
            if (CC2420xRTxP__rtx_status == S_TX_SFD && !(P4IN & (1 << 1))) {



                CC2420xRTxP__tx_duration = TBCCR1 - CC2420xRTxP__tx_duration;
                CC2420xRTxP__detect_duration = TBCCR1;

                TBCCTL3 &= ~(0x0010 | 0x0001);

                TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;


                if (TB1_irq <= 8) {

                     __asm volatile ("add %[d], r0" :  : [d] "m"(TB1_irq));
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");

                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");
                     __asm volatile ("nop");

                    CC2420xRTxP__cc2420_ack_wait_tx();
                    CC2420xRTxP__rtx_status = S_TX_ACK;
                  }
                else 
#line 429
                  {

                    radio_flush_tx();
                    CC2420xRTxP__rtx_status = S_TX_DETECT;
                    CC2420xRTxP__cc2420_signal_detect(TBCCR1);
                  }

                CC2420xRTxP__rtx_time.pkt_send++;
                tx_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__tx_duration);
              }
            else {
#line 438
              if (CC2420xRTxP__rtx_status == S_CI_SFD && !(P4IN & (1 << 1))) {



                  CC2420xRTxP__detect_duration = TBCCR1;

                  TBCCTL3 &= ~(0x0010 | 0x0001);

                  TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;


                  if (TB1_irq <= 8) {

                       __asm volatile ("add %[d], r0" :  : [d] "m"(TB1_irq));
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");

                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");
                       __asm volatile ("nop");

                      CC2420xRTxP__cc2420_ack_wait_tx();
                      CC2420xRTxP__rtx_status = S_CI_ACK;
                    }
                  else 
#line 469
                    {
                      radio_flush_tx();
                      CC2420xRTxP__rtx_status = S_RTX_IDLE;
                      TBCCR5 = TBCCR1 + 288;
                      CC2420xRTxP__detect_duration = TBCCR1;
                      TBCCTL5 = 0x0010;
                    }

                  CC2420xRTxP__rtx_time.pkt_send++;
                }
              else {
#line 478
                if ((CC2420xRTxP__rtx_status == S_RX_DETECT || CC2420xRTxP__rtx_status == S_TX_DETECT) && P4IN & (1 << 1)) {




                    CC2420xRTxP__detect_duration = TBCCR1 - CC2420xRTxP__detect_duration;
                    CC2420xRTxP__rx_duration = TBCCR1;

                    TBCCTL5 &= ~(0x0010 | 0x0001);

                    TBCCTL1 = ((0x8000 | 0x0100) | 0x0800) | 0x0010;

                    TBCCR3 = TBCCR1 + (CC2420xRTxP__pkt_length * 32 + 200) * 4;
                    CC2420xRTxP__abortion_duration = TBCCR1;
                    TBCCTL3 = 0x0010;

                    CC2420xRTxP__m_rx_buf.received = TRUE;
                    CC2420xRTxP__cc2420_begin_rx();

                    CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
                  }
                else {

                  if (((CC2420xRTxP__rtx_status == S_RX_ACK || CC2420xRTxP__rtx_status == S_TX_ACK) || CC2420xRTxP__rtx_status == S_CI_ACK) && P4IN & (1 << 1)) {




                      CC2420xRTxP__turn_around = TBCCR1 - CC2420xRTxP__turn_around;
                      CC2420xRTxP__ack_duration = TBCCR1;

                      TBCCTL2 &= ~(0x0010 | 0x0001);


                      TBCCR3 = TBCCR1 + (5 * 32 + 200) * 4;
                      CC2420xRTxP__abortion_duration = TBCCR1;
                      TBCCTL3 = 0x0010;

                      TBCCTL1 = ((0x8000 | 0x0100) | 0x0800) | 0x0010;

                      turnaround_time_update(&CC2420xRTxP__rtx_time, CC2420xRTxP__turn_around);
                      CC2420xRTxP__rtx_time.pkt_turnaround++;
                    }
                  else {
#line 520
                    if (CC2420xRTxP__rtx_status == S_CI_SFD && P4IN & (1 << 1)) {



                        TBCCTL1 = ((0x8000 | 0x0100) | 0x0800) | 0x0010;

                        TBCCR3 = TBCCR1 + (CC2420xRTxP__pkt_length * 32 + 200) * 4;
                        CC2420xRTxP__abortion_duration = TBCCR1;
                        TBCCTL3 = 0x0010;

                        CC2420xRTxP__rtx_time.pkt_turnaround++;
                      }
                    else {
#line 531
                      if (CC2420xRTxP__rtx_status == S_TX_SFD && P4IN & (1 << 1)) {



                          CC2420xRTxP__tx_duration = TBCCR1;

                          TBCCTL1 = ((0x8000 | 0x0100) | 0x0800) | 0x0010;

                          TBCCR3 = TBCCR1 + (CC2420xRTxP__pkt_length * 32 + 200) * 4;
                          CC2420xRTxP__abortion_duration = TBCCR1;
                          TBCCTL3 = 0x0010;

                          CC2420xRTxP__rtx_time.pkt_turnaround++;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
    }
  else 
#line 545
    {
      switch (CC2420xRTxP__tbiv) {
          case 4: 
            CC2420xRTxP__detect_duration = TBCCR2;
          TBCCTL2 &= ~(0x0010 | 0x0001);

          if (TB2_irq <= 8) {

               __asm volatile ("add %[d], r0" :  : [d] "m"(TB2_irq));
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");

               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");
               __asm volatile ("nop");

              CC2420xRTxP__cc2420_strobe_tx();

              if (CC2420xRTxP__rtx_status == S_RX_ACK) {

                  if (CC2420xRTxP__p_tx_buf != (void *)0) {
                      CC2420xRTxP__rtx_status = S_TX_SFD;
                      CC2420xRTxP__cc2420_load_tx();
                    }
                  else 
#line 576
                    {
                      radio_flush_tx();
                      CC2420xRTxP__rtx_status = S_RX_DETECT;
                      TBCCR5 = TBCCR1 + 288;
                      CC2420xRTxP__detect_duration = TBCCR2;
                      TBCCTL5 = 0x0010;
                    }
                }
              else {
#line 583
                if (CC2420xRTxP__rtx_status == S_TX_ACK) {
                    CC2420xRTxP__rtx_status = S_TX_SFD;
                    CC2420xRTxP__tx_counter++;

                    write_ram(CC2420_TXFIFO, sizeof(cc2420_header_t ) + sizeof(rtx_setting_t ), &CC2420xRTxP__tx_counter, 1);
                    if (CC2420xRTxP__tx_counter == 64) {
                        (__nesc_temp44 = CC2420xRTxP__tx_setting->size.nxdata, __nesc_hton_uint8(__nesc_temp44, (__nesc_temp45 = __nesc_ntoh_uint8(__nesc_temp44)) - 1), __nesc_temp45);
                        if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
                            CC2420xRTxP__rtx_status = S_TX_SFD;
                            CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
                            CC2420xRTxP__cc2420_load_tx();
                            CC2420xRTxP__tx_counter = 0;
                          }
                        else 
#line 595
                          {
                            radio_flush_tx();
                            CC2420xRTxP__rtx_status = S_RX_DETECT;
                            TBCCR5 = TBCCR1 + 288;
                            CC2420xRTxP__detect_duration = TBCCR2;
                            TBCCTL5 = 0x0010;
                          }
                      }
                  }
                else {
#line 603
                  if (CC2420xRTxP__rtx_status == S_CI_ACK) {
                      CC2420xRTxP__rtx_status = S_CI_SFD;
                      CC2420xRTxP__tx_counter++;
                      write_ram(CC2420_TXFIFO, sizeof(cc2420_header_t ) + sizeof(rtx_setting_t ), &CC2420xRTxP__tx_counter, 1);
                      if (CC2420xRTxP__tx_counter == 64) {
                          if (CC2420xRTxP__swap_tx_buf != (void *)0) {
                              CC2420xRTxP__rtx_status = S_TX_SFD;
                              CC2420xRTxP__p_tx_buf = CC2420xRTxP__swap_tx_buf;
                              CC2420xRTxP__cc2420_load_tx();
                              CC2420xRTxP__swap_tx_buf = (void *)0;
                              CC2420xRTxP__tx_counter = 0;
                            }
                          else 
#line 614
                            {
                              radio_flush_tx();
                              CC2420xRTxP__rtx_status = S_RX_DETECT;
                              TBCCR5 = TBCCR1 + 288;
                              CC2420xRTxP__detect_duration = TBCCR2;
                              TBCCTL5 = 0x0010;
                            }
                        }
                    }
                  }
                }
            }
          else 
#line 623
            {

              if (CC2420xRTxP__p_tx_buf != (void *)0) {
                  CC2420xRTxP__tx_counter++;
                  if (CC2420xRTxP__tx_counter == 64) {
                      (__nesc_temp46 = CC2420xRTxP__tx_setting->size.nxdata, __nesc_hton_uint8(__nesc_temp46, (__nesc_temp47 = __nesc_ntoh_uint8(__nesc_temp46)) - 1), __nesc_temp47);
                      if (__nesc_ntoh_uint8(CC2420xRTxP__tx_setting->size.nxdata) != 0) {
                          CC2420xRTxP__rtx_status = S_TX_SFD;
                          CC2420xRTxP__p_tx_buf = (message_t *)((uint8_t *)CC2420xRTxP__p_tx_buf + sizeof(message_t ));
                          CC2420xRTxP__cc2420_load_tx();
                          CC2420xRTxP__tx_counter = 0;
                        }
                      else 
#line 634
                        {
                          radio_flush_tx();
                          CC2420xRTxP__rtx_status = S_RX_DETECT;
                          TBCCR5 = TBCCR1 + 288;
                          CC2420xRTxP__detect_duration = TBCCR2;
                          TBCCTL5 = 0x0010;
                        }
                    }
                  else 
#line 641
                    {
                      CC2420xRTxP__rtx_status = S_TX_DETECT;
                      CC2420xRTxP__cc2420_signal_detect(TBCCR2);
                    }
                }
              else 
#line 645
                {
                  radio_flush_tx();
                  CC2420xRTxP__rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + 288;
                  CC2420xRTxP__detect_duration = TBCCR2;
                  TBCCTL5 = 0x0010;
                }
            }

          CC2420xRTxP__rtx_time.pkt_ack++;
          CC2420xRTxP__rtx_time.pkt_turnaround++;
          break;
          case 6: 
            TBCCTL3 &= ~(0x0010 | 0x0001);

          radio_flush_tx();
          CC2420xRTxP__rtx_status = S_RX_DETECT;
          CC2420xRTxP__abortion_duration = TBCCR3 - CC2420xRTxP__abortion_duration;
          CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__abortion_duration;
          TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
          TBCCR5 = TBCCR1 + 288;
          CC2420xRTxP__detect_duration = TBCCR1;
          TBCCTL5 = 0x0010;
          break;
          case 10: 
            TBCCTL5 &= ~(0x0010 | 0x0001);

          CC2420xRTxP__detect_duration = TBCCR5 - CC2420xRTxP__detect_duration;
          CC2420xRTxP__rtx_time.channel_detection += CC2420xRTxP__detect_duration;
          CC2420xRTxP__rtx_status = S_RTX_IDLE;
          TBCCTL1 = 0x0100;
          CC2420xRTxP__LplTime__timeRadio(&CC2420xRTxP__rtx_time);
          if (CC2420xRTxP__m_rx_buf.occ_size > 0) {
              CC2420xRTxP__LplReceive__receive(&CC2420xRTxP__rx_buf[CC2420xRTxP__m_rx_buf.pos_buf], CC2420xRTxP__m_rx_buf.occ_size);
            }
          if (CC2420xRTxP__p_tx_buf != (void *)0) {
              CC2420xRTxP__LplSend__sendDone(CC2420xRTxP__p_tx_buf, CC2420xRTxP__tx_setting, SUCCESS);
              CC2420xRTxP__p_tx_buf = (void *)0;
            }
          CC2420xRTxP__LplTime__timeCompensated(TBR - TBCCR5, &CC2420xRTxP__rtx_time);
          break;
          default: 
            CC2420xRTxP__VectorTimerB1__fired();
        }
    }
}

# 185 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static void CC2420xLplP__LplTime__timeRadio(rtx_time_compensation_t *rtx_time)
#line 185
{
  if (CC2420xLplP__lpl_status == LPL_X_IDLE) {
    return;
    }
  /* atomic removed: atomic calls only */
#line 188
  {
    cc2420_rx_stop();
    CC2420xLplP__radio_time_perround = (rtx_time->pkt_recv + rtx_time->pkt_send) * rtx_time->pkt_rtx_time
     + rtx_time->pkt_ack * rtx_time->ack_time
     + rtx_time->pkt_turnaround * rtx_time->turnaround_time
     + rtx_time->channel_detection;
    rtx_time->radio_on_time += CC2420xLplP__radio_time_perround;
    rtx_time->tail_total_time += rtx_time->channel_detection;
    rtx_time->rtx_total_time += (rtx_time->pkt_recv + rtx_time->pkt_send) * rtx_time->pkt_rtx_time;
    rtx_time->ack_total_time += rtx_time->pkt_ack * rtx_time->ack_time;
    rtx_time->turnaround_total_time += rtx_time->pkt_turnaround * rtx_time->turnaround_time;

    CC2420xLplP__print_high = rtx_time->channel_detection >> 16;
    printf_u16(1, &CC2420xLplP__print_high);
    CC2420xLplP__print_low = rtx_time->channel_detection & 0xFFFF;
    printf_u16(1, &CC2420xLplP__print_low);
  }
}

#line 170
static void CC2420xLplP__SubReceive__receive(message_t *msg, uint8_t size)
#line 170
{


  uint8_t i;

#line 174
  if (CC2420xLplP__lpl_status == LPL_X_RX) {
      /* atomic removed: atomic calls only */
#line 175
      {
        radio_flush_rx();
      }
    }

  for (i = 0; i < size; i++) {
      CC2420xLplP__Receive__receive(msg + i, get_packet_payload(msg + i), SUCCESS);
    }
}

#line 154
static void CC2420xLplP__SubSend__sendDone(message_t *msg, rtx_setting_t *ts, error_t error)
#line 154
{

  if (CC2420xLplP__lpl_status != LPL_X_TX) {
    return;
    }
  /* atomic removed: atomic calls only */
#line 158
  {
    radio_flush_tx();
    CC2420xLplP__lpl_dsn++;
  }

  if (__nesc_ntoh_uint8(ts->size.nxdata) == 1) {
      CC2420xLplP__Send__sendDone(msg, error);
    }
  else 
#line 165
    {
      CC2420xLplP__BulkSend__sendDone(msg, error);
    }
}

# 189 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

#line 174
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 174
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 25 "../../tos/chips/cc2420/CC2420ActiveMessageP.nc"
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len)
#line 25
{
  uint8_t size = CC2420ActiveMessageP__LplxPacket__getPacketBulk(msg);

  if (len > CC2420ActiveMessageP__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  CC2420ActiveMessageP__AMPacket__setType(msg, id);
  CC2420ActiveMessageP__AMPacket__setDestination(msg, addr);

  if (size > 1) {
    return CC2420ActiveMessageP__BulkSend__send(msg, len);
    }
#line 37
  return CC2420ActiveMessageP__SubSend__send(msg, len);
}

# 151 "../../tos/chips/cc2420/x-rtx/CC2420xRTxP.nc"
static error_t CC2420xRTxP__LplReceive__rxInit(void )
#line 151
{
  /* atomic removed: atomic calls only */
#line 152
  {
    CC2420xRTxP__m_rx_buf.received = FALSE;
  }
  if (CC2420xRTxP__m_rx_buf.occ_size == CC2420xRTxP__m_rx_buf.max_size) {
    return EBUSY;
    }
#line 157
  return SUCCESS;
}

#line 114
static error_t CC2420xRTxP__LplSend__send(message_t *msg, rtx_setting_t *ts)
#line 114
{
  if (CC2420xRTxP__rtx_status != S_RTX_IDLE) {
    return EBUSY;
    }
#line 117
  if (msg == (void *)0) {
    return FAIL;
    }
#line 119
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    {
      CC2420xRTxP__tx_setting = ts;
      CC2420xRTxP__p_tx_buf = msg;
      CC2420xRTxP__tx_counter = 0;
      CC2420xRTxP__rtx_time.pkt_recv = 0;
      CC2420xRTxP__rtx_time.pkt_send = 0;
      CC2420xRTxP__rtx_time.pkt_ack = 0;
      CC2420xRTxP__rtx_time.pkt_turnaround = 0;
      CC2420xRTxP__rtx_time.channel_detection = 0;
      CC2420xRTxP__rtx_status = S_TX_DETECT;
      TBCCTL1 = ((0x4000 | 0x0100) | 0x0800) | 0x0010;
    }
#line 130
    __nesc_atomic_end(__nesc_atomic); }
  CC2420xRTxP__cc2420_signal_detect(TBR);
  return SUCCESS;
}

# 208 "../../tos/chips/cc2420/x-lpl/CC2420xLplP.nc"
static void CC2420xLplP__LplTime__timeCompensated(uint16_t time, rtx_time_compensation_t *rtx_time)
#line 208
{
  if (CC2420xLplP__lpl_status == LPL_X_IDLE) {
    return;
    }
  /* atomic removed: atomic calls only */
#line 211
  {

    CC2420xLplP__RadioTimerUpdate__counterUpdate(CC2420xLplP__radio_time_perround + time + CC2420xLplP__radio_start_time, rtx_time->calibration_factor);
    CC2420xLplP__RadioTimerUpdate__triggerUpdate();

    CC2420xLplP__SleepTimer__startOneShot(128);
    CC2420xLplP__lpl_status = LPL_X_IDLE;
    enable_other_interrupts(&CC2420xLplP__ie_status);
  }
}

# 146 "../../tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 149
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 63 "/opt/tinyos-main-read-only/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 16 "/opt/tinyos-main-read-only/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 134 "/opt/tinyos-main-read-only/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

# 101 "../../tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else 
#line 135
        {
          /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

#line 74
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 91
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 142 "../../tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
}

# 76 "../../tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 80
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 126 "/opt/tinyos-main-read-only/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 169
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

