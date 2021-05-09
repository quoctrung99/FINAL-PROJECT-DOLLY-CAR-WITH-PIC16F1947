# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2

# 1 "./mcc_generated_files/mcc.h" 1
# 49 "./mcc_generated_files/mcc.h"
# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\xc.h" 1 3
# 18 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\xc8debug.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdlib.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdlib.h" 2 3






# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\features.h" 1 3
# 10 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdlib.h" 2 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdlib.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 122 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 168 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdlib.h" 2 3


int atoi (const char *);
long atol (const char *);
long long atoll (const char *);
double atof (const char *);

float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);
long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);

int rand (void);
void srand (unsigned);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));

__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);
long long llabs (long long);

typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;
typedef struct { long long quot, rem; } lldiv_t;

div_t div (int, int);
ldiv_t ldiv (long, long);
lldiv_t lldiv (long long, long long);

typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);





size_t __ctype_get_mb_cur_max(void);
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\xc8debug.h" 2 3








#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 24 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\xc.h" 2 3



# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 1 3




# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\htc.h" 1 3



# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\xc.h" 1 3
# 5 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\htc.h" 2 3
# 6 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 2 3







# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic_chip_select.h" 1 3
# 449 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic_chip_select.h" 3
# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 1 3
# 44 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\__at.h" 1 3
# 45 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 2 3







extern volatile unsigned char INDF0 __attribute__((address(0x000)));

__asm("INDF0 equ 00h");


typedef union {
    struct {
        unsigned INDF0 :8;
    };
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __attribute__((address(0x000)));
# 72 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char INDF1 __attribute__((address(0x001)));

__asm("INDF1 equ 01h");


typedef union {
    struct {
        unsigned INDF1 :8;
    };
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __attribute__((address(0x001)));
# 92 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");


typedef union {
    struct {
        unsigned PCL :8;
    };
} PCLbits_t;
extern volatile PCLbits_t PCLbits __attribute__((address(0x002)));
# 112 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 171 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0x004)));



extern volatile unsigned char FSR0L __attribute__((address(0x004)));

__asm("FSR0L equ 04h");


typedef union {
    struct {
        unsigned FSR0L :8;
    };
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __attribute__((address(0x004)));
# 195 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR0H __attribute__((address(0x005)));

__asm("FSR0H equ 05h");


typedef union {
    struct {
        unsigned FSR0H :8;
    };
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __attribute__((address(0x005)));
# 215 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short FSR1 __attribute__((address(0x006)));



extern volatile unsigned char FSR1L __attribute__((address(0x006)));

__asm("FSR1L equ 06h");


typedef union {
    struct {
        unsigned FSR1L :8;
    };
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __attribute__((address(0x006)));
# 239 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR1H __attribute__((address(0x007)));

__asm("FSR1H equ 07h");


typedef union {
    struct {
        unsigned FSR1H :8;
    };
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __attribute__((address(0x007)));
# 259 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char BSR __attribute__((address(0x008)));

__asm("BSR equ 08h");


typedef union {
    struct {
        unsigned BSR0 :1;
        unsigned BSR1 :1;
        unsigned BSR2 :1;
        unsigned BSR3 :1;
        unsigned BSR4 :1;
    };
    struct {
        unsigned BSR :5;
    };
} BSRbits_t;
extern volatile BSRbits_t BSRbits __attribute__((address(0x008)));
# 311 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char WREG __attribute__((address(0x009)));

__asm("WREG equ 09h");


typedef union {
    struct {
        unsigned WREG0 :8;
    };
} WREGbits_t;
extern volatile WREGbits_t WREGbits __attribute__((address(0x009)));
# 331 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");


typedef union {
    struct {
        unsigned PCLATH :7;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0x00A)));
# 351 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned IOCIF :1;
        unsigned INTF :1;
        unsigned TMR0IF :1;
        unsigned IOCIE :1;
        unsigned INTE :1;
        unsigned TMR0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :2;
        unsigned T0IF :1;
        unsigned :2;
        unsigned T0IE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 429 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTA __attribute__((address(0x00C)));

__asm("PORTA equ 0Ch");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
        unsigned RA7 :1;
    };
    struct {
        unsigned AN0 :1;
        unsigned AN1 :1;
        unsigned AN2 :1;
        unsigned AN3 :1;
        unsigned :1;
        unsigned AN4 :1;
    };
    struct {
        unsigned CPS0 :1;
        unsigned CPS1 :1;
        unsigned CPS2 :1;
        unsigned CPS3 :1;
        unsigned :1;
        unsigned CPS4 :1;
    };
    struct {
        unsigned SEG33 :1;
        unsigned SEG18 :1;
        unsigned SEG34 :1;
        unsigned SEG35 :1;
        unsigned SEG14 :1;
        unsigned SEG15 :1;
        unsigned SEG36 :1;
        unsigned SEG37 :1;
    };
    struct {
        unsigned :2;
        unsigned VREFM :1;
        unsigned VREFP :1;
        unsigned T0CKI :1;
        unsigned :1;
        unsigned OSC2 :1;
        unsigned OSC1 :1;
    };
    struct {
        unsigned :6;
        unsigned CLKOUT :1;
        unsigned CLKIN :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x00C)));
# 656 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTB __attribute__((address(0x00D)));

__asm("PORTB equ 0Dh");


typedef union {
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
    struct {
        unsigned SEG30 :1;
        unsigned SEG8 :1;
        unsigned SEG9 :1;
        unsigned SEG10 :1;
        unsigned SEG11 :1;
        unsigned SEG29 :1;
        unsigned SEG38 :1;
        unsigned SEG39 :1;
    };
    struct {
        unsigned SRI :1;
        unsigned :4;
        unsigned T1G :1;
    };
    struct {
        unsigned FLT0 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0x00D)));
# 791 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTC __attribute__((address(0x00E)));

__asm("PORTC equ 0Eh");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
    struct {
        unsigned SEG40 :1;
        unsigned SEG32 :1;
        unsigned SEG13 :1;
        unsigned SEG17 :1;
        unsigned SEG16 :1;
        unsigned SEG12 :1;
        unsigned SEG27 :1;
        unsigned SEG28 :1;
    };
    struct {
        unsigned T1OSO :1;
        unsigned T1OSI :1;
        unsigned :1;
        unsigned SCK1 :1;
        unsigned SDI1 :1;
        unsigned SDO1 :1;
        unsigned TX1 :1;
        unsigned RX1 :1;
    };
    struct {
        unsigned T1CKI :1;
        unsigned CCP2 :1;
        unsigned CCP1 :1;
        unsigned SCL1 :1;
        unsigned SDA1 :1;
        unsigned :1;
        unsigned CK1 :1;
        unsigned DT1 :1;
    };
    struct {
        unsigned :1;
        unsigned P2A :1;
        unsigned P1A :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0x00E)));
# 1008 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTD __attribute__((address(0x00F)));

__asm("PORTD equ 0Fh");


typedef union {
    struct {
        unsigned RD0 :1;
        unsigned RD1 :1;
        unsigned RD2 :1;
        unsigned RD3 :1;
        unsigned RD4 :1;
        unsigned RD5 :1;
        unsigned RD6 :1;
        unsigned RD7 :1;
    };
    struct {
        unsigned SEG0 :1;
        unsigned SEG1 :1;
        unsigned SEG2 :1;
        unsigned SEG3 :1;
        unsigned SEG4 :1;
        unsigned SEG5 :1;
        unsigned SEG6 :1;
        unsigned SEG7 :1;
    };
    struct {
        unsigned P2D :1;
        unsigned P2C :1;
        unsigned P2B :1;
        unsigned P3C :1;
        unsigned P3B :1;
        unsigned P1C :1;
        unsigned P1B :1;
    };
    struct {
        unsigned :4;
        unsigned SDO2 :1;
        unsigned SDI2 :1;
        unsigned SCK2 :1;
        unsigned nSS2 :1;
    };
    struct {
        unsigned :5;
        unsigned SDA2 :1;
        unsigned SCL2 :1;
    };
} PORTDbits_t;
extern volatile PORTDbits_t PORTDbits __attribute__((address(0x00F)));
# 1206 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTE __attribute__((address(0x010)));

__asm("PORTE equ 010h");


typedef union {
    struct {
        unsigned RE0 :1;
        unsigned RE1 :1;
        unsigned RE2 :1;
        unsigned RE3 :1;
        unsigned RE4 :1;
        unsigned RE5 :1;
        unsigned RE6 :1;
        unsigned RE7 :1;
    };
    struct {
        unsigned VLCD1 :1;
        unsigned VLCD2 :1;
        unsigned VLCD3 :1;
        unsigned COM0 :1;
        unsigned COM1 :1;
        unsigned COM2 :1;
        unsigned COM3 :1;
        unsigned SEG31 :1;
    };
    struct {
        unsigned P2D :1;
        unsigned P2C :1;
        unsigned P2B :1;
        unsigned P3C :1;
        unsigned P3B :1;
        unsigned P1C :1;
        unsigned P1B :1;
        unsigned P2A :1;
    };
    struct {
        unsigned :7;
        unsigned CCP2 :1;
    };
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __attribute__((address(0x010)));
# 1377 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0x011)));

__asm("PIR1 equ 011h");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned TMR2IF :1;
        unsigned CCP1IF :1;
        unsigned SSP1IF :1;
        unsigned TX1IF :1;
        unsigned RC1IF :1;
        unsigned ADIF :1;
        unsigned TMR1GIF :1;
    };
    struct {
        unsigned :3;
        unsigned SSPIF :1;
        unsigned TXIF :1;
        unsigned RCIF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0x011)));
# 1460 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0x012)));

__asm("PIR2 equ 012h");


typedef union {
    struct {
        unsigned CCP2IF :1;
        unsigned C3IF :1;
        unsigned LCDIF :1;
        unsigned BCLIF :1;
        unsigned EEIF :1;
        unsigned C1IF :1;
        unsigned C2IF :1;
        unsigned OSFIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0x012)));
# 1522 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIR3 __attribute__((address(0x013)));

__asm("PIR3 equ 013h");


typedef union {
    struct {
        unsigned :1;
        unsigned TMR4IF :1;
        unsigned :1;
        unsigned TMR6IF :1;
        unsigned CCP3IF :1;
        unsigned CCP4IF :1;
        unsigned CCP5IF :1;
    };
} PIR3bits_t;
extern volatile PIR3bits_t PIR3bits __attribute__((address(0x013)));
# 1568 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIR4 __attribute__((address(0x014)));

__asm("PIR4 equ 014h");


typedef union {
    struct {
        unsigned SSP2IF :1;
        unsigned BCL2IF :1;
        unsigned :2;
        unsigned TX2IF :1;
        unsigned RC2IF :1;
    };
} PIR4bits_t;
extern volatile PIR4bits_t PIR4bits __attribute__((address(0x014)));
# 1607 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TMR0 __attribute__((address(0x015)));

__asm("TMR0 equ 015h");


typedef union {
    struct {
        unsigned TMR0 :8;
    };
} TMR0bits_t;
extern volatile TMR0bits_t TMR0bits __attribute__((address(0x015)));
# 1627 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0x016)));

__asm("TMR1 equ 016h");




extern volatile unsigned char TMR1L __attribute__((address(0x016)));

__asm("TMR1L equ 016h");


typedef union {
    struct {
        unsigned TMR1L :8;
    };
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __attribute__((address(0x016)));
# 1654 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TMR1H __attribute__((address(0x017)));

__asm("TMR1H equ 017h");


typedef union {
    struct {
        unsigned TMR1H :8;
    };
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __attribute__((address(0x017)));
# 1674 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char T1CON __attribute__((address(0x018)));

__asm("T1CON equ 018h");


typedef union {
    struct {
        unsigned TMR1ON :1;
        unsigned :1;
        unsigned nT1SYNC :1;
        unsigned T1OSCEN :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
        unsigned TMR1CS0 :1;
        unsigned TMR1CS1 :1;
    };
    struct {
        unsigned :4;
        unsigned T1CKPS :2;
        unsigned TMR1CS :2;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0x018)));
# 1746 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char T1GCON __attribute__((address(0x019)));

__asm("T1GCON equ 019h");


typedef union {
    struct {
        unsigned T1GSS0 :1;
        unsigned T1GSS1 :1;
        unsigned T1GVAL :1;
        unsigned T1GGO_nDONE :1;
        unsigned T1GSPM :1;
        unsigned T1GTM :1;
        unsigned T1GPOL :1;
        unsigned TMR1GE :1;
    };
    struct {
        unsigned T1GSS :2;
        unsigned :1;
        unsigned T1GGO :1;
    };
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __attribute__((address(0x019)));
# 1823 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TMR2 __attribute__((address(0x01A)));

__asm("TMR2 equ 01Ah");


typedef union {
    struct {
        unsigned TMR2 :8;
    };
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __attribute__((address(0x01A)));
# 1843 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PR2 __attribute__((address(0x01B)));

__asm("PR2 equ 01Bh");


typedef union {
    struct {
        unsigned PR2 :8;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0x01B)));
# 1863 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char T2CON __attribute__((address(0x01C)));

__asm("T2CON equ 01Ch");


typedef union {
    struct {
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned TMR2ON :1;
        unsigned T2OUTPS0 :1;
        unsigned T2OUTPS1 :1;
        unsigned T2OUTPS2 :1;
        unsigned T2OUTPS3 :1;
    };
    struct {
        unsigned T2CKPS :2;
        unsigned :1;
        unsigned T2OUTPS :4;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0x01C)));
# 1934 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CPSCON0 __attribute__((address(0x01E)));

__asm("CPSCON0 equ 01Eh");


typedef union {
    struct {
        unsigned T0XCS :1;
        unsigned CPSOUT :1;
        unsigned CPSRNG0 :1;
        unsigned CPSRNG1 :1;
        unsigned :2;
        unsigned CPSRM :1;
        unsigned CPSON :1;
    };
    struct {
        unsigned :2;
        unsigned CPSRNG :2;
    };
} CPSCON0bits_t;
extern volatile CPSCON0bits_t CPSCON0bits __attribute__((address(0x01E)));
# 1994 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CPSCON1 __attribute__((address(0x01F)));

__asm("CPSCON1 equ 01Fh");


typedef union {
    struct {
        unsigned CPSCH0 :1;
        unsigned CPSCH1 :1;
        unsigned CPSCH2 :1;
        unsigned CPSCH3 :1;
        unsigned CPSCH4 :1;
    };
    struct {
        unsigned CPSCH :5;
    };
} CPSCON1bits_t;
extern volatile CPSCON1bits_t CPSCON1bits __attribute__((address(0x01F)));
# 2046 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x08C)));

__asm("TRISA equ 08Ch");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
        unsigned TRISA6 :1;
        unsigned TRISA7 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x08C)));
# 2108 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISB __attribute__((address(0x08D)));

__asm("TRISB equ 08Dh");


typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0x08D)));
# 2170 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISC __attribute__((address(0x08E)));

__asm("TRISC equ 08Eh");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0x08E)));
# 2232 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISD __attribute__((address(0x08F)));

__asm("TRISD equ 08Fh");


typedef union {
    struct {
        unsigned TRISD0 :1;
        unsigned TRISD1 :1;
        unsigned TRISD2 :1;
        unsigned TRISD3 :1;
        unsigned TRISD4 :1;
        unsigned TRISD5 :1;
        unsigned TRISD6 :1;
        unsigned TRISD7 :1;
    };
} TRISDbits_t;
extern volatile TRISDbits_t TRISDbits __attribute__((address(0x08F)));
# 2294 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISE __attribute__((address(0x090)));

__asm("TRISE equ 090h");


typedef union {
    struct {
        unsigned TRISE0 :1;
        unsigned TRISE1 :1;
        unsigned TRISE2 :1;
        unsigned TRISE3 :1;
        unsigned TRISE4 :1;
        unsigned TRISE5 :1;
        unsigned TRISE6 :1;
        unsigned TRISE7 :1;
    };
} TRISEbits_t;
extern volatile TRISEbits_t TRISEbits __attribute__((address(0x090)));
# 2356 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0x091)));

__asm("PIE1 equ 091h");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned TMR2IE :1;
        unsigned CCP1IE :1;
        unsigned SSP1IE :1;
        unsigned TX1IE :1;
        unsigned RC1IE :1;
        unsigned ADIE :1;
        unsigned TMR1GIE :1;
    };
    struct {
        unsigned :3;
        unsigned SSPIE :1;
        unsigned TXIE :1;
        unsigned RCIE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0x091)));
# 2439 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0x092)));

__asm("PIE2 equ 092h");


typedef union {
    struct {
        unsigned CCP2IE :1;
        unsigned C3IE :1;
        unsigned LCDIE :1;
        unsigned BCLIE :1;
        unsigned EEIE :1;
        unsigned C1IE :1;
        unsigned C2IE :1;
        unsigned OSFIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0x092)));
# 2501 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIE3 __attribute__((address(0x093)));

__asm("PIE3 equ 093h");


typedef union {
    struct {
        unsigned :1;
        unsigned TMR4IE :1;
        unsigned :1;
        unsigned TMR6IE :1;
        unsigned CCP3IE :1;
        unsigned CCP4IE :1;
        unsigned CCP5IE :1;
    };
} PIE3bits_t;
extern volatile PIE3bits_t PIE3bits __attribute__((address(0x093)));
# 2547 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PIE4 __attribute__((address(0x094)));

__asm("PIE4 equ 094h");


typedef union {
    struct {
        unsigned SSP2IE :1;
        unsigned BCL2IE :1;
        unsigned :2;
        unsigned TX2IE :1;
        unsigned RC2IE :1;
    };
} PIE4bits_t;
extern volatile PIE4bits_t PIE4bits __attribute__((address(0x094)));
# 2586 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char OPTION_REG __attribute__((address(0x095)));

__asm("OPTION_REG equ 095h");


typedef union {
    struct {
        unsigned PS0 :1;
        unsigned PS1 :1;
        unsigned PS2 :1;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned INTEDG :1;
        unsigned nWPUEN :1;
    };
    struct {
        unsigned PS :3;
        unsigned :1;
        unsigned TMR0SE :1;
        unsigned TMR0CS :1;
    };
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __attribute__((address(0x095)));
# 2669 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PCON __attribute__((address(0x096)));

__asm("PCON equ 096h");


typedef union {
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned nRI :1;
        unsigned nRMCLR :1;
        unsigned :2;
        unsigned STKUNF :1;
        unsigned STKOVF :1;
    };
} PCONbits_t;
extern volatile PCONbits_t PCONbits __attribute__((address(0x096)));
# 2720 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0x097)));

__asm("WDTCON equ 097h");


typedef union {
    struct {
        unsigned SWDTEN :1;
        unsigned WDTPS0 :1;
        unsigned WDTPS1 :1;
        unsigned WDTPS2 :1;
        unsigned WDTPS3 :1;
        unsigned WDTPS4 :1;
    };
    struct {
        unsigned :1;
        unsigned WDTPS :5;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0x097)));
# 2779 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0x098)));

__asm("OSCTUNE equ 098h");


typedef union {
    struct {
        unsigned TUN0 :1;
        unsigned TUN1 :1;
        unsigned TUN2 :1;
        unsigned TUN3 :1;
        unsigned TUN4 :1;
        unsigned TUN5 :1;
    };
    struct {
        unsigned TUN :6;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0x098)));
# 2837 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char OSCCON __attribute__((address(0x099)));

__asm("OSCCON equ 099h");


typedef union {
    struct {
        unsigned SCS0 :1;
        unsigned SCS1 :1;
        unsigned :1;
        unsigned IRCF0 :1;
        unsigned IRCF1 :1;
        unsigned IRCF2 :1;
        unsigned IRCF3 :1;
        unsigned SPLLEN :1;
    };
    struct {
        unsigned SCS :2;
        unsigned :1;
        unsigned IRCF :4;
    };
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __attribute__((address(0x099)));
# 2909 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char OSCSTAT __attribute__((address(0x09A)));

__asm("OSCSTAT equ 09Ah");


typedef union {
    struct {
        unsigned HFIOFS :1;
        unsigned LFIOFR :1;
        unsigned MFIOFR :1;
        unsigned HFIOFL :1;
        unsigned HFIOFR :1;
        unsigned OSTS :1;
        unsigned PLLR :1;
        unsigned T1OSCR :1;
    };
} OSCSTATbits_t;
extern volatile OSCSTATbits_t OSCSTATbits __attribute__((address(0x09A)));
# 2971 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short ADRES __attribute__((address(0x09B)));

__asm("ADRES equ 09Bh");




extern volatile unsigned char ADRESL __attribute__((address(0x09B)));

__asm("ADRESL equ 09Bh");


typedef union {
    struct {
        unsigned ADRESL :8;
    };
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __attribute__((address(0x09B)));
# 2998 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ADRESH __attribute__((address(0x09C)));

__asm("ADRESH equ 09Ch");


typedef union {
    struct {
        unsigned ADRESH :8;
    };
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __attribute__((address(0x09C)));
# 3018 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0x09D)));

__asm("ADCON0 equ 09Dh");


typedef union {
    struct {
        unsigned ADON :1;
        unsigned GO_nDONE :1;
        unsigned CHS0 :1;
        unsigned CHS1 :1;
        unsigned CHS2 :1;
        unsigned CHS3 :1;
        unsigned CHS4 :1;
    };
    struct {
        unsigned :1;
        unsigned ADGO :1;
        unsigned CHS :5;
    };
    struct {
        unsigned :1;
        unsigned GO :1;
    };
    struct {
        unsigned :1;
        unsigned nDONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0x09D)));
# 3107 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ADCON1 __attribute__((address(0x09E)));

__asm("ADCON1 equ 09Eh");


typedef union {
    struct {
        unsigned ADPREF0 :1;
        unsigned ADPREF1 :1;
        unsigned ADNREF :1;
        unsigned :1;
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
        unsigned ADCS2 :1;
        unsigned ADFM :1;
    };
    struct {
        unsigned ADPREF :2;
        unsigned :2;
        unsigned ADCS :3;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0x09E)));
# 3179 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATA __attribute__((address(0x10C)));

__asm("LATA equ 010Ch");


typedef union {
    struct {
        unsigned LATA0 :1;
        unsigned LATA1 :1;
        unsigned LATA2 :1;
        unsigned LATA3 :1;
        unsigned LATA4 :1;
        unsigned LATA5 :1;
        unsigned LATA6 :1;
        unsigned LATA7 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0x10C)));
# 3241 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATB __attribute__((address(0x10D)));

__asm("LATB equ 010Dh");


typedef union {
    struct {
        unsigned LATB0 :1;
        unsigned LATB1 :1;
        unsigned LATB2 :1;
        unsigned LATB3 :1;
        unsigned LATB4 :1;
        unsigned LATB5 :1;
        unsigned LATB6 :1;
        unsigned LATB7 :1;
    };
} LATBbits_t;
extern volatile LATBbits_t LATBbits __attribute__((address(0x10D)));
# 3303 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATC __attribute__((address(0x10E)));

__asm("LATC equ 010Eh");


typedef union {
    struct {
        unsigned LATC0 :1;
        unsigned LATC1 :1;
        unsigned LATC2 :1;
        unsigned LATC3 :1;
        unsigned LATC4 :1;
        unsigned LATC5 :1;
        unsigned LATC6 :1;
        unsigned LATC7 :1;
    };
} LATCbits_t;
extern volatile LATCbits_t LATCbits __attribute__((address(0x10E)));
# 3365 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATD __attribute__((address(0x10F)));

__asm("LATD equ 010Fh");


typedef union {
    struct {
        unsigned LATD0 :1;
        unsigned LATD1 :1;
        unsigned LATD2 :1;
        unsigned LATD3 :1;
        unsigned LATD4 :1;
        unsigned LATD5 :1;
        unsigned LATD6 :1;
        unsigned LATD7 :1;
    };
    struct {
        unsigned LATD :8;
    };
} LATDbits_t;
extern volatile LATDbits_t LATDbits __attribute__((address(0x10F)));
# 3435 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATE __attribute__((address(0x110)));

__asm("LATE equ 0110h");


typedef union {
    struct {
        unsigned LATE0 :1;
        unsigned LATE1 :1;
        unsigned LATE2 :1;
        unsigned LATE3 :1;
        unsigned LATE4 :1;
        unsigned LATE5 :1;
        unsigned LATE6 :1;
        unsigned LATE7 :1;
    };
} LATEbits_t;
extern volatile LATEbits_t LATEbits __attribute__((address(0x110)));
# 3497 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM1CON0 __attribute__((address(0x111)));

__asm("CM1CON0 equ 0111h");


typedef union {
    struct {
        unsigned C1SYNC :1;
        unsigned C1HYS :1;
        unsigned C1SP :1;
        unsigned :1;
        unsigned C1POL :1;
        unsigned C1OE :1;
        unsigned C1OUT :1;
        unsigned C1ON :1;
    };
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __attribute__((address(0x111)));
# 3554 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM1CON1 __attribute__((address(0x112)));

__asm("CM1CON1 equ 0112h");


typedef union {
    struct {
        unsigned C1NCH0 :1;
        unsigned C1NCH1 :1;
        unsigned :2;
        unsigned C1PCH0 :1;
        unsigned C1PCH1 :1;
        unsigned C1INTN :1;
        unsigned C1INTP :1;
    };
    struct {
        unsigned C1NCH :2;
        unsigned :2;
        unsigned C1PCH :2;
    };
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __attribute__((address(0x112)));
# 3620 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM2CON0 __attribute__((address(0x113)));

__asm("CM2CON0 equ 0113h");


typedef union {
    struct {
        unsigned C2SYNC :1;
        unsigned C2HYS :1;
        unsigned C2SP :1;
        unsigned :1;
        unsigned C2POL :1;
        unsigned C2OE :1;
        unsigned C2OUT :1;
        unsigned C2ON :1;
    };
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __attribute__((address(0x113)));
# 3677 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM2CON1 __attribute__((address(0x114)));

__asm("CM2CON1 equ 0114h");


typedef union {
    struct {
        unsigned C2NCH0 :1;
        unsigned C2NCH1 :1;
        unsigned :2;
        unsigned C2PCH0 :1;
        unsigned C2PCH1 :1;
        unsigned C2INTN :1;
        unsigned C2INTP :1;
    };
    struct {
        unsigned C2NCH :2;
        unsigned :2;
        unsigned C2PCH :2;
    };
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __attribute__((address(0x114)));
# 3743 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CMOUT __attribute__((address(0x115)));

__asm("CMOUT equ 0115h");


typedef union {
    struct {
        unsigned MC1OUT :1;
        unsigned MC2OUT :1;
        unsigned MC3OUT :1;
    };
} CMOUTbits_t;
extern volatile CMOUTbits_t CMOUTbits __attribute__((address(0x115)));
# 3775 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char BORCON __attribute__((address(0x116)));

__asm("BORCON equ 0116h");


typedef union {
    struct {
        unsigned BORRDY :1;
        unsigned :6;
        unsigned SBOREN :1;
    };
} BORCONbits_t;
extern volatile BORCONbits_t BORCONbits __attribute__((address(0x116)));
# 3802 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FVRCON __attribute__((address(0x117)));

__asm("FVRCON equ 0117h");


typedef union {
    struct {
        unsigned ADFVR0 :1;
        unsigned ADFVR1 :1;
        unsigned CDAFVR0 :1;
        unsigned CDAFVR1 :1;
        unsigned TSRNG :1;
        unsigned TSEN :1;
        unsigned FVRRDY :1;
        unsigned FVREN :1;
    };
    struct {
        unsigned ADFVR :2;
        unsigned CDAFVR :2;
    };
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits __attribute__((address(0x117)));
# 3878 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char DACCON0 __attribute__((address(0x118)));

__asm("DACCON0 equ 0118h");


typedef union {
    struct {
        unsigned DACNSS :1;
        unsigned :1;
        unsigned DACPSS0 :1;
        unsigned DACPSS1 :1;
        unsigned :1;
        unsigned DACOE :1;
        unsigned DACLPS :1;
        unsigned DACEN :1;
    };
    struct {
        unsigned :2;
        unsigned DACPSS :2;
    };
} DACCON0bits_t;
extern volatile DACCON0bits_t DACCON0bits __attribute__((address(0x118)));
# 3939 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char DACCON1 __attribute__((address(0x119)));

__asm("DACCON1 equ 0119h");


typedef union {
    struct {
        unsigned DACR0 :1;
        unsigned DACR1 :1;
        unsigned DACR2 :1;
        unsigned DACR3 :1;
        unsigned DACR4 :1;
    };
    struct {
        unsigned DACR :5;
    };
} DACCON1bits_t;
extern volatile DACCON1bits_t DACCON1bits __attribute__((address(0x119)));
# 3991 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SRCON0 __attribute__((address(0x11A)));

__asm("SRCON0 equ 011Ah");


typedef union {
    struct {
        unsigned SRPR :1;
        unsigned SRPS :1;
        unsigned SRNQEN :1;
        unsigned SRQEN :1;
        unsigned SRCLK0 :1;
        unsigned SRCLK1 :1;
        unsigned SRCLK2 :1;
        unsigned SRLEN :1;
    };
    struct {
        unsigned :4;
        unsigned SRCLK :3;
    };
} SRCON0bits_t;
extern volatile SRCON0bits_t SRCON0bits __attribute__((address(0x11A)));
# 4062 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SRCON1 __attribute__((address(0x11B)));

__asm("SRCON1 equ 011Bh");


typedef union {
    struct {
        unsigned SRRC1E :1;
        unsigned SRRC2E :1;
        unsigned SRRCKE :1;
        unsigned SRRPE :1;
        unsigned SRSC1E :1;
        unsigned SRSC2E :1;
        unsigned SRSCKE :1;
        unsigned SRSPE :1;
    };
} SRCON1bits_t;
extern volatile SRCON1bits_t SRCON1bits __attribute__((address(0x11B)));
# 4124 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char APFCON __attribute__((address(0x11D)));

__asm("APFCON equ 011Dh");


typedef union {
    struct {
        unsigned P1BSEL :1;
        unsigned P1CSEL :1;
        unsigned CCP2SEL :1;
        unsigned P2BSEL :1;
        unsigned P2CSEL :1;
        unsigned P2DSEL :1;
        unsigned P3BSEL :1;
        unsigned P3CSEL :1;
    };
} APFCONbits_t;
extern volatile APFCONbits_t APFCONbits __attribute__((address(0x11D)));
# 4186 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM3CON0 __attribute__((address(0x11E)));

__asm("CM3CON0 equ 011Eh");


typedef union {
    struct {
        unsigned C3SYNC :1;
        unsigned C3HYS :1;
        unsigned C3SP :1;
        unsigned :1;
        unsigned C3POL :1;
        unsigned C3OE :1;
        unsigned C3OUT :1;
        unsigned C3ON :1;
    };
} CM3CON0bits_t;
extern volatile CM3CON0bits_t CM3CON0bits __attribute__((address(0x11E)));
# 4243 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CM3CON1 __attribute__((address(0x11F)));

__asm("CM3CON1 equ 011Fh");


typedef union {
    struct {
        unsigned C3NCH0 :1;
        unsigned C3NCH1 :1;
        unsigned :2;
        unsigned C3PCH0 :1;
        unsigned C3PCH1 :1;
        unsigned C3INTN :1;
        unsigned C3INTP :1;
    };
    struct {
        unsigned C3NCH :2;
        unsigned :2;
        unsigned C3PCH :2;
    };
} CM3CON1bits_t;
extern volatile CM3CON1bits_t CM3CON1bits __attribute__((address(0x11F)));
# 4309 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ANSELA __attribute__((address(0x18C)));

__asm("ANSELA equ 018Ch");


typedef union {
    struct {
        unsigned ANSA0 :1;
        unsigned ANSA1 :1;
        unsigned ANSA2 :1;
        unsigned ANSA3 :1;
        unsigned :1;
        unsigned ANSA5 :1;
    };
    struct {
        unsigned ANSELA :6;
    };
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits __attribute__((address(0x18C)));
# 4362 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ANSELE __attribute__((address(0x190)));

__asm("ANSELE equ 0190h");


typedef union {
    struct {
        unsigned ANSE0 :1;
        unsigned ANSE1 :1;
        unsigned ANSE2 :1;
    };
    struct {
        unsigned ANSELE :3;
    };
} ANSELEbits_t;
extern volatile ANSELEbits_t ANSELEbits __attribute__((address(0x190)));
# 4402 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short EEADR __attribute__((address(0x191)));

__asm("EEADR equ 0191h");




extern volatile unsigned char EEADRL __attribute__((address(0x191)));

__asm("EEADRL equ 0191h");


typedef union {
    struct {
        unsigned EEADRL :8;
    };
} EEADRLbits_t;
extern volatile EEADRLbits_t EEADRLbits __attribute__((address(0x191)));
# 4429 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char EEADRH __attribute__((address(0x192)));

__asm("EEADRH equ 0192h");


typedef union {
    struct {
        unsigned EEADRH :7;
    };
} EEADRHbits_t;
extern volatile EEADRHbits_t EEADRHbits __attribute__((address(0x192)));
# 4449 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short EEDAT __attribute__((address(0x193)));

__asm("EEDAT equ 0193h");




extern volatile unsigned char EEDATL __attribute__((address(0x193)));

__asm("EEDATL equ 0193h");


extern volatile unsigned char EEDATA __attribute__((address(0x193)));

__asm("EEDATA equ 0193h");


typedef union {
    struct {
        unsigned EEDATL :8;
    };
} EEDATLbits_t;
extern volatile EEDATLbits_t EEDATLbits __attribute__((address(0x193)));







typedef union {
    struct {
        unsigned EEDATL :8;
    };
} EEDATAbits_t;
extern volatile EEDATAbits_t EEDATAbits __attribute__((address(0x193)));
# 4494 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char EEDATH __attribute__((address(0x194)));

__asm("EEDATH equ 0194h");


typedef union {
    struct {
        unsigned EEDATH :6;
    };
} EEDATHbits_t;
extern volatile EEDATHbits_t EEDATHbits __attribute__((address(0x194)));
# 4514 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0x195)));

__asm("EECON1 equ 0195h");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned FREE :1;
        unsigned LWLO :1;
        unsigned CFGS :1;
        unsigned EEPGD :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0x195)));
# 4576 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0x196)));

__asm("EECON2 equ 0196h");


typedef union {
    struct {
        unsigned EECON2 :8;
    };
} EECON2bits_t;
extern volatile EECON2bits_t EECON2bits __attribute__((address(0x196)));
# 4596 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char RC1REG __attribute__((address(0x199)));

__asm("RC1REG equ 0199h");


extern volatile unsigned char RCREG __attribute__((address(0x199)));

__asm("RCREG equ 0199h");


typedef union {
    struct {
        unsigned RCREG :8;
    };
} RC1REGbits_t;
extern volatile RC1REGbits_t RC1REGbits __attribute__((address(0x199)));







typedef union {
    struct {
        unsigned RCREG :8;
    };
} RCREGbits_t;
extern volatile RCREGbits_t RCREGbits __attribute__((address(0x199)));
# 4634 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TX1REG __attribute__((address(0x19A)));

__asm("TX1REG equ 019Ah");


extern volatile unsigned char TXREG __attribute__((address(0x19A)));

__asm("TXREG equ 019Ah");


typedef union {
    struct {
        unsigned TXREG :8;
    };
} TX1REGbits_t;
extern volatile TX1REGbits_t TX1REGbits __attribute__((address(0x19A)));







typedef union {
    struct {
        unsigned TXREG :8;
    };
} TXREGbits_t;
extern volatile TXREGbits_t TXREGbits __attribute__((address(0x19A)));
# 4672 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short SP1BRG __attribute__((address(0x19B)));

__asm("SP1BRG equ 019Bh");




extern volatile unsigned char SP1BRGL __attribute__((address(0x19B)));

__asm("SP1BRGL equ 019Bh");


extern volatile unsigned char SPBRG __attribute__((address(0x19B)));

__asm("SPBRG equ 019Bh");

extern volatile unsigned char SPBRGL __attribute__((address(0x19B)));

__asm("SPBRGL equ 019Bh");


typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SP1BRGLbits_t;
extern volatile SP1BRGLbits_t SP1BRGLbits __attribute__((address(0x19B)));







typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __attribute__((address(0x19B)));






typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SPBRGLbits_t;
extern volatile SPBRGLbits_t SPBRGLbits __attribute__((address(0x19B)));
# 4733 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SP1BRGH __attribute__((address(0x19C)));

__asm("SP1BRGH equ 019Ch");


extern volatile unsigned char SPBRGH __attribute__((address(0x19C)));

__asm("SPBRGH equ 019Ch");


typedef union {
    struct {
        unsigned SPBRGH :8;
    };
} SP1BRGHbits_t;
extern volatile SP1BRGHbits_t SP1BRGHbits __attribute__((address(0x19C)));







typedef union {
    struct {
        unsigned SPBRGH :8;
    };
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __attribute__((address(0x19C)));
# 4771 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char RC1STA __attribute__((address(0x19D)));

__asm("RC1STA equ 019Dh");


extern volatile unsigned char RCSTA __attribute__((address(0x19D)));

__asm("RCSTA equ 019Dh");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC1STAbits_t;
extern volatile RC1STAbits_t RC1STAbits __attribute__((address(0x19D)));
# 4836 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0x19D)));
# 4893 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TX1STA __attribute__((address(0x19E)));

__asm("TX1STA equ 019Eh");


extern volatile unsigned char TXSTA __attribute__((address(0x19E)));

__asm("TXSTA equ 019Eh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX1STAbits_t;
extern volatile TX1STAbits_t TX1STAbits __attribute__((address(0x19E)));
# 4958 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0x19E)));
# 5015 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char BAUD1CON __attribute__((address(0x19F)));

__asm("BAUD1CON equ 019Fh");


extern volatile unsigned char BAUDCON __attribute__((address(0x19F)));

__asm("BAUDCON equ 019Fh");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD1CONbits_t;
extern volatile BAUD1CONbits_t BAUD1CONbits __attribute__((address(0x19F)));
# 5070 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCONbits_t;
extern volatile BAUDCONbits_t BAUDCONbits __attribute__((address(0x19F)));
# 5117 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char WPUB __attribute__((address(0x20D)));

__asm("WPUB equ 020Dh");


typedef union {
    struct {
        unsigned WPUB0 :1;
        unsigned WPUB1 :1;
        unsigned WPUB2 :1;
        unsigned WPUB3 :1;
        unsigned WPUB4 :1;
        unsigned WPUB5 :1;
        unsigned WPUB6 :1;
        unsigned WPUB7 :1;
    };
    struct {
        unsigned WPUB :8;
    };
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __attribute__((address(0x20D)));
# 5187 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1BUF __attribute__((address(0x211)));

__asm("SSP1BUF equ 0211h");


extern volatile unsigned char SSPBUF __attribute__((address(0x211)));

__asm("SSPBUF equ 0211h");


typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSP1BUFbits_t;
extern volatile SSP1BUFbits_t SSP1BUFbits __attribute__((address(0x211)));







typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSPBUFbits_t;
extern volatile SSPBUFbits_t SSPBUFbits __attribute__((address(0x211)));
# 5225 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1ADD __attribute__((address(0x212)));

__asm("SSP1ADD equ 0212h");


extern volatile unsigned char SSPADD __attribute__((address(0x212)));

__asm("SSPADD equ 0212h");


typedef union {
    struct {
        unsigned SSPADD :8;
    };
} SSP1ADDbits_t;
extern volatile SSP1ADDbits_t SSP1ADDbits __attribute__((address(0x212)));







typedef union {
    struct {
        unsigned SSPADD :8;
    };
} SSPADDbits_t;
extern volatile SSPADDbits_t SSPADDbits __attribute__((address(0x212)));
# 5263 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1MSK __attribute__((address(0x213)));

__asm("SSP1MSK equ 0213h");


extern volatile unsigned char SSPMSK __attribute__((address(0x213)));

__asm("SSPMSK equ 0213h");


typedef union {
    struct {
        unsigned SSPMSK :8;
    };
} SSP1MSKbits_t;
extern volatile SSP1MSKbits_t SSP1MSKbits __attribute__((address(0x213)));







typedef union {
    struct {
        unsigned SSPMSK :8;
    };
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __attribute__((address(0x213)));
# 5301 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1STAT __attribute__((address(0x214)));

__asm("SSP1STAT equ 0214h");


extern volatile unsigned char SSPSTAT __attribute__((address(0x214)));

__asm("SSPSTAT equ 0214h");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
} SSP1STATbits_t;
extern volatile SSP1STATbits_t SSP1STATbits __attribute__((address(0x214)));
# 5366 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __attribute__((address(0x214)));
# 5423 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1CON1 __attribute__((address(0x215)));

__asm("SSP1CON1 equ 0215h");


extern volatile unsigned char SSPCON __attribute__((address(0x215)));

__asm("SSPCON equ 0215h");

extern volatile unsigned char SSPCON1 __attribute__((address(0x215)));

__asm("SSPCON1 equ 0215h");


typedef union {
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
} SSP1CON1bits_t;
extern volatile SSP1CON1bits_t SSP1CON1bits __attribute__((address(0x215)));
# 5492 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
} SSPCONbits_t;
extern volatile SSPCONbits_t SSPCONbits __attribute__((address(0x215)));
# 5546 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __attribute__((address(0x215)));
# 5603 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1CON2 __attribute__((address(0x216)));

__asm("SSP1CON2 equ 0216h");


extern volatile unsigned char SSPCON2 __attribute__((address(0x216)));

__asm("SSPCON2 equ 0216h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSP1CON2bits_t;
extern volatile SSP1CON2bits_t SSP1CON2bits __attribute__((address(0x216)));
# 5668 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __attribute__((address(0x216)));
# 5725 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP1CON3 __attribute__((address(0x217)));

__asm("SSP1CON3 equ 0217h");


extern volatile unsigned char SSPCON3 __attribute__((address(0x217)));

__asm("SSPCON3 equ 0217h");


typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSP1CON3bits_t;
extern volatile SSP1CON3bits_t SSP1CON3bits __attribute__((address(0x217)));
# 5790 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSPCON3bits_t;
extern volatile SSPCON3bits_t SSPCON3bits __attribute__((address(0x217)));
# 5847 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2BUF __attribute__((address(0x219)));

__asm("SSP2BUF equ 0219h");


typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSP2BUFbits_t;
extern volatile SSP2BUFbits_t SSP2BUFbits __attribute__((address(0x219)));
# 5867 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2ADD __attribute__((address(0x21A)));

__asm("SSP2ADD equ 021Ah");


typedef union {
    struct {
        unsigned SSPADD :8;
    };
} SSP2ADDbits_t;
extern volatile SSP2ADDbits_t SSP2ADDbits __attribute__((address(0x21A)));
# 5887 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2MSK __attribute__((address(0x21B)));

__asm("SSP2MSK equ 021Bh");


typedef union {
    struct {
        unsigned SSPMSK :8;
    };
} SSP2MSKbits_t;
extern volatile SSP2MSKbits_t SSP2MSKbits __attribute__((address(0x21B)));
# 5907 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2STAT __attribute__((address(0x21C)));

__asm("SSP2STAT equ 021Ch");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
} SSP2STATbits_t;
extern volatile SSP2STATbits_t SSP2STATbits __attribute__((address(0x21C)));
# 5969 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2CON1 __attribute__((address(0x21D)));

__asm("SSP2CON1 equ 021Dh");


typedef union {
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
} SSP2CON1bits_t;
extern volatile SSP2CON1bits_t SSP2CON1bits __attribute__((address(0x21D)));
# 6031 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2CON2 __attribute__((address(0x21E)));

__asm("SSP2CON2 equ 021Eh");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSP2CON2bits_t;
extern volatile SSP2CON2bits_t SSP2CON2bits __attribute__((address(0x21E)));
# 6093 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SSP2CON3 __attribute__((address(0x21F)));

__asm("SSP2CON3 equ 021Fh");


typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSP2CON3bits_t;
extern volatile SSP2CON3bits_t SSP2CON3bits __attribute__((address(0x21F)));
# 6155 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTF __attribute__((address(0x28C)));

__asm("PORTF equ 028Ch");


typedef union {
    struct {
        unsigned RF0 :1;
        unsigned RF1 :1;
        unsigned RF2 :1;
        unsigned RF3 :1;
        unsigned RF4 :1;
        unsigned RF5 :1;
        unsigned RF6 :1;
        unsigned RF7 :1;
    };
    struct {
        unsigned AN16 :1;
        unsigned AN6 :1;
        unsigned AN7 :1;
        unsigned AN8 :1;
        unsigned AN9 :1;
        unsigned AN10 :1;
        unsigned AN11 :1;
        unsigned AN5 :1;
    };
    struct {
        unsigned SEG41 :1;
        unsigned SEG19 :1;
        unsigned SEG20 :1;
        unsigned SEG21 :1;
        unsigned SEG22 :1;
        unsigned SEG23 :1;
        unsigned SEG24 :1;
        unsigned SEG25 :1;
    };
    struct {
        unsigned CPS16 :1;
        unsigned CPS6 :1;
        unsigned CPS7 :1;
        unsigned CPS8 :1;
        unsigned CPS9 :1;
        unsigned CPS10 :1;
        unsigned CPS11 :1;
        unsigned CPS5 :1;
    };
    struct {
        unsigned C1IN0N :1;
        unsigned C2OUT :1;
        unsigned C1OUT :1;
        unsigned C1IN2N :1;
        unsigned C2INP :1;
        unsigned C1IN1N :1;
        unsigned C1INP :1;
        unsigned C1IN3N :1;
    };
    struct {
        unsigned C2IN0N :1;
        unsigned SRNQ :1;
        unsigned SRQ :1;
        unsigned C2IN2N :1;
        unsigned :1;
        unsigned C2IN1N :1;
        unsigned :1;
        unsigned C2IN3N :1;
    };
    struct {
        unsigned :3;
        unsigned C3IN2N :1;
        unsigned :1;
        unsigned DACOUT :1;
        unsigned :1;
        unsigned C3IN3N :1;
    };
} PORTFbits_t;
extern volatile PORTFbits_t PORTFbits __attribute__((address(0x28C)));
# 6480 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PORTG __attribute__((address(0x28D)));

__asm("PORTG equ 028Dh");


typedef union {
    struct {
        unsigned RG0 :1;
        unsigned RG1 :1;
        unsigned RG2 :1;
        unsigned RG3 :1;
        unsigned RG4 :1;
        unsigned RG5 :1;
    };
    struct {
        unsigned :1;
        unsigned AN15 :1;
        unsigned AN14 :1;
        unsigned AN13 :1;
        unsigned AN12 :1;
        unsigned nMCLR :1;
    };
    struct {
        unsigned SEG42 :1;
        unsigned SEG43 :1;
        unsigned SEG44 :1;
        unsigned SEG45 :1;
        unsigned SEG26 :1;
    };
    struct {
        unsigned :1;
        unsigned CPS15 :1;
        unsigned CPS14 :1;
        unsigned CPS13 :1;
        unsigned CPS12 :1;
    };
    struct {
        unsigned :1;
        unsigned C3OUT :1;
        unsigned C3INP :1;
        unsigned C3IN0N :1;
        unsigned C3IN1N :1;
    };
    struct {
        unsigned CCP3 :1;
        unsigned TX2 :1;
        unsigned RX2 :1;
        unsigned CCP4 :1;
        unsigned CCP5 :1;
    };
    struct {
        unsigned P3A :1;
        unsigned CK2 :1;
        unsigned DT2 :1;
        unsigned P3D :1;
        unsigned P1D :1;
    };
} PORTGbits_t;
extern volatile PORTGbits_t PORTGbits __attribute__((address(0x28D)));
# 6713 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0x291)));

__asm("CCPR1 equ 0291h");




extern volatile unsigned char CCPR1L __attribute__((address(0x291)));

__asm("CCPR1L equ 0291h");


typedef union {
    struct {
        unsigned CCPR1L :8;
    };
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __attribute__((address(0x291)));
# 6740 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPR1H __attribute__((address(0x292)));

__asm("CCPR1H equ 0292h");


typedef union {
    struct {
        unsigned CCPR1H :8;
    };
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __attribute__((address(0x292)));
# 6760 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP1CON __attribute__((address(0x293)));

__asm("CCP1CON equ 0293h");


typedef union {
    struct {
        unsigned CCP1M0 :1;
        unsigned CCP1M1 :1;
        unsigned CCP1M2 :1;
        unsigned CCP1M3 :1;
        unsigned DC1B0 :1;
        unsigned DC1B1 :1;
        unsigned P1M0 :1;
        unsigned P1M1 :1;
    };
    struct {
        unsigned CCP1M :4;
        unsigned DC1B :2;
        unsigned P1M :2;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0x293)));
# 6842 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PWM1CON __attribute__((address(0x294)));

__asm("PWM1CON equ 0294h");


typedef union {
    struct {
        unsigned P1DC0 :1;
        unsigned P1DC1 :1;
        unsigned P1DC2 :1;
        unsigned P1DC3 :1;
        unsigned P1DC4 :1;
        unsigned P1DC5 :1;
        unsigned P1DC6 :1;
        unsigned P1RSEN :1;
    };
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __attribute__((address(0x294)));
# 6904 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP1AS __attribute__((address(0x295)));

__asm("CCP1AS equ 0295h");


extern volatile unsigned char ECCP1AS __attribute__((address(0x295)));

__asm("ECCP1AS equ 0295h");


typedef union {
    struct {
        unsigned PSS1BD0 :1;
        unsigned PSS1BD1 :1;
        unsigned PSS1AC0 :1;
        unsigned PSS1AC1 :1;
        unsigned CCP1AS0 :1;
        unsigned CCP1AS1 :1;
        unsigned CCP1AS2 :1;
        unsigned CCP1ASE :1;
    };
} CCP1ASbits_t;
extern volatile CCP1ASbits_t CCP1ASbits __attribute__((address(0x295)));
# 6969 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned PSS1BD0 :1;
        unsigned PSS1BD1 :1;
        unsigned PSS1AC0 :1;
        unsigned PSS1AC1 :1;
        unsigned CCP1AS0 :1;
        unsigned CCP1AS1 :1;
        unsigned CCP1AS2 :1;
        unsigned CCP1ASE :1;
    };
} ECCP1ASbits_t;
extern volatile ECCP1ASbits_t ECCP1ASbits __attribute__((address(0x295)));
# 7026 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PSTR1CON __attribute__((address(0x296)));

__asm("PSTR1CON equ 0296h");


typedef union {
    struct {
        unsigned STR1A :1;
        unsigned STR1B :1;
        unsigned STR1C :1;
        unsigned STR1D :1;
        unsigned STR1SYNC :1;
    };
} PSTR1CONbits_t;
extern volatile PSTR1CONbits_t PSTR1CONbits __attribute__((address(0x296)));
# 7070 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short CCPR2 __attribute__((address(0x298)));

__asm("CCPR2 equ 0298h");




extern volatile unsigned char CCPR2L __attribute__((address(0x298)));

__asm("CCPR2L equ 0298h");


typedef union {
    struct {
        unsigned CCPR2L :8;
    };
} CCPR2Lbits_t;
extern volatile CCPR2Lbits_t CCPR2Lbits __attribute__((address(0x298)));
# 7097 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPR2H __attribute__((address(0x299)));

__asm("CCPR2H equ 0299h");


typedef union {
    struct {
        unsigned CCPR2H :8;
    };
} CCPR2Hbits_t;
extern volatile CCPR2Hbits_t CCPR2Hbits __attribute__((address(0x299)));
# 7117 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP2CON __attribute__((address(0x29A)));

__asm("CCP2CON equ 029Ah");


typedef union {
    struct {
        unsigned CCP2M0 :1;
        unsigned CCP2M1 :1;
        unsigned CCP2M2 :1;
        unsigned CCP2M3 :1;
        unsigned DC2B0 :1;
        unsigned DC2B1 :1;
        unsigned P2M0 :1;
        unsigned P2M1 :1;
    };
    struct {
        unsigned CCP2M :4;
        unsigned DC2B :2;
        unsigned P2M :2;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0x29A)));
# 7199 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PWM2CON __attribute__((address(0x29B)));

__asm("PWM2CON equ 029Bh");


typedef union {
    struct {
        unsigned P2DC0 :1;
        unsigned P2DC1 :1;
        unsigned P2DC2 :1;
        unsigned P2DC3 :1;
        unsigned P2DC4 :1;
        unsigned P2DC5 :1;
        unsigned P2DC6 :1;
        unsigned P2RSEN :1;
    };
} PWM2CONbits_t;
extern volatile PWM2CONbits_t PWM2CONbits __attribute__((address(0x29B)));
# 7261 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP2AS __attribute__((address(0x29C)));

__asm("CCP2AS equ 029Ch");


extern volatile unsigned char ECCP2AS __attribute__((address(0x29C)));

__asm("ECCP2AS equ 029Ch");


typedef union {
    struct {
        unsigned PSS2BD0 :1;
        unsigned PSS2BD1 :1;
        unsigned PSS2AC0 :1;
        unsigned PSS2AC1 :1;
        unsigned CCP2AS0 :1;
        unsigned CCP2AS1 :1;
        unsigned CCP2AS2 :1;
        unsigned CCP2ASE :1;
    };
} CCP2ASbits_t;
extern volatile CCP2ASbits_t CCP2ASbits __attribute__((address(0x29C)));
# 7326 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned PSS2BD0 :1;
        unsigned PSS2BD1 :1;
        unsigned PSS2AC0 :1;
        unsigned PSS2AC1 :1;
        unsigned CCP2AS0 :1;
        unsigned CCP2AS1 :1;
        unsigned CCP2AS2 :1;
        unsigned CCP2ASE :1;
    };
} ECCP2ASbits_t;
extern volatile ECCP2ASbits_t ECCP2ASbits __attribute__((address(0x29C)));
# 7383 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PSTR2CON __attribute__((address(0x29D)));

__asm("PSTR2CON equ 029Dh");


typedef union {
    struct {
        unsigned STR2A :1;
        unsigned STR2B :1;
        unsigned STR2C :1;
        unsigned STR2D :1;
        unsigned STR2SYNC :1;
    };
} PSTR2CONbits_t;
extern volatile PSTR2CONbits_t PSTR2CONbits __attribute__((address(0x29D)));
# 7427 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPTMRS0 __attribute__((address(0x29E)));

__asm("CCPTMRS0 equ 029Eh");


typedef union {
    struct {
        unsigned C1TSEL0 :1;
        unsigned C1TSEL1 :1;
        unsigned C2TSEL0 :1;
        unsigned C2TSEL1 :1;
        unsigned C3TSEL0 :1;
        unsigned C3TSEL1 :1;
        unsigned C4TSEL0 :1;
        unsigned C4TSEL1 :1;
    };
    struct {
        unsigned C1TSEL :2;
        unsigned C2TSEL :2;
        unsigned C3TSEL :2;
        unsigned C4TSEL :2;
    };
} CCPTMRS0bits_t;
extern volatile CCPTMRS0bits_t CCPTMRS0bits __attribute__((address(0x29E)));
# 7515 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPTMRS1 __attribute__((address(0x29F)));

__asm("CCPTMRS1 equ 029Fh");


typedef union {
    struct {
        unsigned C5TSEL0 :1;
        unsigned C5TSEL1 :1;
    };
    struct {
        unsigned C5TSEL :2;
    };
} CCPTMRS1bits_t;
extern volatile CCPTMRS1bits_t CCPTMRS1bits __attribute__((address(0x29F)));
# 7549 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISF __attribute__((address(0x30C)));

__asm("TRISF equ 030Ch");


typedef union {
    struct {
        unsigned TRISF0 :1;
        unsigned TRISF1 :1;
        unsigned TRISF2 :1;
        unsigned TRISF3 :1;
        unsigned TRISF4 :1;
        unsigned TRISF5 :1;
        unsigned TRISF6 :1;
        unsigned TRISF7 :1;
    };
} TRISFbits_t;
extern volatile TRISFbits_t TRISFbits __attribute__((address(0x30C)));
# 7611 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TRISG __attribute__((address(0x30D)));

__asm("TRISG equ 030Dh");


typedef union {
    struct {
        unsigned TRISG0 :1;
        unsigned TRISG1 :1;
        unsigned TRISG2 :1;
        unsigned TRISG3 :1;
        unsigned TRISG4 :1;
        unsigned TRISG5 :1;
    };
} TRISGbits_t;
extern volatile TRISGbits_t TRISGbits __attribute__((address(0x30D)));
# 7661 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short CCPR3 __attribute__((address(0x311)));

__asm("CCPR3 equ 0311h");




extern volatile unsigned char CCPR3L __attribute__((address(0x311)));

__asm("CCPR3L equ 0311h");


typedef union {
    struct {
        unsigned CCPR3L :8;
    };
} CCPR3Lbits_t;
extern volatile CCPR3Lbits_t CCPR3Lbits __attribute__((address(0x311)));
# 7688 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPR3H __attribute__((address(0x312)));

__asm("CCPR3H equ 0312h");


typedef union {
    struct {
        unsigned CCPR3H :8;
    };
} CCPR3Hbits_t;
extern volatile CCPR3Hbits_t CCPR3Hbits __attribute__((address(0x312)));
# 7708 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP3CON __attribute__((address(0x313)));

__asm("CCP3CON equ 0313h");


typedef union {
    struct {
        unsigned CCP3M0 :1;
        unsigned CCP3M1 :1;
        unsigned CCP3M2 :1;
        unsigned CCP3M3 :1;
        unsigned DC3B0 :1;
        unsigned DC3B1 :1;
        unsigned P3M0 :1;
        unsigned P3M1 :1;
    };
} CCP3CONbits_t;
extern volatile CCP3CONbits_t CCP3CONbits __attribute__((address(0x313)));
# 7770 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PWM3CON __attribute__((address(0x314)));

__asm("PWM3CON equ 0314h");


typedef union {
    struct {
        unsigned P3DC0 :1;
        unsigned P3DC1 :1;
        unsigned P3DC2 :1;
        unsigned P3DC3 :1;
        unsigned P3DC4 :1;
        unsigned P3DC5 :1;
        unsigned P3DC6 :1;
        unsigned P3RSEN :1;
    };
} PWM3CONbits_t;
extern volatile PWM3CONbits_t PWM3CONbits __attribute__((address(0x314)));
# 7832 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP3AS __attribute__((address(0x315)));

__asm("CCP3AS equ 0315h");


extern volatile unsigned char ECCP3AS __attribute__((address(0x315)));

__asm("ECCP3AS equ 0315h");


typedef union {
    struct {
        unsigned PSS3BD0 :1;
        unsigned PSS3BD1 :1;
        unsigned PSS3AC0 :1;
        unsigned PSS3AC1 :1;
        unsigned CCP3AS0 :1;
        unsigned CCP3AS1 :1;
        unsigned CCP3AS2 :1;
        unsigned CCP3ASE :1;
    };
} CCP3ASbits_t;
extern volatile CCP3ASbits_t CCP3ASbits __attribute__((address(0x315)));
# 7897 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
typedef union {
    struct {
        unsigned PSS3BD0 :1;
        unsigned PSS3BD1 :1;
        unsigned PSS3AC0 :1;
        unsigned PSS3AC1 :1;
        unsigned CCP3AS0 :1;
        unsigned CCP3AS1 :1;
        unsigned CCP3AS2 :1;
        unsigned CCP3ASE :1;
    };
} ECCP3ASbits_t;
extern volatile ECCP3ASbits_t ECCP3ASbits __attribute__((address(0x315)));
# 7954 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PSTR3CON __attribute__((address(0x316)));

__asm("PSTR3CON equ 0316h");


typedef union {
    struct {
        unsigned STR3A :1;
        unsigned STR3B :1;
        unsigned STR3C :1;
        unsigned STR3D :1;
        unsigned STR3SYNC :1;
    };
} PSTR3CONbits_t;
extern volatile PSTR3CONbits_t PSTR3CONbits __attribute__((address(0x316)));
# 7998 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short CCPR4 __attribute__((address(0x318)));

__asm("CCPR4 equ 0318h");




extern volatile unsigned char CCPR4L __attribute__((address(0x318)));

__asm("CCPR4L equ 0318h");


typedef union {
    struct {
        unsigned CCPR4L :8;
    };
} CCPR4Lbits_t;
extern volatile CCPR4Lbits_t CCPR4Lbits __attribute__((address(0x318)));
# 8025 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPR4H __attribute__((address(0x319)));

__asm("CCPR4H equ 0319h");


typedef union {
    struct {
        unsigned CCPR4H :8;
    };
} CCPR4Hbits_t;
extern volatile CCPR4Hbits_t CCPR4Hbits __attribute__((address(0x319)));
# 8045 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP4CON __attribute__((address(0x31A)));

__asm("CCP4CON equ 031Ah");


typedef union {
    struct {
        unsigned CCP4M0 :1;
        unsigned CCP4M1 :1;
        unsigned CCP4M2 :1;
        unsigned CCP4M3 :1;
        unsigned DC4B0 :1;
        unsigned DC4B1 :1;
    };
} CCP4CONbits_t;
extern volatile CCP4CONbits_t CCP4CONbits __attribute__((address(0x31A)));
# 8095 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned short CCPR5 __attribute__((address(0x31C)));

__asm("CCPR5 equ 031Ch");




extern volatile unsigned char CCPR5L __attribute__((address(0x31C)));

__asm("CCPR5L equ 031Ch");


typedef union {
    struct {
        unsigned CCPR5L :8;
    };
} CCPR5Lbits_t;
extern volatile CCPR5Lbits_t CCPR5Lbits __attribute__((address(0x31C)));
# 8122 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCPR5H __attribute__((address(0x31D)));

__asm("CCPR5H equ 031Dh");


typedef union {
    struct {
        unsigned CCPR5H :8;
    };
} CCPR5Hbits_t;
extern volatile CCPR5Hbits_t CCPR5Hbits __attribute__((address(0x31D)));
# 8142 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char CCP5CON __attribute__((address(0x31E)));

__asm("CCP5CON equ 031Eh");


typedef union {
    struct {
        unsigned CCP5M0 :1;
        unsigned CCP5M1 :1;
        unsigned CCP5M2 :1;
        unsigned CCP5M3 :1;
        unsigned DC5B0 :1;
        unsigned DC5B1 :1;
    };
} CCP5CONbits_t;
extern volatile CCP5CONbits_t CCP5CONbits __attribute__((address(0x31E)));
# 8192 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATF __attribute__((address(0x38C)));

__asm("LATF equ 038Ch");


typedef union {
    struct {
        unsigned LATF0 :1;
        unsigned LATF1 :1;
        unsigned LATF2 :1;
        unsigned LATF3 :1;
        unsigned LATF4 :1;
        unsigned LATF5 :1;
        unsigned LATF6 :1;
        unsigned LATF7 :1;
    };
} LATFbits_t;
extern volatile LATFbits_t LATFbits __attribute__((address(0x38C)));
# 8254 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LATG __attribute__((address(0x38D)));

__asm("LATG equ 038Dh");


typedef union {
    struct {
        unsigned LATG0 :1;
        unsigned LATG1 :1;
        unsigned LATG2 :1;
        unsigned LATG3 :1;
        unsigned LATG4 :1;
        unsigned LATG5 :1;
    };
} LATGbits_t;
extern volatile LATGbits_t LATGbits __attribute__((address(0x38D)));
# 8304 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char IOCBP __attribute__((address(0x394)));

__asm("IOCBP equ 0394h");


typedef union {
    struct {
        unsigned IOCBP0 :1;
        unsigned IOCBP1 :1;
        unsigned IOCBP2 :1;
        unsigned IOCBP3 :1;
        unsigned IOCBP4 :1;
        unsigned IOCBP5 :1;
        unsigned IOCBP6 :1;
        unsigned IOCBP7 :1;
    };
    struct {
        unsigned IOCBP :8;
    };
} IOCBPbits_t;
extern volatile IOCBPbits_t IOCBPbits __attribute__((address(0x394)));
# 8374 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char IOCBN __attribute__((address(0x395)));

__asm("IOCBN equ 0395h");


typedef union {
    struct {
        unsigned IOCBN0 :1;
        unsigned IOCBN1 :1;
        unsigned IOCBN2 :1;
        unsigned IOCBN3 :1;
        unsigned IOCBN4 :1;
        unsigned IOCBN5 :1;
        unsigned IOCBN6 :1;
        unsigned IOCBN7 :1;
    };
    struct {
        unsigned IOCBN :8;
    };
} IOCBNbits_t;
extern volatile IOCBNbits_t IOCBNbits __attribute__((address(0x395)));
# 8444 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char IOCBF __attribute__((address(0x396)));

__asm("IOCBF equ 0396h");


typedef union {
    struct {
        unsigned IOCBF0 :1;
        unsigned IOCBF1 :1;
        unsigned IOCBF2 :1;
        unsigned IOCBF3 :1;
        unsigned IOCBF4 :1;
        unsigned IOCBF5 :1;
        unsigned IOCBF6 :1;
        unsigned IOCBF7 :1;
    };
    struct {
        unsigned IOCBF :8;
    };
} IOCBFbits_t;
extern volatile IOCBFbits_t IOCBFbits __attribute__((address(0x396)));
# 8514 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ANSELF __attribute__((address(0x40C)));

__asm("ANSELF equ 040Ch");


typedef union {
    struct {
        unsigned ANSF0 :1;
        unsigned ANSF1 :1;
        unsigned ANSF2 :1;
        unsigned ANSF3 :1;
        unsigned ANSF4 :1;
        unsigned ANSF5 :1;
        unsigned ANSF6 :1;
        unsigned ANSF7 :1;
    };
    struct {
        unsigned ANSELF :8;
    };
} ANSELFbits_t;
extern volatile ANSELFbits_t ANSELFbits __attribute__((address(0x40C)));
# 8584 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char ANSELG __attribute__((address(0x40D)));

__asm("ANSELG equ 040Dh");


typedef union {
    struct {
        unsigned :1;
        unsigned ANSG1 :1;
        unsigned ANSG2 :1;
        unsigned ANSG3 :1;
        unsigned ANSG4 :1;
    };
    struct {
        unsigned :1;
        unsigned ANSELG :4;
    };
} ANSELGbits_t;
extern volatile ANSELGbits_t ANSELGbits __attribute__((address(0x40D)));
# 8632 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TMR4 __attribute__((address(0x415)));

__asm("TMR4 equ 0415h");


typedef union {
    struct {
        unsigned TMR4 :8;
    };
} TMR4bits_t;
extern volatile TMR4bits_t TMR4bits __attribute__((address(0x415)));
# 8652 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PR4 __attribute__((address(0x416)));

__asm("PR4 equ 0416h");


typedef union {
    struct {
        unsigned PR4 :8;
    };
} PR4bits_t;
extern volatile PR4bits_t PR4bits __attribute__((address(0x416)));
# 8672 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char T4CON __attribute__((address(0x417)));

__asm("T4CON equ 0417h");


typedef union {
    struct {
        unsigned T4CKPS0 :1;
        unsigned T4CKPS1 :1;
        unsigned TMR4ON :1;
        unsigned T4OUTPS0 :1;
        unsigned T4OUTPS1 :1;
        unsigned T4OUTPS2 :1;
        unsigned T4OUTPS3 :1;
    };
    struct {
        unsigned T4CKPS :2;
        unsigned :1;
        unsigned T4OUTPS :4;
    };
} T4CONbits_t;
extern volatile T4CONbits_t T4CONbits __attribute__((address(0x417)));
# 8743 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TMR6 __attribute__((address(0x41C)));

__asm("TMR6 equ 041Ch");


typedef union {
    struct {
        unsigned TMR6 :8;
    };
} TMR6bits_t;
extern volatile TMR6bits_t TMR6bits __attribute__((address(0x41C)));
# 8763 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PR6 __attribute__((address(0x41D)));

__asm("PR6 equ 041Dh");


typedef union {
    struct {
        unsigned PR6 :8;
    };
} PR6bits_t;
extern volatile PR6bits_t PR6bits __attribute__((address(0x41D)));
# 8783 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char T6CON __attribute__((address(0x41E)));

__asm("T6CON equ 041Eh");


typedef union {
    struct {
        unsigned T6CKPS0 :1;
        unsigned T6CKPS1 :1;
        unsigned TMR6ON :1;
        unsigned T6OUTPS0 :1;
        unsigned T6OUTPS1 :1;
        unsigned T6OUTPS2 :1;
        unsigned T6OUTPS3 :1;
    };
    struct {
        unsigned T6CKPS :2;
        unsigned :1;
        unsigned T6OUTPS :4;
    };
} T6CONbits_t;
extern volatile T6CONbits_t T6CONbits __attribute__((address(0x41E)));
# 8854 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char WPUG __attribute__((address(0x48D)));

__asm("WPUG equ 048Dh");


typedef union {
    struct {
        unsigned :5;
        unsigned WPUG5 :1;
    };
} WPUGbits_t;
extern volatile WPUGbits_t WPUGbits __attribute__((address(0x48D)));
# 8875 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char RC2REG __attribute__((address(0x491)));

__asm("RC2REG equ 0491h");


typedef union {
    struct {
        unsigned RCREG :8;
    };
} RC2REGbits_t;
extern volatile RC2REGbits_t RC2REGbits __attribute__((address(0x491)));
# 8895 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TX2REG __attribute__((address(0x492)));

__asm("TX2REG equ 0492h");


typedef union {
    struct {
        unsigned TXREG :8;
    };
} TX2REGbits_t;
extern volatile TX2REGbits_t TX2REGbits __attribute__((address(0x492)));
# 8915 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SP2BRGL __attribute__((address(0x493)));

__asm("SP2BRGL equ 0493h");


extern volatile unsigned char SPBRG2 __attribute__((address(0x493)));

__asm("SPBRG2 equ 0493h");


typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SP2BRGLbits_t;
extern volatile SP2BRGLbits_t SP2BRGLbits __attribute__((address(0x493)));







typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SPBRG2bits_t;
extern volatile SPBRG2bits_t SPBRG2bits __attribute__((address(0x493)));
# 8953 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char SP2BRGH __attribute__((address(0x494)));

__asm("SP2BRGH equ 0494h");


typedef union {
    struct {
        unsigned SPBRGH :8;
    };
} SP2BRGHbits_t;
extern volatile SP2BRGHbits_t SP2BRGHbits __attribute__((address(0x494)));
# 8973 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char RC2STA __attribute__((address(0x495)));

__asm("RC2STA equ 0495h");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC2STAbits_t;
extern volatile RC2STAbits_t RC2STAbits __attribute__((address(0x495)));
# 9035 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TX2STA __attribute__((address(0x496)));

__asm("TX2STA equ 0496h");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX2STAbits_t;
extern volatile TX2STAbits_t TX2STAbits __attribute__((address(0x496)));
# 9097 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char BAUD2CON __attribute__((address(0x497)));

__asm("BAUD2CON equ 0497h");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD2CONbits_t;
extern volatile BAUD2CONbits_t BAUD2CONbits __attribute__((address(0x497)));
# 9149 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDCON __attribute__((address(0x791)));

__asm("LCDCON equ 0791h");


typedef union {
    struct {
        unsigned LMUX0 :1;
        unsigned LMUX1 :1;
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned :1;
        unsigned WERR :1;
        unsigned SLPEN :1;
        unsigned LCDEN :1;
    };
    struct {
        unsigned LMUX :2;
        unsigned CS :2;
    };
} LCDCONbits_t;
extern volatile LCDCONbits_t LCDCONbits __attribute__((address(0x791)));
# 9220 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDPS __attribute__((address(0x792)));

__asm("LCDPS equ 0792h");


typedef union {
    struct {
        unsigned LP0 :1;
        unsigned LP1 :1;
        unsigned LP2 :1;
        unsigned LP3 :1;
        unsigned WA :1;
        unsigned LCDA :1;
        unsigned BIASMD :1;
        unsigned WFT :1;
    };
    struct {
        unsigned LP :4;
    };
} LCDPSbits_t;
extern volatile LCDPSbits_t LCDPSbits __attribute__((address(0x792)));
# 9290 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDREF __attribute__((address(0x793)));

__asm("LCDREF equ 0793h");


typedef union {
    struct {
        unsigned :1;
        unsigned VLCD1PE :1;
        unsigned VLCD2PE :1;
        unsigned VLCD3PE :1;
        unsigned :1;
        unsigned LCDIRI :1;
        unsigned LCDIRS :1;
        unsigned LCDIRE :1;
    };
} LCDREFbits_t;
extern volatile LCDREFbits_t LCDREFbits __attribute__((address(0x793)));
# 9342 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDCST __attribute__((address(0x794)));

__asm("LCDCST equ 0794h");


typedef union {
    struct {
        unsigned LCDCST0 :1;
        unsigned LCDCST1 :1;
        unsigned LCDCST2 :1;
    };
    struct {
        unsigned LCDCST :3;
    };
} LCDCSTbits_t;
extern volatile LCDCSTbits_t LCDCSTbits __attribute__((address(0x794)));
# 9382 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDRL __attribute__((address(0x795)));

__asm("LCDRL equ 0795h");


typedef union {
    struct {
        unsigned LRLAT0 :1;
        unsigned LRLAT1 :1;
        unsigned LRLAT2 :1;
        unsigned :1;
        unsigned LRLBP0 :1;
        unsigned LRLBP1 :1;
        unsigned LRLAP0 :1;
        unsigned LRLAP1 :1;
    };
    struct {
        unsigned LRLAT :3;
        unsigned :1;
        unsigned LRLBP :2;
        unsigned LRLAP :2;
    };
} LCDRLbits_t;
extern volatile LCDRLbits_t LCDRLbits __attribute__((address(0x795)));
# 9460 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE0 __attribute__((address(0x798)));

__asm("LCDSE0 equ 0798h");


typedef union {
    struct {
        unsigned SE0 :1;
        unsigned SE1 :1;
        unsigned SE2 :1;
        unsigned SE3 :1;
        unsigned SE4 :1;
        unsigned SE5 :1;
        unsigned SE6 :1;
        unsigned SE7 :1;
    };
} LCDSE0bits_t;
extern volatile LCDSE0bits_t LCDSE0bits __attribute__((address(0x798)));
# 9522 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE1 __attribute__((address(0x799)));

__asm("LCDSE1 equ 0799h");


typedef union {
    struct {
        unsigned SE8 :1;
        unsigned SE9 :1;
        unsigned SE10 :1;
        unsigned SE11 :1;
        unsigned SE12 :1;
        unsigned SE13 :1;
        unsigned SE14 :1;
        unsigned SE15 :1;
    };
} LCDSE1bits_t;
extern volatile LCDSE1bits_t LCDSE1bits __attribute__((address(0x799)));
# 9584 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE2 __attribute__((address(0x79A)));

__asm("LCDSE2 equ 079Ah");


typedef union {
    struct {
        unsigned SE16 :1;
        unsigned SE17 :1;
        unsigned SE18 :1;
        unsigned SE19 :1;
        unsigned SE20 :1;
        unsigned SE21 :1;
        unsigned SE22 :1;
        unsigned SE23 :1;
    };
} LCDSE2bits_t;
extern volatile LCDSE2bits_t LCDSE2bits __attribute__((address(0x79A)));
# 9646 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE3 __attribute__((address(0x79B)));

__asm("LCDSE3 equ 079Bh");


typedef union {
    struct {
        unsigned SE24 :1;
        unsigned SE25 :1;
        unsigned SE26 :1;
        unsigned SE27 :1;
        unsigned SE28 :1;
        unsigned SE29 :1;
        unsigned SE30 :1;
        unsigned SE31 :1;
    };
} LCDSE3bits_t;
extern volatile LCDSE3bits_t LCDSE3bits __attribute__((address(0x79B)));
# 9708 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE4 __attribute__((address(0x79C)));

__asm("LCDSE4 equ 079Ch");


typedef union {
    struct {
        unsigned SE32 :1;
        unsigned SE33 :1;
        unsigned SE34 :1;
        unsigned SE35 :1;
        unsigned SE36 :1;
        unsigned SE37 :1;
        unsigned SE38 :1;
        unsigned SE39 :1;
    };
} LCDSE4bits_t;
extern volatile LCDSE4bits_t LCDSE4bits __attribute__((address(0x79C)));
# 9770 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDSE5 __attribute__((address(0x79D)));

__asm("LCDSE5 equ 079Dh");


typedef union {
    struct {
        unsigned SE40 :1;
        unsigned SE41 :1;
        unsigned SE42 :1;
        unsigned SE43 :1;
        unsigned SE44 :1;
        unsigned SE45 :1;
    };
} LCDSE5bits_t;
extern volatile LCDSE5bits_t LCDSE5bits __attribute__((address(0x79D)));
# 9820 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA0 __attribute__((address(0x7A0)));

__asm("LCDDATA0 equ 07A0h");


typedef union {
    struct {
        unsigned SEG0COM0 :1;
        unsigned SEG1COM0 :1;
        unsigned SEG2COM0 :1;
        unsigned SEG3COM0 :1;
        unsigned SEG4COM0 :1;
        unsigned SEG5COM0 :1;
        unsigned SEG6COM0 :1;
        unsigned SEG7COM0 :1;
    };
} LCDDATA0bits_t;
extern volatile LCDDATA0bits_t LCDDATA0bits __attribute__((address(0x7A0)));
# 9882 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA1 __attribute__((address(0x7A1)));

__asm("LCDDATA1 equ 07A1h");


typedef union {
    struct {
        unsigned SEG8COM0 :1;
        unsigned SEG9COM0 :1;
        unsigned SEG10COM0 :1;
        unsigned SEG11COM0 :1;
        unsigned SEG12COM0 :1;
        unsigned SEG13COM0 :1;
        unsigned SEG14COM0 :1;
        unsigned SEG15COM0 :1;
    };
} LCDDATA1bits_t;
extern volatile LCDDATA1bits_t LCDDATA1bits __attribute__((address(0x7A1)));
# 9944 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA2 __attribute__((address(0x7A2)));

__asm("LCDDATA2 equ 07A2h");


typedef union {
    struct {
        unsigned SEG16COM0 :1;
        unsigned SEG17COM0 :1;
        unsigned SEG18COM0 :1;
        unsigned SEG19COM0 :1;
        unsigned SEG20COM0 :1;
        unsigned SEG21COM0 :1;
        unsigned SEG22COM0 :1;
        unsigned SEG23COM0 :1;
    };
} LCDDATA2bits_t;
extern volatile LCDDATA2bits_t LCDDATA2bits __attribute__((address(0x7A2)));
# 10006 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA3 __attribute__((address(0x7A3)));

__asm("LCDDATA3 equ 07A3h");


typedef union {
    struct {
        unsigned SEG0COM1 :1;
        unsigned SEG1COM1 :1;
        unsigned SEG2COM1 :1;
        unsigned SEG3COM1 :1;
        unsigned SEG4COM1 :1;
        unsigned SEG5COM1 :1;
        unsigned SEG6COM1 :1;
        unsigned SEG7COM1 :1;
    };
} LCDDATA3bits_t;
extern volatile LCDDATA3bits_t LCDDATA3bits __attribute__((address(0x7A3)));
# 10068 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA4 __attribute__((address(0x7A4)));

__asm("LCDDATA4 equ 07A4h");


typedef union {
    struct {
        unsigned SEG8COM1 :1;
        unsigned SEG9COM1 :1;
        unsigned SEG10COM1 :1;
        unsigned SEG11COM1 :1;
        unsigned SEG12COM1 :1;
        unsigned SEG13COM1 :1;
        unsigned SEG14COM1 :1;
        unsigned SEG15COM1 :1;
    };
} LCDDATA4bits_t;
extern volatile LCDDATA4bits_t LCDDATA4bits __attribute__((address(0x7A4)));
# 10130 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA5 __attribute__((address(0x7A5)));

__asm("LCDDATA5 equ 07A5h");


typedef union {
    struct {
        unsigned SEG16COM1 :1;
        unsigned SEG17COM1 :1;
        unsigned SEG18COM1 :1;
        unsigned SEG19COM1 :1;
        unsigned SEG20COM1 :1;
        unsigned SEG21COM1 :1;
        unsigned SEG22COM1 :1;
        unsigned SEG23COM1 :1;
    };
} LCDDATA5bits_t;
extern volatile LCDDATA5bits_t LCDDATA5bits __attribute__((address(0x7A5)));
# 10192 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA6 __attribute__((address(0x7A6)));

__asm("LCDDATA6 equ 07A6h");


typedef union {
    struct {
        unsigned SEG0COM2 :1;
        unsigned SEG1COM2 :1;
        unsigned SEG2COM2 :1;
        unsigned SEG3COM2 :1;
        unsigned SEG4COM2 :1;
        unsigned SEG5COM2 :1;
        unsigned SEG6COM2 :1;
        unsigned SEG7COM2 :1;
    };
} LCDDATA6bits_t;
extern volatile LCDDATA6bits_t LCDDATA6bits __attribute__((address(0x7A6)));
# 10254 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA7 __attribute__((address(0x7A7)));

__asm("LCDDATA7 equ 07A7h");


typedef union {
    struct {
        unsigned SEG8COM2 :1;
        unsigned SEG9COM2 :1;
        unsigned SEG10COM2 :1;
        unsigned SEG11COM2 :1;
        unsigned SEG12COM2 :1;
        unsigned SEG13COM2 :1;
        unsigned SEG14COM2 :1;
        unsigned SEG15COM2 :1;
    };
} LCDDATA7bits_t;
extern volatile LCDDATA7bits_t LCDDATA7bits __attribute__((address(0x7A7)));
# 10316 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA8 __attribute__((address(0x7A8)));

__asm("LCDDATA8 equ 07A8h");


typedef union {
    struct {
        unsigned SEG16COM2 :1;
        unsigned SEG17COM2 :1;
        unsigned SEG18COM2 :1;
        unsigned SEG19COM2 :1;
        unsigned SEG20COM2 :1;
        unsigned SEG21COM2 :1;
        unsigned SEG22COM2 :1;
        unsigned SEG23COM2 :1;
    };
} LCDDATA8bits_t;
extern volatile LCDDATA8bits_t LCDDATA8bits __attribute__((address(0x7A8)));
# 10378 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA9 __attribute__((address(0x7A9)));

__asm("LCDDATA9 equ 07A9h");


typedef union {
    struct {
        unsigned SEG0COM3 :1;
        unsigned SEG1COM3 :1;
        unsigned SEG2COM3 :1;
        unsigned SEG3COM3 :1;
        unsigned SEG4COM3 :1;
        unsigned SEG5COM3 :1;
        unsigned SEG6COM3 :1;
        unsigned SEG7COM3 :1;
    };
} LCDDATA9bits_t;
extern volatile LCDDATA9bits_t LCDDATA9bits __attribute__((address(0x7A9)));
# 10440 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA10 __attribute__((address(0x7AA)));

__asm("LCDDATA10 equ 07AAh");


typedef union {
    struct {
        unsigned SEG8COM3 :1;
        unsigned SEG9COM3 :1;
        unsigned SEG10COM3 :1;
        unsigned SEG11COM3 :1;
        unsigned SEG12COM3 :1;
        unsigned SEG13COM3 :1;
        unsigned SEG14COM3 :1;
        unsigned SEG15COM3 :1;
    };
} LCDDATA10bits_t;
extern volatile LCDDATA10bits_t LCDDATA10bits __attribute__((address(0x7AA)));
# 10502 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA11 __attribute__((address(0x7AB)));

__asm("LCDDATA11 equ 07ABh");


typedef union {
    struct {
        unsigned SEG16COM3 :1;
        unsigned SEG17COM3 :1;
        unsigned SEG18COM3 :1;
        unsigned SEG19COM3 :1;
        unsigned SEG20COM3 :1;
        unsigned SEG21COM3 :1;
        unsigned SEG22COM3 :1;
        unsigned SEG23COM3 :1;
    };
} LCDDATA11bits_t;
extern volatile LCDDATA11bits_t LCDDATA11bits __attribute__((address(0x7AB)));
# 10564 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA12 __attribute__((address(0x7AC)));

__asm("LCDDATA12 equ 07ACh");


typedef union {
    struct {
        unsigned SEG24COM0 :1;
        unsigned SEG25COM0 :1;
        unsigned SEG26COM0 :1;
        unsigned SEG27COM0 :1;
        unsigned SEG28COM0 :1;
        unsigned SEG29COM0 :1;
        unsigned SEG30COM0 :1;
        unsigned SEG31COM0 :1;
    };
} LCDDATA12bits_t;
extern volatile LCDDATA12bits_t LCDDATA12bits __attribute__((address(0x7AC)));
# 10626 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA13 __attribute__((address(0x7AD)));

__asm("LCDDATA13 equ 07ADh");


typedef union {
    struct {
        unsigned SEG32COM0 :1;
        unsigned SEG33COM0 :1;
        unsigned SEG34COM0 :1;
        unsigned SEG35COM0 :1;
        unsigned SEG36COM0 :1;
        unsigned SEG37COM0 :1;
        unsigned SEG38COM0 :1;
        unsigned SEG39COM0 :1;
    };
} LCDDATA13bits_t;
extern volatile LCDDATA13bits_t LCDDATA13bits __attribute__((address(0x7AD)));
# 10688 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA14 __attribute__((address(0x7AE)));

__asm("LCDDATA14 equ 07AEh");


typedef union {
    struct {
        unsigned SEG40COM0 :1;
        unsigned SEG41COM0 :1;
        unsigned SEG42COM0 :1;
        unsigned SEG43COM0 :1;
        unsigned SEG44COM0 :1;
        unsigned SEG45COM0 :1;
    };
} LCDDATA14bits_t;
extern volatile LCDDATA14bits_t LCDDATA14bits __attribute__((address(0x7AE)));
# 10738 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA15 __attribute__((address(0x7AF)));

__asm("LCDDATA15 equ 07AFh");


typedef union {
    struct {
        unsigned SEG24COM1 :1;
        unsigned SEG25COM1 :1;
        unsigned SEG26COM1 :1;
        unsigned SEG27COM1 :1;
        unsigned SEG28COM1 :1;
        unsigned SEG29COM1 :1;
        unsigned SEG30COM1 :1;
        unsigned SEG31COM1 :1;
    };
} LCDDATA15bits_t;
extern volatile LCDDATA15bits_t LCDDATA15bits __attribute__((address(0x7AF)));
# 10800 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA16 __attribute__((address(0x7B0)));

__asm("LCDDATA16 equ 07B0h");


typedef union {
    struct {
        unsigned SEG32COM1 :1;
        unsigned SEG33COM1 :1;
        unsigned SEG34COM1 :1;
        unsigned SEG35COM1 :1;
        unsigned SEG36COM1 :1;
        unsigned SEG37COM1 :1;
        unsigned SEG38COM1 :1;
        unsigned SEG39COM1 :1;
    };
} LCDDATA16bits_t;
extern volatile LCDDATA16bits_t LCDDATA16bits __attribute__((address(0x7B0)));
# 10862 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA17 __attribute__((address(0x7B1)));

__asm("LCDDATA17 equ 07B1h");


typedef union {
    struct {
        unsigned SEG40COM1 :1;
        unsigned SEG41COM1 :1;
        unsigned SEG42COM1 :1;
        unsigned SEG43COM1 :1;
        unsigned SEG44COM1 :1;
        unsigned SEG45COM1 :1;
    };
} LCDDATA17bits_t;
extern volatile LCDDATA17bits_t LCDDATA17bits __attribute__((address(0x7B1)));
# 10912 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA18 __attribute__((address(0x7B2)));

__asm("LCDDATA18 equ 07B2h");


typedef union {
    struct {
        unsigned SEG24COM2 :1;
        unsigned SEG25COM2 :1;
        unsigned SEG26COM2 :1;
        unsigned SEG27COM2 :1;
        unsigned SEG28COM2 :1;
        unsigned SEG29COM2 :1;
        unsigned SEG30COM2 :1;
        unsigned SEG31COM2 :1;
    };
} LCDDATA18bits_t;
extern volatile LCDDATA18bits_t LCDDATA18bits __attribute__((address(0x7B2)));
# 10974 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA19 __attribute__((address(0x7B3)));

__asm("LCDDATA19 equ 07B3h");


typedef union {
    struct {
        unsigned SEG32COM2 :1;
        unsigned SEG33COM2 :1;
        unsigned SEG34COM2 :1;
        unsigned SEG35COM2 :1;
        unsigned SEG36COM2 :1;
        unsigned SEG37COM2 :1;
        unsigned SEG38COM2 :1;
        unsigned SEG39COM2 :1;
    };
} LCDDATA19bits_t;
extern volatile LCDDATA19bits_t LCDDATA19bits __attribute__((address(0x7B3)));
# 11036 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA20 __attribute__((address(0x7B4)));

__asm("LCDDATA20 equ 07B4h");


typedef union {
    struct {
        unsigned SEG40COM2 :1;
        unsigned SEG41COM2 :1;
        unsigned SEG42COM2 :1;
        unsigned SEG43COM2 :1;
        unsigned SEG44COM2 :1;
        unsigned SEG45COM2 :1;
    };
} LCDDATA20bits_t;
extern volatile LCDDATA20bits_t LCDDATA20bits __attribute__((address(0x7B4)));
# 11086 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA21 __attribute__((address(0x7B5)));

__asm("LCDDATA21 equ 07B5h");


typedef union {
    struct {
        unsigned SEG24COM3 :1;
        unsigned SEG25COM3 :1;
        unsigned SEG26COM3 :1;
        unsigned SEG27COM3 :1;
        unsigned SEG28COM3 :1;
        unsigned SEG29COM3 :1;
        unsigned SEG30COM3 :1;
        unsigned SEG31COM3 :1;
    };
} LCDDATA21bits_t;
extern volatile LCDDATA21bits_t LCDDATA21bits __attribute__((address(0x7B5)));
# 11148 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA22 __attribute__((address(0x7B6)));

__asm("LCDDATA22 equ 07B6h");


typedef union {
    struct {
        unsigned SEG32COM3 :1;
        unsigned SEG33COM3 :1;
        unsigned SEG34COM3 :1;
        unsigned SEG35COM3 :1;
        unsigned SEG36COM3 :1;
        unsigned SEG37COM3 :1;
        unsigned SEG38COM3 :1;
        unsigned SEG39COM3 :1;
    };
} LCDDATA22bits_t;
extern volatile LCDDATA22bits_t LCDDATA22bits __attribute__((address(0x7B6)));
# 11210 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char LCDDATA23 __attribute__((address(0x7B7)));

__asm("LCDDATA23 equ 07B7h");


typedef union {
    struct {
        unsigned SEG40COM3 :1;
        unsigned SEG41COM3 :1;
        unsigned SEG42COM3 :1;
        unsigned SEG43COM3 :1;
        unsigned SEG44COM3 :1;
        unsigned SEG45COM3 :1;
    };
} LCDDATA23bits_t;
extern volatile LCDDATA23bits_t LCDDATA23bits __attribute__((address(0x7B7)));
# 11260 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char STATUS_SHAD __attribute__((address(0xFE4)));

__asm("STATUS_SHAD equ 0FE4h");


typedef union {
    struct {
        unsigned C_SHAD :1;
        unsigned DC_SHAD :1;
        unsigned Z_SHAD :1;
    };
} STATUS_SHADbits_t;
extern volatile STATUS_SHADbits_t STATUS_SHADbits __attribute__((address(0xFE4)));
# 11292 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char WREG_SHAD __attribute__((address(0xFE5)));

__asm("WREG_SHAD equ 0FE5h");


typedef union {
    struct {
        unsigned WREG_SHAD :8;
    };
} WREG_SHADbits_t;
extern volatile WREG_SHADbits_t WREG_SHADbits __attribute__((address(0xFE5)));
# 11312 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char BSR_SHAD __attribute__((address(0xFE6)));

__asm("BSR_SHAD equ 0FE6h");


typedef union {
    struct {
        unsigned BSR_SHAD :5;
    };
} BSR_SHADbits_t;
extern volatile BSR_SHADbits_t BSR_SHADbits __attribute__((address(0xFE6)));
# 11332 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char PCLATH_SHAD __attribute__((address(0xFE7)));

__asm("PCLATH_SHAD equ 0FE7h");


typedef union {
    struct {
        unsigned PCLATH_SHAD :7;
    };
} PCLATH_SHADbits_t;
extern volatile PCLATH_SHADbits_t PCLATH_SHADbits __attribute__((address(0xFE7)));
# 11352 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR0L_SHAD __attribute__((address(0xFE8)));

__asm("FSR0L_SHAD equ 0FE8h");


typedef union {
    struct {
        unsigned FSR0L_SHAD :8;
    };
} FSR0L_SHADbits_t;
extern volatile FSR0L_SHADbits_t FSR0L_SHADbits __attribute__((address(0xFE8)));
# 11372 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR0H_SHAD __attribute__((address(0xFE9)));

__asm("FSR0H_SHAD equ 0FE9h");


typedef union {
    struct {
        unsigned FSR0H_SHAD :8;
    };
} FSR0H_SHADbits_t;
extern volatile FSR0H_SHADbits_t FSR0H_SHADbits __attribute__((address(0xFE9)));
# 11392 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR1L_SHAD __attribute__((address(0xFEA)));

__asm("FSR1L_SHAD equ 0FEAh");


typedef union {
    struct {
        unsigned FSR1L_SHAD :8;
    };
} FSR1L_SHADbits_t;
extern volatile FSR1L_SHADbits_t FSR1L_SHADbits __attribute__((address(0xFEA)));
# 11412 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char FSR1H_SHAD __attribute__((address(0xFEB)));

__asm("FSR1H_SHAD equ 0FEBh");


typedef union {
    struct {
        unsigned FSR1H_SHAD :8;
    };
} FSR1H_SHADbits_t;
extern volatile FSR1H_SHADbits_t FSR1H_SHADbits __attribute__((address(0xFEB)));
# 11432 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char STKPTR __attribute__((address(0xFED)));

__asm("STKPTR equ 0FEDh");


typedef union {
    struct {
        unsigned STKPTR :5;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0xFED)));
# 11452 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TOSL __attribute__((address(0xFEE)));

__asm("TOSL equ 0FEEh");


typedef union {
    struct {
        unsigned TOSL :8;
    };
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __attribute__((address(0xFEE)));
# 11472 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile unsigned char TOSH __attribute__((address(0xFEF)));

__asm("TOSH equ 0FEFh");


typedef union {
    struct {
        unsigned TOSH :7;
    };
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __attribute__((address(0xFEF)));
# 11498 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\proc\\pic16f1947.h" 3
extern volatile __bit ADCS0 __attribute__((address(0x4F4)));


extern volatile __bit ADCS1 __attribute__((address(0x4F5)));


extern volatile __bit ADCS2 __attribute__((address(0x4F6)));


extern volatile __bit ADFM __attribute__((address(0x4F7)));


extern volatile __bit ADFVR0 __attribute__((address(0x8B8)));


extern volatile __bit ADFVR1 __attribute__((address(0x8B9)));


extern volatile __bit ADGO __attribute__((address(0x4E9)));


extern volatile __bit ADIE __attribute__((address(0x48E)));


extern volatile __bit ADIF __attribute__((address(0x8E)));


extern volatile __bit ADNREF __attribute__((address(0x4F2)));


extern volatile __bit ADON __attribute__((address(0x4E8)));


extern volatile __bit ADPREF0 __attribute__((address(0x4F0)));


extern volatile __bit ADPREF1 __attribute__((address(0x4F1)));


extern volatile __bit AN0 __attribute__((address(0x60)));


extern volatile __bit AN1 __attribute__((address(0x61)));


extern volatile __bit AN10 __attribute__((address(0x1465)));


extern volatile __bit AN11 __attribute__((address(0x1466)));


extern volatile __bit AN12 __attribute__((address(0x146C)));


extern volatile __bit AN13 __attribute__((address(0x146B)));


extern volatile __bit AN14 __attribute__((address(0x146A)));


extern volatile __bit AN15 __attribute__((address(0x1469)));


extern volatile __bit AN16 __attribute__((address(0x1460)));


extern volatile __bit AN2 __attribute__((address(0x62)));


extern volatile __bit AN3 __attribute__((address(0x63)));


extern volatile __bit AN4 __attribute__((address(0x65)));


extern volatile __bit AN5 __attribute__((address(0x1467)));


extern volatile __bit AN6 __attribute__((address(0x1461)));


extern volatile __bit AN7 __attribute__((address(0x1462)));


extern volatile __bit AN8 __attribute__((address(0x1463)));


extern volatile __bit AN9 __attribute__((address(0x1464)));


extern volatile __bit ANSA0 __attribute__((address(0xC60)));


extern volatile __bit ANSA1 __attribute__((address(0xC61)));


extern volatile __bit ANSA2 __attribute__((address(0xC62)));


extern volatile __bit ANSA3 __attribute__((address(0xC63)));


extern volatile __bit ANSA5 __attribute__((address(0xC65)));


extern volatile __bit ANSE0 __attribute__((address(0xC80)));


extern volatile __bit ANSE1 __attribute__((address(0xC81)));


extern volatile __bit ANSE2 __attribute__((address(0xC82)));


extern volatile __bit ANSF0 __attribute__((address(0x2060)));


extern volatile __bit ANSF1 __attribute__((address(0x2061)));


extern volatile __bit ANSF2 __attribute__((address(0x2062)));


extern volatile __bit ANSF3 __attribute__((address(0x2063)));


extern volatile __bit ANSF4 __attribute__((address(0x2064)));


extern volatile __bit ANSF5 __attribute__((address(0x2065)));


extern volatile __bit ANSF6 __attribute__((address(0x2066)));


extern volatile __bit ANSF7 __attribute__((address(0x2067)));


extern volatile __bit ANSG1 __attribute__((address(0x2069)));


extern volatile __bit ANSG2 __attribute__((address(0x206A)));


extern volatile __bit ANSG3 __attribute__((address(0x206B)));


extern volatile __bit ANSG4 __attribute__((address(0x206C)));


extern volatile __bit BCL2IE __attribute__((address(0x4A1)));


extern volatile __bit BCL2IF __attribute__((address(0xA1)));


extern volatile __bit BCLIE __attribute__((address(0x493)));


extern volatile __bit BCLIF __attribute__((address(0x93)));


extern volatile __bit BIASMD __attribute__((address(0x3C96)));


extern volatile __bit BORRDY __attribute__((address(0x8B0)));


extern volatile __bit BSR0 __attribute__((address(0x40)));


extern volatile __bit BSR1 __attribute__((address(0x41)));


extern volatile __bit BSR2 __attribute__((address(0x42)));


extern volatile __bit BSR3 __attribute__((address(0x43)));


extern volatile __bit BSR4 __attribute__((address(0x44)));


extern volatile __bit C1HYS __attribute__((address(0x889)));


extern volatile __bit C1IE __attribute__((address(0x495)));


extern volatile __bit C1IF __attribute__((address(0x95)));


extern volatile __bit C1IN0N __attribute__((address(0x1460)));


extern volatile __bit C1IN1N __attribute__((address(0x1465)));


extern volatile __bit C1IN2N __attribute__((address(0x1463)));


extern volatile __bit C1IN3N __attribute__((address(0x1467)));


extern volatile __bit C1INP __attribute__((address(0x1466)));


extern volatile __bit C1INTN __attribute__((address(0x896)));


extern volatile __bit C1INTP __attribute__((address(0x897)));


extern volatile __bit C1NCH0 __attribute__((address(0x890)));


extern volatile __bit C1NCH1 __attribute__((address(0x891)));


extern volatile __bit C1OE __attribute__((address(0x88D)));


extern volatile __bit C1ON __attribute__((address(0x88F)));


extern volatile __bit C1PCH0 __attribute__((address(0x894)));


extern volatile __bit C1PCH1 __attribute__((address(0x895)));


extern volatile __bit C1POL __attribute__((address(0x88C)));


extern volatile __bit C1SP __attribute__((address(0x88A)));


extern volatile __bit C1SYNC __attribute__((address(0x888)));


extern volatile __bit C1TSEL0 __attribute__((address(0x14F0)));


extern volatile __bit C1TSEL1 __attribute__((address(0x14F1)));


extern volatile __bit C2HYS __attribute__((address(0x899)));


extern volatile __bit C2IE __attribute__((address(0x496)));


extern volatile __bit C2IF __attribute__((address(0x96)));


extern volatile __bit C2IN0N __attribute__((address(0x1460)));


extern volatile __bit C2IN1N __attribute__((address(0x1465)));


extern volatile __bit C2IN2N __attribute__((address(0x1463)));


extern volatile __bit C2IN3N __attribute__((address(0x1467)));


extern volatile __bit C2INP __attribute__((address(0x1464)));


extern volatile __bit C2INTN __attribute__((address(0x8A6)));


extern volatile __bit C2INTP __attribute__((address(0x8A7)));


extern volatile __bit C2NCH0 __attribute__((address(0x8A0)));


extern volatile __bit C2NCH1 __attribute__((address(0x8A1)));


extern volatile __bit C2OE __attribute__((address(0x89D)));


extern volatile __bit C2ON __attribute__((address(0x89F)));


extern volatile __bit C2PCH0 __attribute__((address(0x8A4)));


extern volatile __bit C2PCH1 __attribute__((address(0x8A5)));


extern volatile __bit C2POL __attribute__((address(0x89C)));


extern volatile __bit C2SP __attribute__((address(0x89A)));


extern volatile __bit C2SYNC __attribute__((address(0x898)));


extern volatile __bit C2TSEL0 __attribute__((address(0x14F2)));


extern volatile __bit C2TSEL1 __attribute__((address(0x14F3)));


extern volatile __bit C3HYS __attribute__((address(0x8F1)));


extern volatile __bit C3IE __attribute__((address(0x491)));


extern volatile __bit C3IF __attribute__((address(0x91)));


extern volatile __bit C3IN0N __attribute__((address(0x146B)));


extern volatile __bit C3IN1N __attribute__((address(0x146C)));


extern volatile __bit C3IN2N __attribute__((address(0x1463)));


extern volatile __bit C3IN3N __attribute__((address(0x1467)));


extern volatile __bit C3INP __attribute__((address(0x146A)));


extern volatile __bit C3INTN __attribute__((address(0x8FE)));


extern volatile __bit C3INTP __attribute__((address(0x8FF)));


extern volatile __bit C3NCH0 __attribute__((address(0x8F8)));


extern volatile __bit C3NCH1 __attribute__((address(0x8F9)));


extern volatile __bit C3OE __attribute__((address(0x8F5)));


extern volatile __bit C3ON __attribute__((address(0x8F7)));


extern volatile __bit C3PCH0 __attribute__((address(0x8FC)));


extern volatile __bit C3PCH1 __attribute__((address(0x8FD)));


extern volatile __bit C3POL __attribute__((address(0x8F4)));


extern volatile __bit C3SP __attribute__((address(0x8F2)));


extern volatile __bit C3SYNC __attribute__((address(0x8F0)));


extern volatile __bit C3TSEL0 __attribute__((address(0x14F4)));


extern volatile __bit C3TSEL1 __attribute__((address(0x14F5)));


extern volatile __bit C4TSEL0 __attribute__((address(0x14F6)));


extern volatile __bit C4TSEL1 __attribute__((address(0x14F7)));


extern volatile __bit C5TSEL0 __attribute__((address(0x14F8)));


extern volatile __bit C5TSEL1 __attribute__((address(0x14F9)));


extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit CCP1 __attribute__((address(0x72)));


extern volatile __bit CCP1AS0 __attribute__((address(0x14AC)));


extern volatile __bit CCP1AS1 __attribute__((address(0x14AD)));


extern volatile __bit CCP1AS2 __attribute__((address(0x14AE)));


extern volatile __bit CCP1ASE __attribute__((address(0x14AF)));


extern volatile __bit CCP1IE __attribute__((address(0x48A)));


extern volatile __bit CCP1IF __attribute__((address(0x8A)));


extern volatile __bit CCP1M0 __attribute__((address(0x1498)));


extern volatile __bit CCP1M1 __attribute__((address(0x1499)));


extern volatile __bit CCP1M2 __attribute__((address(0x149A)));


extern volatile __bit CCP1M3 __attribute__((address(0x149B)));


extern volatile __bit CCP2AS0 __attribute__((address(0x14E4)));


extern volatile __bit CCP2AS1 __attribute__((address(0x14E5)));


extern volatile __bit CCP2AS2 __attribute__((address(0x14E6)));


extern volatile __bit CCP2ASE __attribute__((address(0x14E7)));


extern volatile __bit CCP2IE __attribute__((address(0x490)));


extern volatile __bit CCP2IF __attribute__((address(0x90)));


extern volatile __bit CCP2M0 __attribute__((address(0x14D0)));


extern volatile __bit CCP2M1 __attribute__((address(0x14D1)));


extern volatile __bit CCP2M2 __attribute__((address(0x14D2)));


extern volatile __bit CCP2M3 __attribute__((address(0x14D3)));


extern volatile __bit CCP2SEL __attribute__((address(0x8EA)));


extern volatile __bit CCP3 __attribute__((address(0x1468)));


extern volatile __bit CCP3AS0 __attribute__((address(0x18AC)));


extern volatile __bit CCP3AS1 __attribute__((address(0x18AD)));


extern volatile __bit CCP3AS2 __attribute__((address(0x18AE)));


extern volatile __bit CCP3ASE __attribute__((address(0x18AF)));


extern volatile __bit CCP3IE __attribute__((address(0x49C)));


extern volatile __bit CCP3IF __attribute__((address(0x9C)));


extern volatile __bit CCP3M0 __attribute__((address(0x1898)));


extern volatile __bit CCP3M1 __attribute__((address(0x1899)));


extern volatile __bit CCP3M2 __attribute__((address(0x189A)));


extern volatile __bit CCP3M3 __attribute__((address(0x189B)));


extern volatile __bit CCP4 __attribute__((address(0x146B)));


extern volatile __bit CCP4IE __attribute__((address(0x49D)));


extern volatile __bit CCP4IF __attribute__((address(0x9D)));


extern volatile __bit CCP4M0 __attribute__((address(0x18D0)));


extern volatile __bit CCP4M1 __attribute__((address(0x18D1)));


extern volatile __bit CCP4M2 __attribute__((address(0x18D2)));


extern volatile __bit CCP4M3 __attribute__((address(0x18D3)));


extern volatile __bit CCP5 __attribute__((address(0x146C)));


extern volatile __bit CCP5IE __attribute__((address(0x49E)));


extern volatile __bit CCP5IF __attribute__((address(0x9E)));


extern volatile __bit CCP5M0 __attribute__((address(0x18F0)));


extern volatile __bit CCP5M1 __attribute__((address(0x18F1)));


extern volatile __bit CCP5M2 __attribute__((address(0x18F2)));


extern volatile __bit CCP5M3 __attribute__((address(0x18F3)));


extern volatile __bit CDAFVR0 __attribute__((address(0x8BA)));


extern volatile __bit CDAFVR1 __attribute__((address(0x8BB)));


extern volatile __bit CFGS __attribute__((address(0xCAE)));


extern volatile __bit CHS0 __attribute__((address(0x4EA)));


extern volatile __bit CHS1 __attribute__((address(0x4EB)));


extern volatile __bit CHS2 __attribute__((address(0x4EC)));


extern volatile __bit CHS3 __attribute__((address(0x4ED)));


extern volatile __bit CHS4 __attribute__((address(0x4EE)));


extern volatile __bit CK1 __attribute__((address(0x76)));


extern volatile __bit CK2 __attribute__((address(0x1469)));


extern volatile __bit CLKIN __attribute__((address(0x67)));


extern volatile __bit CLKOUT __attribute__((address(0x66)));


extern volatile __bit COM0 __attribute__((address(0x83)));


extern volatile __bit COM1 __attribute__((address(0x84)));


extern volatile __bit COM2 __attribute__((address(0x85)));


extern volatile __bit COM3 __attribute__((address(0x86)));


extern volatile __bit CPS0 __attribute__((address(0x60)));


extern volatile __bit CPS1 __attribute__((address(0x61)));


extern volatile __bit CPS10 __attribute__((address(0x1465)));


extern volatile __bit CPS11 __attribute__((address(0x1466)));


extern volatile __bit CPS12 __attribute__((address(0x146C)));


extern volatile __bit CPS13 __attribute__((address(0x146B)));


extern volatile __bit CPS14 __attribute__((address(0x146A)));


extern volatile __bit CPS15 __attribute__((address(0x1469)));


extern volatile __bit CPS16 __attribute__((address(0x1460)));


extern volatile __bit CPS2 __attribute__((address(0x62)));


extern volatile __bit CPS3 __attribute__((address(0x63)));


extern volatile __bit CPS4 __attribute__((address(0x65)));


extern volatile __bit CPS5 __attribute__((address(0x1467)));


extern volatile __bit CPS6 __attribute__((address(0x1461)));


extern volatile __bit CPS7 __attribute__((address(0x1462)));


extern volatile __bit CPS8 __attribute__((address(0x1463)));


extern volatile __bit CPS9 __attribute__((address(0x1464)));


extern volatile __bit CPSCH0 __attribute__((address(0xF8)));


extern volatile __bit CPSCH1 __attribute__((address(0xF9)));


extern volatile __bit CPSCH2 __attribute__((address(0xFA)));


extern volatile __bit CPSCH3 __attribute__((address(0xFB)));


extern volatile __bit CPSCH4 __attribute__((address(0xFC)));


extern volatile __bit CPSON __attribute__((address(0xF7)));


extern volatile __bit CPSOUT __attribute__((address(0xF1)));


extern volatile __bit CPSRM __attribute__((address(0xF6)));


extern volatile __bit CPSRNG0 __attribute__((address(0xF2)));


extern volatile __bit CPSRNG1 __attribute__((address(0xF3)));


extern volatile __bit CS0 __attribute__((address(0x3C8A)));


extern volatile __bit CS1 __attribute__((address(0x3C8B)));


extern volatile __bit C_SHAD __attribute__((address(0x7F20)));


extern volatile __bit DACEN __attribute__((address(0x8C7)));


extern volatile __bit DACLPS __attribute__((address(0x8C6)));


extern volatile __bit DACNSS __attribute__((address(0x8C0)));


extern volatile __bit DACOE __attribute__((address(0x8C5)));


extern volatile __bit DACOUT __attribute__((address(0x1465)));


extern volatile __bit DACPSS0 __attribute__((address(0x8C2)));


extern volatile __bit DACPSS1 __attribute__((address(0x8C3)));


extern volatile __bit DACR0 __attribute__((address(0x8C8)));


extern volatile __bit DACR1 __attribute__((address(0x8C9)));


extern volatile __bit DACR2 __attribute__((address(0x8CA)));


extern volatile __bit DACR3 __attribute__((address(0x8CB)));


extern volatile __bit DACR4 __attribute__((address(0x8CC)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit DC1B0 __attribute__((address(0x149C)));


extern volatile __bit DC1B1 __attribute__((address(0x149D)));


extern volatile __bit DC2B0 __attribute__((address(0x14D4)));


extern volatile __bit DC2B1 __attribute__((address(0x14D5)));


extern volatile __bit DC3B0 __attribute__((address(0x189C)));


extern volatile __bit DC3B1 __attribute__((address(0x189D)));


extern volatile __bit DC4B0 __attribute__((address(0x18D4)));


extern volatile __bit DC4B1 __attribute__((address(0x18D5)));


extern volatile __bit DC5B0 __attribute__((address(0x18F4)));


extern volatile __bit DC5B1 __attribute__((address(0x18F5)));


extern volatile __bit DC_SHAD __attribute__((address(0x7F21)));


extern volatile __bit DT1 __attribute__((address(0x77)));


extern volatile __bit DT2 __attribute__((address(0x146A)));


extern volatile __bit EEIE __attribute__((address(0x494)));


extern volatile __bit EEIF __attribute__((address(0x94)));


extern volatile __bit EEPGD __attribute__((address(0xCAF)));


extern volatile __bit FLT0 __attribute__((address(0x68)));


extern volatile __bit FREE __attribute__((address(0xCAC)));


extern volatile __bit FVREN __attribute__((address(0x8BF)));


extern volatile __bit FVRRDY __attribute__((address(0x8BE)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit GO __attribute__((address(0x4E9)));


extern volatile __bit GO_nDONE __attribute__((address(0x4E9)));


extern volatile __bit HFIOFL __attribute__((address(0x4D3)));


extern volatile __bit HFIOFR __attribute__((address(0x4D4)));


extern volatile __bit HFIOFS __attribute__((address(0x4D0)));


extern volatile __bit INTE __attribute__((address(0x5C)));


extern volatile __bit INTEDG __attribute__((address(0x4AE)));


extern volatile __bit INTF __attribute__((address(0x59)));


extern volatile __bit IOCBF0 __attribute__((address(0x1CB0)));


extern volatile __bit IOCBF1 __attribute__((address(0x1CB1)));


extern volatile __bit IOCBF2 __attribute__((address(0x1CB2)));


extern volatile __bit IOCBF3 __attribute__((address(0x1CB3)));


extern volatile __bit IOCBF4 __attribute__((address(0x1CB4)));


extern volatile __bit IOCBF5 __attribute__((address(0x1CB5)));


extern volatile __bit IOCBF6 __attribute__((address(0x1CB6)));


extern volatile __bit IOCBF7 __attribute__((address(0x1CB7)));


extern volatile __bit IOCBN0 __attribute__((address(0x1CA8)));


extern volatile __bit IOCBN1 __attribute__((address(0x1CA9)));


extern volatile __bit IOCBN2 __attribute__((address(0x1CAA)));


extern volatile __bit IOCBN3 __attribute__((address(0x1CAB)));


extern volatile __bit IOCBN4 __attribute__((address(0x1CAC)));


extern volatile __bit IOCBN5 __attribute__((address(0x1CAD)));


extern volatile __bit IOCBN6 __attribute__((address(0x1CAE)));


extern volatile __bit IOCBN7 __attribute__((address(0x1CAF)));


extern volatile __bit IOCBP0 __attribute__((address(0x1CA0)));


extern volatile __bit IOCBP1 __attribute__((address(0x1CA1)));


extern volatile __bit IOCBP2 __attribute__((address(0x1CA2)));


extern volatile __bit IOCBP3 __attribute__((address(0x1CA3)));


extern volatile __bit IOCBP4 __attribute__((address(0x1CA4)));


extern volatile __bit IOCBP5 __attribute__((address(0x1CA5)));


extern volatile __bit IOCBP6 __attribute__((address(0x1CA6)));


extern volatile __bit IOCBP7 __attribute__((address(0x1CA7)));


extern volatile __bit IOCIE __attribute__((address(0x5B)));


extern volatile __bit IOCIF __attribute__((address(0x58)));


extern volatile __bit IRCF0 __attribute__((address(0x4CB)));


extern volatile __bit IRCF1 __attribute__((address(0x4CC)));


extern volatile __bit IRCF2 __attribute__((address(0x4CD)));


extern volatile __bit IRCF3 __attribute__((address(0x4CE)));


extern volatile __bit LATA0 __attribute__((address(0x860)));


extern volatile __bit LATA1 __attribute__((address(0x861)));


extern volatile __bit LATA2 __attribute__((address(0x862)));


extern volatile __bit LATA3 __attribute__((address(0x863)));


extern volatile __bit LATA4 __attribute__((address(0x864)));


extern volatile __bit LATA5 __attribute__((address(0x865)));


extern volatile __bit LATA6 __attribute__((address(0x866)));


extern volatile __bit LATA7 __attribute__((address(0x867)));


extern volatile __bit LATB0 __attribute__((address(0x868)));


extern volatile __bit LATB1 __attribute__((address(0x869)));


extern volatile __bit LATB2 __attribute__((address(0x86A)));


extern volatile __bit LATB3 __attribute__((address(0x86B)));


extern volatile __bit LATB4 __attribute__((address(0x86C)));


extern volatile __bit LATB5 __attribute__((address(0x86D)));


extern volatile __bit LATB6 __attribute__((address(0x86E)));


extern volatile __bit LATB7 __attribute__((address(0x86F)));


extern volatile __bit LATC0 __attribute__((address(0x870)));


extern volatile __bit LATC1 __attribute__((address(0x871)));


extern volatile __bit LATC2 __attribute__((address(0x872)));


extern volatile __bit LATC3 __attribute__((address(0x873)));


extern volatile __bit LATC4 __attribute__((address(0x874)));


extern volatile __bit LATC5 __attribute__((address(0x875)));


extern volatile __bit LATC6 __attribute__((address(0x876)));


extern volatile __bit LATC7 __attribute__((address(0x877)));


extern volatile __bit LATD0 __attribute__((address(0x878)));


extern volatile __bit LATD1 __attribute__((address(0x879)));


extern volatile __bit LATD2 __attribute__((address(0x87A)));


extern volatile __bit LATD3 __attribute__((address(0x87B)));


extern volatile __bit LATD4 __attribute__((address(0x87C)));


extern volatile __bit LATD5 __attribute__((address(0x87D)));


extern volatile __bit LATD6 __attribute__((address(0x87E)));


extern volatile __bit LATD7 __attribute__((address(0x87F)));


extern volatile __bit LATE0 __attribute__((address(0x880)));


extern volatile __bit LATE1 __attribute__((address(0x881)));


extern volatile __bit LATE2 __attribute__((address(0x882)));


extern volatile __bit LATE3 __attribute__((address(0x883)));


extern volatile __bit LATE4 __attribute__((address(0x884)));


extern volatile __bit LATE5 __attribute__((address(0x885)));


extern volatile __bit LATE6 __attribute__((address(0x886)));


extern volatile __bit LATE7 __attribute__((address(0x887)));


extern volatile __bit LATF0 __attribute__((address(0x1C60)));


extern volatile __bit LATF1 __attribute__((address(0x1C61)));


extern volatile __bit LATF2 __attribute__((address(0x1C62)));


extern volatile __bit LATF3 __attribute__((address(0x1C63)));


extern volatile __bit LATF4 __attribute__((address(0x1C64)));


extern volatile __bit LATF5 __attribute__((address(0x1C65)));


extern volatile __bit LATF6 __attribute__((address(0x1C66)));


extern volatile __bit LATF7 __attribute__((address(0x1C67)));


extern volatile __bit LATG0 __attribute__((address(0x1C68)));


extern volatile __bit LATG1 __attribute__((address(0x1C69)));


extern volatile __bit LATG2 __attribute__((address(0x1C6A)));


extern volatile __bit LATG3 __attribute__((address(0x1C6B)));


extern volatile __bit LATG4 __attribute__((address(0x1C6C)));


extern volatile __bit LATG5 __attribute__((address(0x1C6D)));


extern volatile __bit LCDA __attribute__((address(0x3C95)));


extern volatile __bit LCDCST0 __attribute__((address(0x3CA0)));


extern volatile __bit LCDCST1 __attribute__((address(0x3CA1)));


extern volatile __bit LCDCST2 __attribute__((address(0x3CA2)));


extern volatile __bit LCDEN __attribute__((address(0x3C8F)));


extern volatile __bit LCDIE __attribute__((address(0x492)));


extern volatile __bit LCDIF __attribute__((address(0x92)));


extern volatile __bit LCDIRE __attribute__((address(0x3C9F)));


extern volatile __bit LCDIRI __attribute__((address(0x3C9D)));


extern volatile __bit LCDIRS __attribute__((address(0x3C9E)));


extern volatile __bit LFIOFR __attribute__((address(0x4D1)));


extern volatile __bit LMUX0 __attribute__((address(0x3C88)));


extern volatile __bit LMUX1 __attribute__((address(0x3C89)));


extern volatile __bit LP0 __attribute__((address(0x3C90)));


extern volatile __bit LP1 __attribute__((address(0x3C91)));


extern volatile __bit LP2 __attribute__((address(0x3C92)));


extern volatile __bit LP3 __attribute__((address(0x3C93)));


extern volatile __bit LRLAP0 __attribute__((address(0x3CAE)));


extern volatile __bit LRLAP1 __attribute__((address(0x3CAF)));


extern volatile __bit LRLAT0 __attribute__((address(0x3CA8)));


extern volatile __bit LRLAT1 __attribute__((address(0x3CA9)));


extern volatile __bit LRLAT2 __attribute__((address(0x3CAA)));


extern volatile __bit LRLBP0 __attribute__((address(0x3CAC)));


extern volatile __bit LRLBP1 __attribute__((address(0x3CAD)));


extern volatile __bit LWLO __attribute__((address(0xCAD)));


extern volatile __bit MC1OUT __attribute__((address(0x8A8)));


extern volatile __bit MC2OUT __attribute__((address(0x8A9)));


extern volatile __bit MC3OUT __attribute__((address(0x8AA)));


extern volatile __bit MFIOFR __attribute__((address(0x4D2)));


extern volatile __bit OSC1 __attribute__((address(0x67)));


extern volatile __bit OSC2 __attribute__((address(0x66)));


extern volatile __bit OSFIE __attribute__((address(0x497)));


extern volatile __bit OSFIF __attribute__((address(0x97)));


extern volatile __bit OSTS __attribute__((address(0x4D5)));


extern volatile __bit P1A __attribute__((address(0x72)));


extern volatile __bit P1BSEL __attribute__((address(0x8E8)));


extern volatile __bit P1CSEL __attribute__((address(0x8E9)));


extern volatile __bit P1D __attribute__((address(0x146C)));


extern volatile __bit P1DC0 __attribute__((address(0x14A0)));


extern volatile __bit P1DC1 __attribute__((address(0x14A1)));


extern volatile __bit P1DC2 __attribute__((address(0x14A2)));


extern volatile __bit P1DC3 __attribute__((address(0x14A3)));


extern volatile __bit P1DC4 __attribute__((address(0x14A4)));


extern volatile __bit P1DC5 __attribute__((address(0x14A5)));


extern volatile __bit P1DC6 __attribute__((address(0x14A6)));


extern volatile __bit P1M0 __attribute__((address(0x149E)));


extern volatile __bit P1M1 __attribute__((address(0x149F)));


extern volatile __bit P1RSEN __attribute__((address(0x14A7)));


extern volatile __bit P2BSEL __attribute__((address(0x8EB)));


extern volatile __bit P2CSEL __attribute__((address(0x8EC)));


extern volatile __bit P2DC0 __attribute__((address(0x14D8)));


extern volatile __bit P2DC1 __attribute__((address(0x14D9)));


extern volatile __bit P2DC2 __attribute__((address(0x14DA)));


extern volatile __bit P2DC3 __attribute__((address(0x14DB)));


extern volatile __bit P2DC4 __attribute__((address(0x14DC)));


extern volatile __bit P2DC5 __attribute__((address(0x14DD)));


extern volatile __bit P2DC6 __attribute__((address(0x14DE)));


extern volatile __bit P2DSEL __attribute__((address(0x8ED)));


extern volatile __bit P2M0 __attribute__((address(0x14D6)));


extern volatile __bit P2M1 __attribute__((address(0x14D7)));


extern volatile __bit P2RSEN __attribute__((address(0x14DF)));


extern volatile __bit P3A __attribute__((address(0x1468)));


extern volatile __bit P3BSEL __attribute__((address(0x8EE)));


extern volatile __bit P3CSEL __attribute__((address(0x8EF)));


extern volatile __bit P3D __attribute__((address(0x146B)));


extern volatile __bit P3DC0 __attribute__((address(0x18A0)));


extern volatile __bit P3DC1 __attribute__((address(0x18A1)));


extern volatile __bit P3DC2 __attribute__((address(0x18A2)));


extern volatile __bit P3DC3 __attribute__((address(0x18A3)));


extern volatile __bit P3DC4 __attribute__((address(0x18A4)));


extern volatile __bit P3DC5 __attribute__((address(0x18A5)));


extern volatile __bit P3DC6 __attribute__((address(0x18A6)));


extern volatile __bit P3M0 __attribute__((address(0x189E)));


extern volatile __bit P3M1 __attribute__((address(0x189F)));


extern volatile __bit P3RSEN __attribute__((address(0x18A7)));


extern volatile __bit PEIE __attribute__((address(0x5E)));


extern volatile __bit PLLR __attribute__((address(0x4D6)));


extern volatile __bit PS0 __attribute__((address(0x4A8)));


extern volatile __bit PS1 __attribute__((address(0x4A9)));


extern volatile __bit PS2 __attribute__((address(0x4AA)));


extern volatile __bit PSA __attribute__((address(0x4AB)));


extern volatile __bit PSS1AC0 __attribute__((address(0x14AA)));


extern volatile __bit PSS1AC1 __attribute__((address(0x14AB)));


extern volatile __bit PSS1BD0 __attribute__((address(0x14A8)));


extern volatile __bit PSS1BD1 __attribute__((address(0x14A9)));


extern volatile __bit PSS2AC0 __attribute__((address(0x14E2)));


extern volatile __bit PSS2AC1 __attribute__((address(0x14E3)));


extern volatile __bit PSS2BD0 __attribute__((address(0x14E0)));


extern volatile __bit PSS2BD1 __attribute__((address(0x14E1)));


extern volatile __bit PSS3AC0 __attribute__((address(0x18AA)));


extern volatile __bit PSS3AC1 __attribute__((address(0x18AB)));


extern volatile __bit PSS3BD0 __attribute__((address(0x18A8)));


extern volatile __bit PSS3BD1 __attribute__((address(0x18A9)));


extern volatile __bit RA0 __attribute__((address(0x60)));


extern volatile __bit RA1 __attribute__((address(0x61)));


extern volatile __bit RA2 __attribute__((address(0x62)));


extern volatile __bit RA3 __attribute__((address(0x63)));


extern volatile __bit RA4 __attribute__((address(0x64)));


extern volatile __bit RA5 __attribute__((address(0x65)));


extern volatile __bit RA6 __attribute__((address(0x66)));


extern volatile __bit RA7 __attribute__((address(0x67)));


extern volatile __bit RB0 __attribute__((address(0x68)));


extern volatile __bit RB1 __attribute__((address(0x69)));


extern volatile __bit RB2 __attribute__((address(0x6A)));


extern volatile __bit RB3 __attribute__((address(0x6B)));


extern volatile __bit RB4 __attribute__((address(0x6C)));


extern volatile __bit RB5 __attribute__((address(0x6D)));


extern volatile __bit RB6 __attribute__((address(0x6E)));


extern volatile __bit RB7 __attribute__((address(0x6F)));


extern volatile __bit RC0 __attribute__((address(0x70)));


extern volatile __bit RC1 __attribute__((address(0x71)));


extern volatile __bit RC1IE __attribute__((address(0x48D)));


extern volatile __bit RC1IF __attribute__((address(0x8D)));


extern volatile __bit RC2 __attribute__((address(0x72)));


extern volatile __bit RC2IE __attribute__((address(0x4A5)));


extern volatile __bit RC2IF __attribute__((address(0xA5)));


extern volatile __bit RC3 __attribute__((address(0x73)));


extern volatile __bit RC4 __attribute__((address(0x74)));


extern volatile __bit RC5 __attribute__((address(0x75)));


extern volatile __bit RC6 __attribute__((address(0x76)));


extern volatile __bit RC7 __attribute__((address(0x77)));


extern volatile __bit RCIE __attribute__((address(0x48D)));


extern volatile __bit RCIF __attribute__((address(0x8D)));


extern volatile __bit RD __attribute__((address(0xCA8)));


extern volatile __bit RD0 __attribute__((address(0x78)));


extern volatile __bit RD1 __attribute__((address(0x79)));


extern volatile __bit RD2 __attribute__((address(0x7A)));


extern volatile __bit RD3 __attribute__((address(0x7B)));


extern volatile __bit RD4 __attribute__((address(0x7C)));


extern volatile __bit RD5 __attribute__((address(0x7D)));


extern volatile __bit RD6 __attribute__((address(0x7E)));


extern volatile __bit RD7 __attribute__((address(0x7F)));


extern volatile __bit RE0 __attribute__((address(0x80)));


extern volatile __bit RE1 __attribute__((address(0x81)));


extern volatile __bit RE2 __attribute__((address(0x82)));


extern volatile __bit RE3 __attribute__((address(0x83)));


extern volatile __bit RE4 __attribute__((address(0x84)));


extern volatile __bit RE5 __attribute__((address(0x85)));


extern volatile __bit RE6 __attribute__((address(0x86)));


extern volatile __bit RE7 __attribute__((address(0x87)));


extern volatile __bit RF0 __attribute__((address(0x1460)));


extern volatile __bit RF1 __attribute__((address(0x1461)));


extern volatile __bit RF2 __attribute__((address(0x1462)));


extern volatile __bit RF3 __attribute__((address(0x1463)));


extern volatile __bit RF4 __attribute__((address(0x1464)));


extern volatile __bit RF5 __attribute__((address(0x1465)));


extern volatile __bit RF6 __attribute__((address(0x1466)));


extern volatile __bit RF7 __attribute__((address(0x1467)));


extern volatile __bit RG0 __attribute__((address(0x1468)));


extern volatile __bit RG1 __attribute__((address(0x1469)));


extern volatile __bit RG2 __attribute__((address(0x146A)));


extern volatile __bit RG3 __attribute__((address(0x146B)));


extern volatile __bit RG4 __attribute__((address(0x146C)));


extern volatile __bit RG5 __attribute__((address(0x146D)));


extern volatile __bit RX1 __attribute__((address(0x77)));


extern volatile __bit RX2 __attribute__((address(0x146A)));


extern volatile __bit SBOREN __attribute__((address(0x8B7)));


extern volatile __bit SCK1 __attribute__((address(0x73)));


extern volatile __bit SCK2 __attribute__((address(0x7E)));


extern volatile __bit SCL1 __attribute__((address(0x73)));


extern volatile __bit SCL2 __attribute__((address(0x7E)));


extern volatile __bit SCS0 __attribute__((address(0x4C8)));


extern volatile __bit SCS1 __attribute__((address(0x4C9)));


extern volatile __bit SDA1 __attribute__((address(0x74)));


extern volatile __bit SDA2 __attribute__((address(0x7D)));


extern volatile __bit SDI1 __attribute__((address(0x74)));


extern volatile __bit SDI2 __attribute__((address(0x7D)));


extern volatile __bit SDO1 __attribute__((address(0x75)));


extern volatile __bit SDO2 __attribute__((address(0x7C)));


extern volatile __bit SE0 __attribute__((address(0x3CC0)));


extern volatile __bit SE1 __attribute__((address(0x3CC1)));


extern volatile __bit SE10 __attribute__((address(0x3CCA)));


extern volatile __bit SE11 __attribute__((address(0x3CCB)));


extern volatile __bit SE12 __attribute__((address(0x3CCC)));


extern volatile __bit SE13 __attribute__((address(0x3CCD)));


extern volatile __bit SE14 __attribute__((address(0x3CCE)));


extern volatile __bit SE15 __attribute__((address(0x3CCF)));


extern volatile __bit SE16 __attribute__((address(0x3CD0)));


extern volatile __bit SE17 __attribute__((address(0x3CD1)));


extern volatile __bit SE18 __attribute__((address(0x3CD2)));


extern volatile __bit SE19 __attribute__((address(0x3CD3)));


extern volatile __bit SE2 __attribute__((address(0x3CC2)));


extern volatile __bit SE20 __attribute__((address(0x3CD4)));


extern volatile __bit SE21 __attribute__((address(0x3CD5)));


extern volatile __bit SE22 __attribute__((address(0x3CD6)));


extern volatile __bit SE23 __attribute__((address(0x3CD7)));


extern volatile __bit SE24 __attribute__((address(0x3CD8)));


extern volatile __bit SE25 __attribute__((address(0x3CD9)));


extern volatile __bit SE26 __attribute__((address(0x3CDA)));


extern volatile __bit SE27 __attribute__((address(0x3CDB)));


extern volatile __bit SE28 __attribute__((address(0x3CDC)));


extern volatile __bit SE29 __attribute__((address(0x3CDD)));


extern volatile __bit SE3 __attribute__((address(0x3CC3)));


extern volatile __bit SE30 __attribute__((address(0x3CDE)));


extern volatile __bit SE31 __attribute__((address(0x3CDF)));


extern volatile __bit SE32 __attribute__((address(0x3CE0)));


extern volatile __bit SE33 __attribute__((address(0x3CE1)));


extern volatile __bit SE34 __attribute__((address(0x3CE2)));


extern volatile __bit SE35 __attribute__((address(0x3CE3)));


extern volatile __bit SE36 __attribute__((address(0x3CE4)));


extern volatile __bit SE37 __attribute__((address(0x3CE5)));


extern volatile __bit SE38 __attribute__((address(0x3CE6)));


extern volatile __bit SE39 __attribute__((address(0x3CE7)));


extern volatile __bit SE4 __attribute__((address(0x3CC4)));


extern volatile __bit SE40 __attribute__((address(0x3CE8)));


extern volatile __bit SE41 __attribute__((address(0x3CE9)));


extern volatile __bit SE42 __attribute__((address(0x3CEA)));


extern volatile __bit SE43 __attribute__((address(0x3CEB)));


extern volatile __bit SE44 __attribute__((address(0x3CEC)));


extern volatile __bit SE45 __attribute__((address(0x3CED)));


extern volatile __bit SE5 __attribute__((address(0x3CC5)));


extern volatile __bit SE6 __attribute__((address(0x3CC6)));


extern volatile __bit SE7 __attribute__((address(0x3CC7)));


extern volatile __bit SE8 __attribute__((address(0x3CC8)));


extern volatile __bit SE9 __attribute__((address(0x3CC9)));


extern volatile __bit SEG0 __attribute__((address(0x78)));


extern volatile __bit SEG0COM0 __attribute__((address(0x3D00)));


extern volatile __bit SEG0COM1 __attribute__((address(0x3D18)));


extern volatile __bit SEG0COM2 __attribute__((address(0x3D30)));


extern volatile __bit SEG0COM3 __attribute__((address(0x3D48)));


extern volatile __bit SEG1 __attribute__((address(0x79)));


extern volatile __bit SEG10 __attribute__((address(0x6B)));


extern volatile __bit SEG10COM0 __attribute__((address(0x3D0A)));


extern volatile __bit SEG10COM1 __attribute__((address(0x3D22)));


extern volatile __bit SEG10COM2 __attribute__((address(0x3D3A)));


extern volatile __bit SEG10COM3 __attribute__((address(0x3D52)));


extern volatile __bit SEG11 __attribute__((address(0x6C)));


extern volatile __bit SEG11COM0 __attribute__((address(0x3D0B)));


extern volatile __bit SEG11COM1 __attribute__((address(0x3D23)));


extern volatile __bit SEG11COM2 __attribute__((address(0x3D3B)));


extern volatile __bit SEG11COM3 __attribute__((address(0x3D53)));


extern volatile __bit SEG12 __attribute__((address(0x75)));


extern volatile __bit SEG12COM0 __attribute__((address(0x3D0C)));


extern volatile __bit SEG12COM1 __attribute__((address(0x3D24)));


extern volatile __bit SEG12COM2 __attribute__((address(0x3D3C)));


extern volatile __bit SEG12COM3 __attribute__((address(0x3D54)));


extern volatile __bit SEG13 __attribute__((address(0x72)));


extern volatile __bit SEG13COM0 __attribute__((address(0x3D0D)));


extern volatile __bit SEG13COM1 __attribute__((address(0x3D25)));


extern volatile __bit SEG13COM2 __attribute__((address(0x3D3D)));


extern volatile __bit SEG13COM3 __attribute__((address(0x3D55)));


extern volatile __bit SEG14 __attribute__((address(0x64)));


extern volatile __bit SEG14COM0 __attribute__((address(0x3D0E)));


extern volatile __bit SEG14COM1 __attribute__((address(0x3D26)));


extern volatile __bit SEG14COM2 __attribute__((address(0x3D3E)));


extern volatile __bit SEG14COM3 __attribute__((address(0x3D56)));


extern volatile __bit SEG15 __attribute__((address(0x65)));


extern volatile __bit SEG15COM0 __attribute__((address(0x3D0F)));


extern volatile __bit SEG15COM1 __attribute__((address(0x3D27)));


extern volatile __bit SEG15COM2 __attribute__((address(0x3D3F)));


extern volatile __bit SEG15COM3 __attribute__((address(0x3D57)));


extern volatile __bit SEG16 __attribute__((address(0x74)));


extern volatile __bit SEG16COM0 __attribute__((address(0x3D10)));


extern volatile __bit SEG16COM1 __attribute__((address(0x3D28)));


extern volatile __bit SEG16COM2 __attribute__((address(0x3D40)));


extern volatile __bit SEG16COM3 __attribute__((address(0x3D58)));


extern volatile __bit SEG17 __attribute__((address(0x73)));


extern volatile __bit SEG17COM0 __attribute__((address(0x3D11)));


extern volatile __bit SEG17COM1 __attribute__((address(0x3D29)));


extern volatile __bit SEG17COM2 __attribute__((address(0x3D41)));


extern volatile __bit SEG17COM3 __attribute__((address(0x3D59)));


extern volatile __bit SEG18 __attribute__((address(0x61)));


extern volatile __bit SEG18COM0 __attribute__((address(0x3D12)));


extern volatile __bit SEG18COM1 __attribute__((address(0x3D2A)));


extern volatile __bit SEG18COM2 __attribute__((address(0x3D42)));


extern volatile __bit SEG18COM3 __attribute__((address(0x3D5A)));


extern volatile __bit SEG19 __attribute__((address(0x1461)));


extern volatile __bit SEG19COM0 __attribute__((address(0x3D13)));


extern volatile __bit SEG19COM1 __attribute__((address(0x3D2B)));


extern volatile __bit SEG19COM2 __attribute__((address(0x3D43)));


extern volatile __bit SEG19COM3 __attribute__((address(0x3D5B)));


extern volatile __bit SEG1COM0 __attribute__((address(0x3D01)));


extern volatile __bit SEG1COM1 __attribute__((address(0x3D19)));


extern volatile __bit SEG1COM2 __attribute__((address(0x3D31)));


extern volatile __bit SEG1COM3 __attribute__((address(0x3D49)));


extern volatile __bit SEG2 __attribute__((address(0x7A)));


extern volatile __bit SEG20 __attribute__((address(0x1462)));


extern volatile __bit SEG20COM0 __attribute__((address(0x3D14)));


extern volatile __bit SEG20COM1 __attribute__((address(0x3D2C)));


extern volatile __bit SEG20COM2 __attribute__((address(0x3D44)));


extern volatile __bit SEG20COM3 __attribute__((address(0x3D5C)));


extern volatile __bit SEG21 __attribute__((address(0x1463)));


extern volatile __bit SEG21COM0 __attribute__((address(0x3D15)));


extern volatile __bit SEG21COM1 __attribute__((address(0x3D2D)));


extern volatile __bit SEG21COM2 __attribute__((address(0x3D45)));


extern volatile __bit SEG21COM3 __attribute__((address(0x3D5D)));


extern volatile __bit SEG22 __attribute__((address(0x1464)));


extern volatile __bit SEG22COM0 __attribute__((address(0x3D16)));


extern volatile __bit SEG22COM1 __attribute__((address(0x3D2E)));


extern volatile __bit SEG22COM2 __attribute__((address(0x3D46)));


extern volatile __bit SEG22COM3 __attribute__((address(0x3D5E)));


extern volatile __bit SEG23 __attribute__((address(0x1465)));


extern volatile __bit SEG23COM0 __attribute__((address(0x3D17)));


extern volatile __bit SEG23COM1 __attribute__((address(0x3D2F)));


extern volatile __bit SEG23COM2 __attribute__((address(0x3D47)));


extern volatile __bit SEG23COM3 __attribute__((address(0x3D5F)));


extern volatile __bit SEG24 __attribute__((address(0x1466)));


extern volatile __bit SEG24COM0 __attribute__((address(0x3D60)));


extern volatile __bit SEG24COM1 __attribute__((address(0x3D78)));


extern volatile __bit SEG24COM2 __attribute__((address(0x3D90)));


extern volatile __bit SEG24COM3 __attribute__((address(0x3DA8)));


extern volatile __bit SEG25 __attribute__((address(0x1467)));


extern volatile __bit SEG25COM0 __attribute__((address(0x3D61)));


extern volatile __bit SEG25COM1 __attribute__((address(0x3D79)));


extern volatile __bit SEG25COM2 __attribute__((address(0x3D91)));


extern volatile __bit SEG25COM3 __attribute__((address(0x3DA9)));


extern volatile __bit SEG26 __attribute__((address(0x146C)));


extern volatile __bit SEG26COM0 __attribute__((address(0x3D62)));


extern volatile __bit SEG26COM1 __attribute__((address(0x3D7A)));


extern volatile __bit SEG26COM2 __attribute__((address(0x3D92)));


extern volatile __bit SEG26COM3 __attribute__((address(0x3DAA)));


extern volatile __bit SEG27 __attribute__((address(0x76)));


extern volatile __bit SEG27COM0 __attribute__((address(0x3D63)));


extern volatile __bit SEG27COM1 __attribute__((address(0x3D7B)));


extern volatile __bit SEG27COM2 __attribute__((address(0x3D93)));


extern volatile __bit SEG27COM3 __attribute__((address(0x3DAB)));


extern volatile __bit SEG28 __attribute__((address(0x77)));


extern volatile __bit SEG28COM0 __attribute__((address(0x3D64)));


extern volatile __bit SEG28COM1 __attribute__((address(0x3D7C)));


extern volatile __bit SEG28COM2 __attribute__((address(0x3D94)));


extern volatile __bit SEG28COM3 __attribute__((address(0x3DAC)));


extern volatile __bit SEG29 __attribute__((address(0x6D)));


extern volatile __bit SEG29COM0 __attribute__((address(0x3D65)));


extern volatile __bit SEG29COM1 __attribute__((address(0x3D7D)));


extern volatile __bit SEG29COM2 __attribute__((address(0x3D95)));


extern volatile __bit SEG29COM3 __attribute__((address(0x3DAD)));


extern volatile __bit SEG2COM0 __attribute__((address(0x3D02)));


extern volatile __bit SEG2COM1 __attribute__((address(0x3D1A)));


extern volatile __bit SEG2COM2 __attribute__((address(0x3D32)));


extern volatile __bit SEG2COM3 __attribute__((address(0x3D4A)));


extern volatile __bit SEG3 __attribute__((address(0x7B)));


extern volatile __bit SEG30 __attribute__((address(0x68)));


extern volatile __bit SEG30COM0 __attribute__((address(0x3D66)));


extern volatile __bit SEG30COM1 __attribute__((address(0x3D7E)));


extern volatile __bit SEG30COM2 __attribute__((address(0x3D96)));


extern volatile __bit SEG30COM3 __attribute__((address(0x3DAE)));


extern volatile __bit SEG31 __attribute__((address(0x87)));


extern volatile __bit SEG31COM0 __attribute__((address(0x3D67)));


extern volatile __bit SEG31COM1 __attribute__((address(0x3D7F)));


extern volatile __bit SEG31COM2 __attribute__((address(0x3D97)));


extern volatile __bit SEG31COM3 __attribute__((address(0x3DAF)));


extern volatile __bit SEG32 __attribute__((address(0x71)));


extern volatile __bit SEG32COM0 __attribute__((address(0x3D68)));


extern volatile __bit SEG32COM1 __attribute__((address(0x3D80)));


extern volatile __bit SEG32COM2 __attribute__((address(0x3D98)));


extern volatile __bit SEG32COM3 __attribute__((address(0x3DB0)));


extern volatile __bit SEG33 __attribute__((address(0x60)));


extern volatile __bit SEG33COM0 __attribute__((address(0x3D69)));


extern volatile __bit SEG33COM1 __attribute__((address(0x3D81)));


extern volatile __bit SEG33COM2 __attribute__((address(0x3D99)));


extern volatile __bit SEG33COM3 __attribute__((address(0x3DB1)));


extern volatile __bit SEG34 __attribute__((address(0x62)));


extern volatile __bit SEG34COM0 __attribute__((address(0x3D6A)));


extern volatile __bit SEG34COM1 __attribute__((address(0x3D82)));


extern volatile __bit SEG34COM2 __attribute__((address(0x3D9A)));


extern volatile __bit SEG34COM3 __attribute__((address(0x3DB2)));


extern volatile __bit SEG35 __attribute__((address(0x63)));


extern volatile __bit SEG35COM0 __attribute__((address(0x3D6B)));


extern volatile __bit SEG35COM1 __attribute__((address(0x3D83)));


extern volatile __bit SEG35COM2 __attribute__((address(0x3D9B)));


extern volatile __bit SEG35COM3 __attribute__((address(0x3DB3)));


extern volatile __bit SEG36 __attribute__((address(0x66)));


extern volatile __bit SEG36COM0 __attribute__((address(0x3D6C)));


extern volatile __bit SEG36COM1 __attribute__((address(0x3D84)));


extern volatile __bit SEG36COM2 __attribute__((address(0x3D9C)));


extern volatile __bit SEG36COM3 __attribute__((address(0x3DB4)));


extern volatile __bit SEG37 __attribute__((address(0x67)));


extern volatile __bit SEG37COM0 __attribute__((address(0x3D6D)));


extern volatile __bit SEG37COM1 __attribute__((address(0x3D85)));


extern volatile __bit SEG37COM2 __attribute__((address(0x3D9D)));


extern volatile __bit SEG37COM3 __attribute__((address(0x3DB5)));


extern volatile __bit SEG38 __attribute__((address(0x6E)));


extern volatile __bit SEG38COM0 __attribute__((address(0x3D6E)));


extern volatile __bit SEG38COM1 __attribute__((address(0x3D86)));


extern volatile __bit SEG38COM2 __attribute__((address(0x3D9E)));


extern volatile __bit SEG38COM3 __attribute__((address(0x3DB6)));


extern volatile __bit SEG39 __attribute__((address(0x6F)));


extern volatile __bit SEG39COM0 __attribute__((address(0x3D6F)));


extern volatile __bit SEG39COM1 __attribute__((address(0x3D87)));


extern volatile __bit SEG39COM2 __attribute__((address(0x3D9F)));


extern volatile __bit SEG39COM3 __attribute__((address(0x3DB7)));


extern volatile __bit SEG3COM0 __attribute__((address(0x3D03)));


extern volatile __bit SEG3COM1 __attribute__((address(0x3D1B)));


extern volatile __bit SEG3COM2 __attribute__((address(0x3D33)));


extern volatile __bit SEG3COM3 __attribute__((address(0x3D4B)));


extern volatile __bit SEG4 __attribute__((address(0x7C)));


extern volatile __bit SEG40 __attribute__((address(0x70)));


extern volatile __bit SEG40COM0 __attribute__((address(0x3D70)));


extern volatile __bit SEG40COM1 __attribute__((address(0x3D88)));


extern volatile __bit SEG40COM2 __attribute__((address(0x3DA0)));


extern volatile __bit SEG40COM3 __attribute__((address(0x3DB8)));


extern volatile __bit SEG41 __attribute__((address(0x1460)));


extern volatile __bit SEG41COM0 __attribute__((address(0x3D71)));


extern volatile __bit SEG41COM1 __attribute__((address(0x3D89)));


extern volatile __bit SEG41COM2 __attribute__((address(0x3DA1)));


extern volatile __bit SEG41COM3 __attribute__((address(0x3DB9)));


extern volatile __bit SEG42 __attribute__((address(0x1468)));


extern volatile __bit SEG42COM0 __attribute__((address(0x3D72)));


extern volatile __bit SEG42COM1 __attribute__((address(0x3D8A)));


extern volatile __bit SEG42COM2 __attribute__((address(0x3DA2)));


extern volatile __bit SEG42COM3 __attribute__((address(0x3DBA)));


extern volatile __bit SEG43 __attribute__((address(0x1469)));


extern volatile __bit SEG43COM0 __attribute__((address(0x3D73)));


extern volatile __bit SEG43COM1 __attribute__((address(0x3D8B)));


extern volatile __bit SEG43COM2 __attribute__((address(0x3DA3)));


extern volatile __bit SEG43COM3 __attribute__((address(0x3DBB)));


extern volatile __bit SEG44 __attribute__((address(0x146A)));


extern volatile __bit SEG44COM0 __attribute__((address(0x3D74)));


extern volatile __bit SEG44COM1 __attribute__((address(0x3D8C)));


extern volatile __bit SEG44COM2 __attribute__((address(0x3DA4)));


extern volatile __bit SEG44COM3 __attribute__((address(0x3DBC)));


extern volatile __bit SEG45 __attribute__((address(0x146B)));


extern volatile __bit SEG45COM0 __attribute__((address(0x3D75)));


extern volatile __bit SEG45COM1 __attribute__((address(0x3D8D)));


extern volatile __bit SEG45COM2 __attribute__((address(0x3DA5)));


extern volatile __bit SEG45COM3 __attribute__((address(0x3DBD)));


extern volatile __bit SEG4COM0 __attribute__((address(0x3D04)));


extern volatile __bit SEG4COM1 __attribute__((address(0x3D1C)));


extern volatile __bit SEG4COM2 __attribute__((address(0x3D34)));


extern volatile __bit SEG4COM3 __attribute__((address(0x3D4C)));


extern volatile __bit SEG5 __attribute__((address(0x7D)));


extern volatile __bit SEG5COM0 __attribute__((address(0x3D05)));


extern volatile __bit SEG5COM1 __attribute__((address(0x3D1D)));


extern volatile __bit SEG5COM2 __attribute__((address(0x3D35)));


extern volatile __bit SEG5COM3 __attribute__((address(0x3D4D)));


extern volatile __bit SEG6 __attribute__((address(0x7E)));


extern volatile __bit SEG6COM0 __attribute__((address(0x3D06)));


extern volatile __bit SEG6COM1 __attribute__((address(0x3D1E)));


extern volatile __bit SEG6COM2 __attribute__((address(0x3D36)));


extern volatile __bit SEG6COM3 __attribute__((address(0x3D4E)));


extern volatile __bit SEG7 __attribute__((address(0x7F)));


extern volatile __bit SEG7COM0 __attribute__((address(0x3D07)));


extern volatile __bit SEG7COM1 __attribute__((address(0x3D1F)));


extern volatile __bit SEG7COM2 __attribute__((address(0x3D37)));


extern volatile __bit SEG7COM3 __attribute__((address(0x3D4F)));


extern volatile __bit SEG8 __attribute__((address(0x69)));


extern volatile __bit SEG8COM0 __attribute__((address(0x3D08)));


extern volatile __bit SEG8COM1 __attribute__((address(0x3D20)));


extern volatile __bit SEG8COM2 __attribute__((address(0x3D38)));


extern volatile __bit SEG8COM3 __attribute__((address(0x3D50)));


extern volatile __bit SEG9 __attribute__((address(0x6A)));


extern volatile __bit SEG9COM0 __attribute__((address(0x3D09)));


extern volatile __bit SEG9COM1 __attribute__((address(0x3D21)));


extern volatile __bit SEG9COM2 __attribute__((address(0x3D39)));


extern volatile __bit SEG9COM3 __attribute__((address(0x3D51)));


extern volatile __bit SLPEN __attribute__((address(0x3C8E)));


extern volatile __bit SPLLEN __attribute__((address(0x4CF)));


extern volatile __bit SRCLK0 __attribute__((address(0x8D4)));


extern volatile __bit SRCLK1 __attribute__((address(0x8D5)));


extern volatile __bit SRCLK2 __attribute__((address(0x8D6)));


extern volatile __bit SRI __attribute__((address(0x68)));


extern volatile __bit SRLEN __attribute__((address(0x8D7)));


extern volatile __bit SRNQ __attribute__((address(0x1461)));


extern volatile __bit SRNQEN __attribute__((address(0x8D2)));


extern volatile __bit SRPR __attribute__((address(0x8D0)));


extern volatile __bit SRPS __attribute__((address(0x8D1)));


extern volatile __bit SRQ __attribute__((address(0x1462)));


extern volatile __bit SRQEN __attribute__((address(0x8D3)));


extern volatile __bit SRRC1E __attribute__((address(0x8D8)));


extern volatile __bit SRRC2E __attribute__((address(0x8D9)));


extern volatile __bit SRRCKE __attribute__((address(0x8DA)));


extern volatile __bit SRRPE __attribute__((address(0x8DB)));


extern volatile __bit SRSC1E __attribute__((address(0x8DC)));


extern volatile __bit SRSC2E __attribute__((address(0x8DD)));


extern volatile __bit SRSCKE __attribute__((address(0x8DE)));


extern volatile __bit SRSPE __attribute__((address(0x8DF)));


extern volatile __bit SSP1IE __attribute__((address(0x48B)));


extern volatile __bit SSP1IF __attribute__((address(0x8B)));


extern volatile __bit SSP2IE __attribute__((address(0x4A0)));


extern volatile __bit SSP2IF __attribute__((address(0xA0)));


extern volatile __bit SSPIE __attribute__((address(0x48B)));


extern volatile __bit SSPIF __attribute__((address(0x8B)));


extern volatile __bit STKOVF __attribute__((address(0x4B7)));


extern volatile __bit STKUNF __attribute__((address(0x4B6)));


extern volatile __bit STR1A __attribute__((address(0x14B0)));


extern volatile __bit STR1B __attribute__((address(0x14B1)));


extern volatile __bit STR1C __attribute__((address(0x14B2)));


extern volatile __bit STR1D __attribute__((address(0x14B3)));


extern volatile __bit STR1SYNC __attribute__((address(0x14B4)));


extern volatile __bit STR2A __attribute__((address(0x14E8)));


extern volatile __bit STR2B __attribute__((address(0x14E9)));


extern volatile __bit STR2C __attribute__((address(0x14EA)));


extern volatile __bit STR2D __attribute__((address(0x14EB)));


extern volatile __bit STR2SYNC __attribute__((address(0x14EC)));


extern volatile __bit STR3A __attribute__((address(0x18B0)));


extern volatile __bit STR3B __attribute__((address(0x18B1)));


extern volatile __bit STR3C __attribute__((address(0x18B2)));


extern volatile __bit STR3D __attribute__((address(0x18B3)));


extern volatile __bit STR3SYNC __attribute__((address(0x18B4)));


extern volatile __bit SWDTEN __attribute__((address(0x4B8)));


extern volatile __bit T0CKI __attribute__((address(0x64)));


extern volatile __bit T0CS __attribute__((address(0x4AD)));


extern volatile __bit T0IE __attribute__((address(0x5D)));


extern volatile __bit T0IF __attribute__((address(0x5A)));


extern volatile __bit T0SE __attribute__((address(0x4AC)));


extern volatile __bit T0XCS __attribute__((address(0xF0)));


extern volatile __bit T1CKI __attribute__((address(0x70)));


extern volatile __bit T1CKPS0 __attribute__((address(0xC4)));


extern volatile __bit T1CKPS1 __attribute__((address(0xC5)));


extern volatile __bit T1G __attribute__((address(0x6D)));


extern volatile __bit T1GGO __attribute__((address(0xCB)));


extern volatile __bit T1GGO_nDONE __attribute__((address(0xCB)));


extern volatile __bit T1GPOL __attribute__((address(0xCE)));


extern volatile __bit T1GSPM __attribute__((address(0xCC)));


extern volatile __bit T1GSS0 __attribute__((address(0xC8)));


extern volatile __bit T1GSS1 __attribute__((address(0xC9)));


extern volatile __bit T1GTM __attribute__((address(0xCD)));


extern volatile __bit T1GVAL __attribute__((address(0xCA)));


extern volatile __bit T1OSCEN __attribute__((address(0xC3)));


extern volatile __bit T1OSCR __attribute__((address(0x4D7)));


extern volatile __bit T1OSI __attribute__((address(0x71)));


extern volatile __bit T1OSO __attribute__((address(0x70)));


extern volatile __bit T2CKPS0 __attribute__((address(0xE0)));


extern volatile __bit T2CKPS1 __attribute__((address(0xE1)));


extern volatile __bit T2OUTPS0 __attribute__((address(0xE3)));


extern volatile __bit T2OUTPS1 __attribute__((address(0xE4)));


extern volatile __bit T2OUTPS2 __attribute__((address(0xE5)));


extern volatile __bit T2OUTPS3 __attribute__((address(0xE6)));


extern volatile __bit T4CKPS0 __attribute__((address(0x20B8)));


extern volatile __bit T4CKPS1 __attribute__((address(0x20B9)));


extern volatile __bit T4OUTPS0 __attribute__((address(0x20BB)));


extern volatile __bit T4OUTPS1 __attribute__((address(0x20BC)));


extern volatile __bit T4OUTPS2 __attribute__((address(0x20BD)));


extern volatile __bit T4OUTPS3 __attribute__((address(0x20BE)));


extern volatile __bit T6CKPS0 __attribute__((address(0x20F0)));


extern volatile __bit T6CKPS1 __attribute__((address(0x20F1)));


extern volatile __bit T6OUTPS0 __attribute__((address(0x20F3)));


extern volatile __bit T6OUTPS1 __attribute__((address(0x20F4)));


extern volatile __bit T6OUTPS2 __attribute__((address(0x20F5)));


extern volatile __bit T6OUTPS3 __attribute__((address(0x20F6)));


extern volatile __bit TMR0CS __attribute__((address(0x4AD)));


extern volatile __bit TMR0IE __attribute__((address(0x5D)));


extern volatile __bit TMR0IF __attribute__((address(0x5A)));


extern volatile __bit TMR0SE __attribute__((address(0x4AC)));


extern volatile __bit TMR1CS0 __attribute__((address(0xC6)));


extern volatile __bit TMR1CS1 __attribute__((address(0xC7)));


extern volatile __bit TMR1GE __attribute__((address(0xCF)));


extern volatile __bit TMR1GIE __attribute__((address(0x48F)));


extern volatile __bit TMR1GIF __attribute__((address(0x8F)));


extern volatile __bit TMR1IE __attribute__((address(0x488)));


extern volatile __bit TMR1IF __attribute__((address(0x88)));


extern volatile __bit TMR1ON __attribute__((address(0xC0)));


extern volatile __bit TMR2IE __attribute__((address(0x489)));


extern volatile __bit TMR2IF __attribute__((address(0x89)));


extern volatile __bit TMR2ON __attribute__((address(0xE2)));


extern volatile __bit TMR4IE __attribute__((address(0x499)));


extern volatile __bit TMR4IF __attribute__((address(0x99)));


extern volatile __bit TMR4ON __attribute__((address(0x20BA)));


extern volatile __bit TMR6IE __attribute__((address(0x49B)));


extern volatile __bit TMR6IF __attribute__((address(0x9B)));


extern volatile __bit TMR6ON __attribute__((address(0x20F2)));


extern volatile __bit TRISA0 __attribute__((address(0x460)));


extern volatile __bit TRISA1 __attribute__((address(0x461)));


extern volatile __bit TRISA2 __attribute__((address(0x462)));


extern volatile __bit TRISA3 __attribute__((address(0x463)));


extern volatile __bit TRISA4 __attribute__((address(0x464)));


extern volatile __bit TRISA5 __attribute__((address(0x465)));


extern volatile __bit TRISA6 __attribute__((address(0x466)));


extern volatile __bit TRISA7 __attribute__((address(0x467)));


extern volatile __bit TRISB0 __attribute__((address(0x468)));


extern volatile __bit TRISB1 __attribute__((address(0x469)));


extern volatile __bit TRISB2 __attribute__((address(0x46A)));


extern volatile __bit TRISB3 __attribute__((address(0x46B)));


extern volatile __bit TRISB4 __attribute__((address(0x46C)));


extern volatile __bit TRISB5 __attribute__((address(0x46D)));


extern volatile __bit TRISB6 __attribute__((address(0x46E)));


extern volatile __bit TRISB7 __attribute__((address(0x46F)));


extern volatile __bit TRISC0 __attribute__((address(0x470)));


extern volatile __bit TRISC1 __attribute__((address(0x471)));


extern volatile __bit TRISC2 __attribute__((address(0x472)));


extern volatile __bit TRISC3 __attribute__((address(0x473)));


extern volatile __bit TRISC4 __attribute__((address(0x474)));


extern volatile __bit TRISC5 __attribute__((address(0x475)));


extern volatile __bit TRISC6 __attribute__((address(0x476)));


extern volatile __bit TRISC7 __attribute__((address(0x477)));


extern volatile __bit TRISD0 __attribute__((address(0x478)));


extern volatile __bit TRISD1 __attribute__((address(0x479)));


extern volatile __bit TRISD2 __attribute__((address(0x47A)));


extern volatile __bit TRISD3 __attribute__((address(0x47B)));


extern volatile __bit TRISD4 __attribute__((address(0x47C)));


extern volatile __bit TRISD5 __attribute__((address(0x47D)));


extern volatile __bit TRISD6 __attribute__((address(0x47E)));


extern volatile __bit TRISD7 __attribute__((address(0x47F)));


extern volatile __bit TRISE0 __attribute__((address(0x480)));


extern volatile __bit TRISE1 __attribute__((address(0x481)));


extern volatile __bit TRISE2 __attribute__((address(0x482)));


extern volatile __bit TRISE3 __attribute__((address(0x483)));


extern volatile __bit TRISE4 __attribute__((address(0x484)));


extern volatile __bit TRISE5 __attribute__((address(0x485)));


extern volatile __bit TRISE6 __attribute__((address(0x486)));


extern volatile __bit TRISE7 __attribute__((address(0x487)));


extern volatile __bit TRISF0 __attribute__((address(0x1860)));


extern volatile __bit TRISF1 __attribute__((address(0x1861)));


extern volatile __bit TRISF2 __attribute__((address(0x1862)));


extern volatile __bit TRISF3 __attribute__((address(0x1863)));


extern volatile __bit TRISF4 __attribute__((address(0x1864)));


extern volatile __bit TRISF5 __attribute__((address(0x1865)));


extern volatile __bit TRISF6 __attribute__((address(0x1866)));


extern volatile __bit TRISF7 __attribute__((address(0x1867)));


extern volatile __bit TRISG0 __attribute__((address(0x1868)));


extern volatile __bit TRISG1 __attribute__((address(0x1869)));


extern volatile __bit TRISG2 __attribute__((address(0x186A)));


extern volatile __bit TRISG3 __attribute__((address(0x186B)));


extern volatile __bit TRISG4 __attribute__((address(0x186C)));


extern volatile __bit TRISG5 __attribute__((address(0x186D)));


extern volatile __bit TSEN __attribute__((address(0x8BD)));


extern volatile __bit TSRNG __attribute__((address(0x8BC)));


extern volatile __bit TUN0 __attribute__((address(0x4C0)));


extern volatile __bit TUN1 __attribute__((address(0x4C1)));


extern volatile __bit TUN2 __attribute__((address(0x4C2)));


extern volatile __bit TUN3 __attribute__((address(0x4C3)));


extern volatile __bit TUN4 __attribute__((address(0x4C4)));


extern volatile __bit TUN5 __attribute__((address(0x4C5)));


extern volatile __bit TX1 __attribute__((address(0x76)));


extern volatile __bit TX1IE __attribute__((address(0x48C)));


extern volatile __bit TX1IF __attribute__((address(0x8C)));


extern volatile __bit TX2 __attribute__((address(0x1469)));


extern volatile __bit TX2IE __attribute__((address(0x4A4)));


extern volatile __bit TX2IF __attribute__((address(0xA4)));


extern volatile __bit TXIE __attribute__((address(0x48C)));


extern volatile __bit TXIF __attribute__((address(0x8C)));


extern volatile __bit VLCD1 __attribute__((address(0x80)));


extern volatile __bit VLCD1PE __attribute__((address(0x3C99)));


extern volatile __bit VLCD2 __attribute__((address(0x81)));


extern volatile __bit VLCD2PE __attribute__((address(0x3C9A)));


extern volatile __bit VLCD3 __attribute__((address(0x82)));


extern volatile __bit VLCD3PE __attribute__((address(0x3C9B)));


extern volatile __bit VREFM __attribute__((address(0x62)));


extern volatile __bit VREFP __attribute__((address(0x63)));


extern volatile __bit WA __attribute__((address(0x3C94)));


extern volatile __bit WDTPS0 __attribute__((address(0x4B9)));


extern volatile __bit WDTPS1 __attribute__((address(0x4BA)));


extern volatile __bit WDTPS2 __attribute__((address(0x4BB)));


extern volatile __bit WDTPS3 __attribute__((address(0x4BC)));


extern volatile __bit WDTPS4 __attribute__((address(0x4BD)));


extern volatile __bit WERR __attribute__((address(0x3C8D)));


extern volatile __bit WFT __attribute__((address(0x3C97)));


extern volatile __bit WPUB0 __attribute__((address(0x1068)));


extern volatile __bit WPUB1 __attribute__((address(0x1069)));


extern volatile __bit WPUB2 __attribute__((address(0x106A)));


extern volatile __bit WPUB3 __attribute__((address(0x106B)));


extern volatile __bit WPUB4 __attribute__((address(0x106C)));


extern volatile __bit WPUB5 __attribute__((address(0x106D)));


extern volatile __bit WPUB6 __attribute__((address(0x106E)));


extern volatile __bit WPUB7 __attribute__((address(0x106F)));


extern volatile __bit WPUG5 __attribute__((address(0x246D)));


extern volatile __bit WR __attribute__((address(0xCA9)));


extern volatile __bit WREN __attribute__((address(0xCAA)));


extern volatile __bit WRERR __attribute__((address(0xCAB)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit Z_SHAD __attribute__((address(0x7F22)));


extern volatile __bit nBOR __attribute__((address(0x4B0)));


extern volatile __bit nDONE __attribute__((address(0x4E9)));


extern volatile __bit nMCLR __attribute__((address(0x146D)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nPOR __attribute__((address(0x4B1)));


extern volatile __bit nRI __attribute__((address(0x4B2)));


extern volatile __bit nRMCLR __attribute__((address(0x4B3)));


extern volatile __bit nSS2 __attribute__((address(0x7F)));


extern volatile __bit nT1SYNC __attribute__((address(0xC2)));


extern volatile __bit nTO __attribute__((address(0x1C)));


extern volatile __bit nWPUEN __attribute__((address(0x4AF)));
# 450 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic_chip_select.h" 2 3
# 14 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 2 3
# 30 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 3
#pragma intrinsic(__nop)
extern void __nop(void);
# 78 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);



# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\eeprom_routines.h" 1 3
# 114 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 86 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 2 3





#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(unsigned long);

#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(unsigned char);
# 137 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 28 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\xc.h" 2 3
# 49 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/device_config.h" 1
# 50 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/pin_manager.h" 1
# 325 "./mcc_generated_files/pin_manager.h"
void PIN_MANAGER_Initialize (void);
# 337 "./mcc_generated_files/pin_manager.h"
void PIN_MANAGER_IOC(void);
# 51 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 1 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdint.h" 2 3
# 52 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdbool.h" 1 3
# 53 "./mcc_generated_files/mcc.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\conio.h" 1 3







# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\errno.h" 1 3
# 12 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\errno.h" 3
extern int errno;
# 8 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\conio.h" 2 3

# 1 "C:/Program Files/Microchip/MPLABX/v5.45/packs/Microchip/PIC12-16F1xxx_DFP/1.2.63/xc8\\pic\\include\\__null.h" 1 3
# 9 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\conio.h" 2 3



extern void init_uart(void);

extern char getch(void);
extern char getche(void);
extern void putch(char);
extern void ungetch(char);

extern __bit kbhit(void);



extern char * cgets(char *);
extern void cputs(const char *);
# 54 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/interrupt_manager.h" 1
# 55 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/i2c2_master.h" 1
# 54 "./mcc_generated_files/i2c2_master.h"
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdio.h" 1 3
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdio.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 1 3





typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 137 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long ssize_t;
# 246 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long off_t;
# 399 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdio.h" 2 3
# 52 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);
int ungetc(int, FILE *);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

#pragma printf_check(printf) const
#pragma printf_check(vprintf) const
#pragma printf_check(sprintf) const
#pragma printf_check(snprintf) const
#pragma printf_check(vsprintf) const
#pragma printf_check(vsnprintf) const

int printf(const char *restrict, ...);
int fprintf(FILE *restrict, const char *restrict, ...);
int sprintf(char *restrict, const char *restrict, ...);
int snprintf(char *restrict, size_t, const char *restrict, ...);

int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

int scanf(const char *restrict, ...);
int fscanf(FILE *restrict, const char *restrict, ...);
int sscanf(const char *restrict, const char *restrict, ...);
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 54 "./mcc_generated_files/i2c2_master.h" 2




typedef enum {
    I2C2_NOERR,
    I2C2_BUSY,
    I2C2_FAIL


} i2c2_error_t;

typedef enum
{
    I2C2_STOP=1,
    I2C2_RESTART_READ,
    I2C2_RESTART_WRITE,
    I2C2_CONTINUE,
    I2C2_RESET_LINK
} i2c2_operations_t;

typedef uint8_t i2c2_address_t;
typedef i2c2_operations_t (*i2c2_callback_t)(void *funPtr);


i2c2_operations_t I2C2_CallbackReturnStop(void *funPtr);
i2c2_operations_t I2C2_CallbackReturnReset(void *funPtr);
i2c2_operations_t I2C2_CallbackRestartWrite(void *funPtr);
i2c2_operations_t I2C2_CallbackRestartRead(void *funPtr);






void I2C2_Initialize(void);
# 101 "./mcc_generated_files/i2c2_master.h"
i2c2_error_t I2C2_Open(i2c2_address_t address);
# 111 "./mcc_generated_files/i2c2_master.h"
i2c2_error_t I2C2_Close(void);
# 123 "./mcc_generated_files/i2c2_master.h"
i2c2_error_t I2C2_MasterOperation(_Bool read);




i2c2_error_t I2C2_MasterWrite(void);




i2c2_error_t I2C2_MasterRead(void);
# 142 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetTimeout(uint8_t timeOut);
# 152 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetBuffer(void *buffer, size_t bufferSize);
# 164 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetDataCompleteCallback(i2c2_callback_t cb, void *ptr);
# 174 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetWriteCollisionCallback(i2c2_callback_t cb, void *ptr);
# 184 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetAddressNackCallback(i2c2_callback_t cb, void *ptr);
# 194 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetDataNackCallback(i2c2_callback_t cb, void *ptr);
# 204 "./mcc_generated_files/i2c2_master.h"
void I2C2_SetTimeoutCallback(i2c2_callback_t cb, void *ptr);
# 56 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/i2c1_master.h" 1
# 58 "./mcc_generated_files/i2c1_master.h"
typedef enum {
    I2C1_NOERR,
    I2C1_BUSY,
    I2C1_FAIL


} i2c1_error_t;

typedef enum
{
    I2C1_STOP=1,
    I2C1_RESTART_READ,
    I2C1_RESTART_WRITE,
    I2C1_CONTINUE,
    I2C1_RESET_LINK
} i2c1_operations_t;

typedef uint8_t i2c1_address_t;
typedef i2c1_operations_t (*i2c1_callback_t)(void *funPtr);


i2c1_operations_t I2C1_CallbackReturnStop(void *funPtr);
i2c1_operations_t I2C1_CallbackReturnReset(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartWrite(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartRead(void *funPtr);






void I2C1_Initialize(void);
# 101 "./mcc_generated_files/i2c1_master.h"
i2c1_error_t I2C1_Open(i2c1_address_t address);
# 111 "./mcc_generated_files/i2c1_master.h"
i2c1_error_t I2C1_Close(void);
# 123 "./mcc_generated_files/i2c1_master.h"
i2c1_error_t I2C1_MasterOperation(_Bool read);




i2c1_error_t I2C1_MasterWrite(void);




i2c1_error_t I2C1_MasterRead(void);
# 142 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetTimeout(uint8_t timeOut);
# 152 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetBuffer(void *buffer, size_t bufferSize);
# 164 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetDataCompleteCallback(i2c1_callback_t cb, void *ptr);
# 174 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetWriteCollisionCallback(i2c1_callback_t cb, void *ptr);
# 184 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetAddressNackCallback(i2c1_callback_t cb, void *ptr);
# 194 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetDataNackCallback(i2c1_callback_t cb, void *ptr);
# 204 "./mcc_generated_files/i2c1_master.h"
void I2C1_SetTimeoutCallback(i2c1_callback_t cb, void *ptr);
# 57 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/tmr1.h" 1
# 101 "./mcc_generated_files/tmr1.h"
void TMR1_Initialize(void);
# 130 "./mcc_generated_files/tmr1.h"
void TMR1_StartTimer(void);
# 162 "./mcc_generated_files/tmr1.h"
void TMR1_StopTimer(void);
# 197 "./mcc_generated_files/tmr1.h"
uint16_t TMR1_ReadTimer(void);
# 236 "./mcc_generated_files/tmr1.h"
void TMR1_WriteTimer(uint16_t timerVal);
# 272 "./mcc_generated_files/tmr1.h"
void TMR1_Reload(void);
# 311 "./mcc_generated_files/tmr1.h"
void TMR1_StartSinglePulseAcquisition(void);
# 350 "./mcc_generated_files/tmr1.h"
uint8_t TMR1_CheckGateValueStatus(void);
# 368 "./mcc_generated_files/tmr1.h"
void TMR1_ISR(void);
# 385 "./mcc_generated_files/tmr1.h"
void TMR1_CallBack(void);
# 403 "./mcc_generated_files/tmr1.h"
 void TMR1_SetInterruptHandler(void (* InterruptHandler)(void));
# 421 "./mcc_generated_files/tmr1.h"
extern void (*TMR1_InterruptHandler)(void);
# 439 "./mcc_generated_files/tmr1.h"
void TMR1_DefaultInterruptHandler(void);
# 58 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/eusart2.h" 1
# 75 "./mcc_generated_files/eusart2.h"
typedef union {
    struct {
        unsigned perr : 1;
        unsigned ferr : 1;
        unsigned oerr : 1;
        unsigned reserved : 5;
    };
    uint8_t status;
}eusart2_status_t;
# 110 "./mcc_generated_files/eusart2.h"
void EUSART2_Initialize(void);
# 158 "./mcc_generated_files/eusart2.h"
_Bool EUSART2_is_tx_ready(void);
# 206 "./mcc_generated_files/eusart2.h"
_Bool EUSART2_is_rx_ready(void);
# 253 "./mcc_generated_files/eusart2.h"
_Bool EUSART2_is_tx_done(void);
# 301 "./mcc_generated_files/eusart2.h"
eusart2_status_t EUSART2_get_last_status(void);
# 321 "./mcc_generated_files/eusart2.h"
uint8_t EUSART2_Read(void);
# 341 "./mcc_generated_files/eusart2.h"
void EUSART2_Write(uint8_t txData);
# 361 "./mcc_generated_files/eusart2.h"
void EUSART2_SetFramingErrorHandler(void (* interruptHandler)(void));
# 379 "./mcc_generated_files/eusart2.h"
void EUSART2_SetOverrunErrorHandler(void (* interruptHandler)(void));
# 397 "./mcc_generated_files/eusart2.h"
void EUSART2_SetErrorHandler(void (* interruptHandler)(void));
# 59 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/eusart1.h" 1
# 75 "./mcc_generated_files/eusart1.h"
typedef union {
    struct {
        unsigned perr : 1;
        unsigned ferr : 1;
        unsigned oerr : 1;
        unsigned reserved : 5;
    };
    uint8_t status;
}eusart1_status_t;
# 110 "./mcc_generated_files/eusart1.h"
void EUSART1_Initialize(void);
# 158 "./mcc_generated_files/eusart1.h"
_Bool EUSART1_is_tx_ready(void);
# 206 "./mcc_generated_files/eusart1.h"
_Bool EUSART1_is_rx_ready(void);
# 253 "./mcc_generated_files/eusart1.h"
_Bool EUSART1_is_tx_done(void);
# 301 "./mcc_generated_files/eusart1.h"
eusart1_status_t EUSART1_get_last_status(void);
# 321 "./mcc_generated_files/eusart1.h"
uint8_t EUSART1_Read(void);
# 341 "./mcc_generated_files/eusart1.h"
void EUSART1_Write(uint8_t txData);
# 361 "./mcc_generated_files/eusart1.h"
void EUSART1_SetFramingErrorHandler(void (* interruptHandler)(void));
# 379 "./mcc_generated_files/eusart1.h"
void EUSART1_SetOverrunErrorHandler(void (* interruptHandler)(void));
# 397 "./mcc_generated_files/eusart1.h"
void EUSART1_SetErrorHandler(void (* interruptHandler)(void));
# 60 "./mcc_generated_files/mcc.h" 2

# 1 "./mcc_generated_files/drivers/i2c_simple_master.h" 1
# 37 "./mcc_generated_files/drivers/i2c_simple_master.h"
uint8_t i2c_read1ByteRegister(i2c1_address_t address, uint8_t reg);
uint16_t i2c_read2ByteRegister(i2c1_address_t address, uint8_t reg);
void i2c_write1ByteRegister(i2c1_address_t address, uint8_t reg, uint8_t data);
void i2c_write2ByteRegister(i2c1_address_t address, uint8_t reg, uint16_t data);

void i2c_writeNBytes(i2c1_address_t address, void* data, size_t len);
void i2c_readDataBlock(i2c1_address_t address, uint8_t reg, void *data, size_t len);
void i2c_readNBytes(i2c1_address_t address, void *data, size_t len);
# 61 "./mcc_generated_files/mcc.h" 2
# 76 "./mcc_generated_files/mcc.h"
void SYSTEM_Initialize(void);
# 89 "./mcc_generated_files/mcc.h"
void OSCILLATOR_Initialize(void);
# 101 "./mcc_generated_files/mcc.h"
void WDT_Initialize(void);
# 2 "main.c" 2

# 1 "./mcc_generated_files/examples/i2c1_master_example.h" 1
# 54 "./mcc_generated_files/examples/i2c1_master_example.h"
uint8_t I2C1_Read1ByteRegister(i2c1_address_t address, uint8_t reg);
uint16_t I2C1_Read2ByteRegister(i2c1_address_t address, uint8_t reg);
void I2C1_Write1ByteRegister(i2c1_address_t address, uint8_t reg, uint8_t data);
void I2C1_Write2ByteRegister(i2c1_address_t address, uint8_t reg, uint16_t data);
void I2C1_WriteNBytes(i2c1_address_t address, uint8_t *data, size_t len);
void I2C1_ReadNBytes(i2c1_address_t address, uint8_t *data, size_t len);
void I2C1_ReadDataBlock(i2c1_address_t address, uint8_t reg, uint8_t *data, size_t len);
# 3 "main.c" 2

# 1 "./mcc_generated_files/examples/i2c2_master_example.h" 1
# 54 "./mcc_generated_files/examples/i2c2_master_example.h"
uint8_t I2C2_Read1ByteRegister(i2c2_address_t address, uint8_t reg);
uint16_t I2C2_Read2ByteRegister(i2c2_address_t address, uint8_t reg);
void I2C2_Write1ByteRegister(i2c2_address_t address, uint8_t reg, uint8_t data);
void I2C2_Write2ByteRegister(i2c2_address_t address, uint8_t reg, uint16_t data);
void I2C2_WriteNBytes(i2c2_address_t address, uint8_t *data, size_t len);
void I2C2_ReadNBytes(i2c2_address_t address, uint8_t *data, size_t len);
void I2C2_ReadDataBlock(i2c2_address_t address, uint8_t reg, uint8_t *data, size_t len);
# 4 "main.c" 2



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\string.h" 1 3
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\string.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 411 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct __locale_struct * locale_t;
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\string.h" 2 3


void *memcpy (void *restrict, const void *restrict, size_t);
void *memmove (void *, const void *, size_t);
void *memset (void *, int, size_t);
int memcmp (const void *, const void *, size_t);
void *memchr (const void *, int, size_t);

char *strcpy (char *restrict, const char *restrict);
char *strncpy (char *restrict, const char *restrict, size_t);

char *strcat (char *restrict, const char *restrict);
char *strncat (char *restrict, const char *restrict, size_t);

int strcmp (const char *, const char *);
int strncmp (const char *, const char *, size_t);

int strcoll (const char *, const char *);
size_t strxfrm (char *restrict, const char *restrict, size_t);

char *strchr (const char *, int);
char *strrchr (const char *, int);

size_t strcspn (const char *, const char *);
size_t strspn (const char *, const char *);
char *strpbrk (const char *, const char *);
char *strstr (const char *, const char *);
char *strtok (char *restrict, const char *restrict);

size_t strlen (const char *);

char *strerror (int);
# 65 "C:\\Program Files\\Microchip\\xc8\\v2.32\\pic\\include\\c99\\string.h" 3
char *strtok_r (char *restrict, const char *restrict, char **restrict);
int strerror_r (int, char *, size_t);
char *stpcpy(char *restrict, const char *restrict);
char *stpncpy(char *restrict, const char *restrict, size_t);
size_t strnlen (const char *, size_t);
char *strdup (const char *);
char *strndup (const char *, size_t);
char *strsignal(int);
char *strerror_l (int, locale_t);
int strcoll_l (const char *, const char *, locale_t);
size_t strxfrm_l (char *restrict, const char *restrict, size_t, locale_t);




void *memccpy (void *restrict, const void *restrict, int, size_t);
# 7 "main.c" 2
# 62 "main.c"
unsigned int t_left = 1000, cntLeft = 0;
unsigned int t_right = 1000, cntRight = 0;
uint16_t totalTime = 0, cntTime = 0;
unsigned char flag_stop = 0;


uint8_t shift = 0, shift_one = 0;
float distance_raw[2] = {0,0};
float distance_raw_one[2] = {0,0};
float DistanceValueTop = 0;
float DistanceValueBottom = 0;


float errorOmega=0, errorOmega_pre=0;
double errorOmega_sum = 0;
float PID_Omega=0;


unsigned char flagManual = 0;
float velManual = 0, velManual_t = 0;
float omegaManual = 0;
int cntManual = 0;


unsigned char flagALine = 0, cntALine = 0;
unsigned int length_ALine = 0;
int direct_ALine = 0;
float velALine = 0, omegaALine = 0, timeALine = 0;


unsigned char flagACircle = 0, cntACircle = 0;
unsigned int angle_ACircle = 0;
float velACircle = 0, omegaACircle = 0, valRadius_AC = 0, timeACircle = 0;


unsigned char cntCamera = 0, cnt_frame = 0;
unsigned char flagCamera = 0;
float valRadius_TL = 0, valVLength_TL = 0, valTime_TL = 0;
unsigned int sum_frame = 0, delay_frame = 0;
float angle_move = 0, omega_TL = 0, velocity_TL = 100, time_move = 0;


char buffer_sys[50];
unsigned char cnt_sys = 0;
float velocity_sys = 0, omega_sys = 0;

void putch(char value)
{
   while(!EUSART2_is_tx_ready());
   EUSART2_Write(value);
}
void T1_ROBOT_ISR()
{
    if (t_left != 1000){
        if (cntLeft < (float)(t_left/4)) cntLeft++;
        else{
            LATFbits.LATF2 = !LATFbits.LATF2;
            cntLeft = 0;
        }
    }
    if (t_right != 1000){
        if (cntRight < (float)(t_right/4)) cntRight++;
        else {
            LATFbits.LATF6 = !LATFbits.LATF6;
            cntRight = 0;
        }
    }
    if(cntTime < totalTime) cntTime = cntTime + 1;
    else{
        flag_stop = 0;
        omega_sys = 0;
        velocity_sys = 0;
        t_left = 1000;
        t_right = 1000;
    }
}

void KE(float Omega_body, float Velocity_body){
    float OmegaLeft = 0;
    float OmegaRight = 0;

    OmegaLeft = Velocity_body/32 + Omega_body*(69.5/32);
    OmegaRight = Velocity_body/32 - Omega_body*(69.5/32);




    if(OmegaLeft < 0){
        OmegaLeft = -OmegaLeft;
        LATFbits.LATF7 = 0;
    }
    else LATFbits.LATF7 = 1;

    if(OmegaRight < 0){
        OmegaRight = -OmegaRight;
        LATFbits.LATF3 = 1;
    }
    else LATFbits.LATF3 = 0;

    if (OmegaLeft == 0) t_left = 1000;
    else t_left = (unsigned int)(31.416/OmegaLeft);
    if (OmegaRight == 0) t_right = 1000;
    else t_right = (unsigned int)((31.416+2)/OmegaRight);
}

float distance_sensor_bottom(){


    shift = I2C2_Read1ByteRegister(0x80 >>1,0x35);
    distance_raw[0] = I2C2_Read1ByteRegister(0x80 >> 1,0x5E);

    distance_raw[1] = I2C2_Read1ByteRegister(0x80 >> 1,0x5F);
    return ((distance_raw[0] * 16 + distance_raw[1])/16/4);
}
float distance_sensor_top(){






    shift_one = I2C1_Read1ByteRegister(0x80 >>1,0x35);
    distance_raw_one[0] = I2C1_Read1ByteRegister(0x80 >>1,0x5E);

    distance_raw_one[1] = I2C1_Read1ByteRegister(0x80 >>1,0x5F);
    return ((distance_raw_one[0] * 16 + distance_raw_one[1])/16/4);
}

void SetUpModeHalf_step()
{
    LATFbits.LATF0 = 1;
    LATFbits.LATF1 = 0;
    LATFbits.LATF4 = 1;
    LATFbits.LATF5 = 0;
}

void ManualHandle()
{
    char bufferManual_t[20];
    if((buffer_sys[0] == 'M') || (flagManual == 1)){
        flagManual = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'M'){
                buffer_sys[0] = '0';
            }
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velManual = atoi(bufferManual_t);
                    for(int j = 0; j < strlen(bufferManual_t);j++){
                        bufferManual_t[j] = ' ';
                    }
                    cntManual = 0;
                }
                if(buffer_sys[i] == '#'){
                    flagManual = 0;

                    for(int j = 0; j < strlen(bufferManual_t);j++){
                        bufferManual_t[j] = ' ';
                    }
                }
            }
            else{
                bufferManual_t[cntManual] = buffer_sys[i];
                cntManual = cntManual + 1;
            }
        }
    }
}

void ManualControl()
{
    if(buffer_sys[0] == 'T'){
        cnt_sys = 0;
        totalTime = 100000;
        velManual_t = velManual;
        if(velManual < 0) velManual = -velManual;
        velocity_sys = velManual;


    }
    else if(buffer_sys[0] == 'B'){
        cnt_sys = 0;
        totalTime = 100000;
        velManual_t = velManual;
        if(velManual > 0) velManual = -velManual;
        velocity_sys = velManual;


    }
    else if(buffer_sys[0] == 'L'){
        cnt_sys = 0;
        totalTime = 100000;
        if(velManual != 0) velManual_t = velManual;
        velManual = 0;
        omegaManual = 7/2;
        velocity_sys = velManual;
        omega_sys = omegaManual;
    }
    else if(buffer_sys[0] == 'R'){
        cnt_sys = 0;
        totalTime = 100000;
        if(velManual != 0) velManual_t = velManual;
        velManual = 0;
        omegaManual = -7/2;
        velocity_sys = velManual;
        omega_sys = omegaManual;
    }
    else if(buffer_sys[0] == 'N'){
        cnt_sys = 0;
        cntTime = 0;
        if(velManual_t != velManual) velManual = velManual_t;
        omegaManual = 0;
        totalTime = 0;
        velocity_sys = 0;
        omega_sys = 0;
    }
}

void AutoCircleHandle()
{
    char bufferACircle_t[20];
    if((buffer_sys[0] == 'A') || (flagACircle == 1)){
        flagACircle = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'A'){
                buffer_sys[0] = '0';
            }
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == 'r') || (buffer_sys[i] == 'a') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velACircle = atoi(bufferACircle_t);

                    for(int j = 0; j < strlen(bufferACircle_t);j++){
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;
                }
                if(buffer_sys[i] == 'r'){
                    valRadius_AC = atoi(bufferACircle_t);

                    for(int j = 0; j < strlen(bufferACircle_t);j++){
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;
                }
                if(buffer_sys[i] == 'a'){
                    angle_ACircle = atoi(bufferACircle_t);

                    for(int j = 0; j < strlen(bufferACircle_t);j++){
                        bufferACircle_t[j] = ' ';
                    }
                    cntACircle = 0;
                }
                if(buffer_sys[i] == '#'){
                    flagACircle = 2;

                    for(int j = 0; j < strlen(bufferACircle_t);j++){
                        bufferACircle_t[j] = ' ';
                    }
                }
            }
            else{
                bufferACircle_t[cntACircle] = buffer_sys[i];
                cntACircle = cntACircle + 1;
            }
        }
    }
    if(flagACircle == 2){
        timeACircle = (3.14159*valRadius_AC*angle_ACircle*100)/(18*velACircle);
        omegaACircle = velACircle/valRadius_AC;
        if(totalTime != timeACircle) totalTime = timeACircle;
        cntTime = 0;
        velocity_sys = velACircle;
        omega_sys = omegaACircle;
        flagACircle = 0;
    }
}

void AutoLineHandle()
{
    char bufferALine_t[20];
    if((buffer_sys[0] == 'S') || (flagALine == 1)){
        flagALine = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'S'){
                buffer_sys[0] = '0';
            }
            if((buffer_sys[i] == 'v') || (buffer_sys[i] == 'l') || (buffer_sys[i] == 'd') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'v')){
                    velALine = atoi(bufferALine_t);

                    for(int j = 0; j < strlen(bufferALine_t);j++){
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;
                }
                if(buffer_sys[i] == 'l'){
                    length_ALine = atoi(bufferALine_t);

                    for(int j = 0; j < strlen(bufferALine_t);j++){
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;
                }
                if(buffer_sys[i] == 'd'){
                    direct_ALine = atoi(bufferALine_t);

                    for(int j = 0; j < strlen(bufferALine_t);j++){
                        bufferALine_t[j] = ' ';
                    }
                    cntALine = 0;
                }
                if(buffer_sys[i] == '#'){
                    if(direct_ALine != 0) flagALine = 2;
                    else flagALine = 3;

                    for(int j = 0; j < strlen(bufferALine_t);j++){
                        bufferALine_t[j] = ' ';
                    }
                }
            }
            else{
                bufferALine_t[cntALine] = buffer_sys[i];
                cntALine = cntALine + 1;
            }
        }
    }
}

void AutoLineControl()
{
    if(flagALine == 2){
        if(direct_ALine < 0){
            direct_ALine = -direct_ALine;
            omegaALine = -3;
        }
        else omegaALine = 3;
        timeALine = (2*3.14159*69.5*direct_ALine*100)/3753;
        cntTime = 0;
        if(totalTime != timeALine) totalTime = timeALine-135;
        KE(omegaALine,0);
        while(cntTime < totalTime);
        flagALine = 3;
        timeALine = 0;
    }
    if (flagALine == 3){
        timeALine = (length_ALine/velALine)*1000;
        if(totalTime != timeALine) totalTime = timeALine;
        cntTime = 0;
        omegaALine = 0;
        velocity_sys = velALine;
        omega_sys = omegaALine;
        flagALine = 0;
    }
}

void TimeLapseHandle()
{
    char bufferCamera_t[20];
    if((buffer_sys[0] == 'C') || (flagCamera == 1)){
        flagCamera = 1;
        for(int i = 0; i < strlen(buffer_sys); i++){
            if(buffer_sys[0] == 'C'){
                buffer_sys[0] = '0';
            }
            if((buffer_sys[i] == 'r') || (buffer_sys[i] == 'l') || (buffer_sys[i] == 't') || (buffer_sys[i] == '#')){
                if((buffer_sys[i] == 'r')){
                    valRadius_TL = atoi(bufferCamera_t);

                    for(int j = 0; j < strlen(bufferCamera_t);j++){
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;
                }
                if(buffer_sys[i] == 'l'){
                    valVLength_TL = atoi(bufferCamera_t);

                    for(int j = 0; j < strlen(bufferCamera_t);j++){
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;
                }
                if(buffer_sys[i] == 't'){
                    valTime_TL = atoi(bufferCamera_t);

                    for(int j = 0; j < strlen(bufferCamera_t);j++){
                        bufferCamera_t[j] = ' ';
                    }
                    cntCamera = 0;
                }
                if(buffer_sys[i] == '#'){
                    flagCamera = 2;

                    for(int j = 0; j < strlen(bufferCamera_t);j++){
                        bufferCamera_t[j] = ' ';
                    }
                }
            }
            else{
                bufferCamera_t[cntCamera] = buffer_sys[i];
                cntCamera = cntCamera + 1;
            }
        }
    }
    if(flagCamera == 2){
        sum_frame = valVLength_TL*1800;
        angle_move = (float)360/sum_frame;
        delay_frame = (angle_move*valTime_TL*1000)/6;
        time_move = (3.14159*valRadius_TL*angle_move*100)/(18*velocity_TL);
        omega_TL = velocity_TL/valRadius_TL;
        flagCamera = 3;
    }
}

void TimeLapseControl()
{
    if(flagCamera == 3){
        if(cnt_frame <= sum_frame){
            if(flag_stop == 0){
                cnt_frame++;
                EUSART1_Write('1');
                for(int i = 0; i < delay_frame; i++) _delay((unsigned long)((1)*(16000000/4000.0)));
                if(totalTime != time_move) totalTime = time_move;
                flag_stop = 1;
                cntTime = 0;
                velocity_sys = velocity_TL;
                omega_sys = omega_TL;
            }
        }
        else{
            EUSART1_Write('d');
            flagCamera = 0;
        }
    }
}

void PID()
{
    DistanceValueTop = distance_sensor_top();
    DistanceValueBottom = distance_sensor_bottom();
    errorOmega = (DistanceValueTop - DistanceValueBottom);
    if (errorOmega_sum < 1000) errorOmega_sum += errorOmega;
    PID_Omega = 0.34*errorOmega + 0.075*errorOmega_sum + 1*(errorOmega - errorOmega_pre);
    if(PID_Omega > 30) PID_Omega = 30;
    if(PID_Omega < -30) PID_Omega = -30;

    omega_sys = PID_Omega;
    velocity_sys = 0;
}
# 520 "main.c"
void main(void)
{

    SYSTEM_Initialize();
    TMR1_SetInterruptHandler(T1_ROBOT_ISR);






      (INTCONbits.GIE = 1);


      (INTCONbits.PEIE = 1);






    _delay((unsigned long)((300)*(16000000/4000.0)));
    LATCbits.LATC2 = 1;
    SetUpModeHalf_step();
    while (1){
        if(EUSART1_is_rx_ready()){
           buffer_sys[cnt_sys] = EUSART1_Read();
           EUSART2_Write(buffer_sys[cnt_sys]);
           cnt_sys = cnt_sys + 1;
        }
        if (buffer_sys[cnt_sys - 1] == '#') {
            ManualHandle();
            AutoCircleHandle();
            AutoLineHandle();
            TimeLapseHandle();
            cnt_sys = 0;
        }
        ManualControl();
        AutoLineControl();
        TimeLapseControl();
        KE(omega_sys, velocity_sys);


    }
}
