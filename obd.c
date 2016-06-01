#include <xc.h>
#include "obd_config.h"
#include "idCan.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "math.h"
#define _XTAL_FREQ 16000000
void board_initialization(void);
#define absled PORTDbits.RD6
#define epsled PORTDbits.RD5
#define telecomando PORTDbits.RD4
#define parcheggio PORTDbits.RD3
CANmessage msg;
__interrupt(low_priority) void ISR_Bassa(void) {
    //INTERRUPT CANBUS
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.identifier == ECU_STATE_ABS) {
                if (msg.RTR == 1) {
                    absled = 0;
                } else {
                    absled = 1;
                }
            }
            if (msg.identifier == ECU_STATE_EPS) {
                if (msg.RTR == 1) {
                    epsled = 0;
                } else {
                    epsled = 1;
                }
            }
            if (msg.identifier == ECU_STATE_REMOTECAN) {
                if (msg.RTR == 1) {
                    telecomando = 0;
                } else {
                    telecomando = 1;
                }
            }
            if (msg.identifier == PARK_ASSIST_STATE) {
                if (msg.data[0] == 4) {
                    parcheggio = 1;
                }
            }
            if (msg.identifier == SENSOR_DISTANCE) {
                if ((msg.data[0] & 0b01000000) >> 6 == 1) {
                    PORTDbits.RD0 = 1;
                } else {
                    PORTDbits.RD0 = 0;
                }
            }
        }
        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }
}

void main(void) {
    board_initialization();
    parcheggio = 1;
    delay_ms(200);
    parcheggio = 0;
    while (1) {
        PORTAbits.RA1 = ~PORTAbits.RA1;
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
        __delay_ms(10);
    }
}

void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b11111101; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0b11111011; //CAN BUS / ON-OFF SWITCH
    LATC = 0x00;
    TRISC = 0xff; //USART Tx and Rx / LCD
    LATD = 0x00;
    TRISD = 0x00; //LCD / Backlight ON/OFF
    LATE = 0x00;
    TRISE = 0x00;

    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)

    //Interrupt Flags
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    //Interrupts Priority
    RCONbits.IPEN = 1; //abilita priorità interrupt
    IPR3bits.RXB1IP = 0; //interrupt alta priorità per can
    IPR3bits.RXB0IP = 0; //interrupt alta priorità per can
    //Interrupts Enables
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
}