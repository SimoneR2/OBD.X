#include <xc.h>
#include "obd_config.h"
#include "idCan.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#include "math.h"
#define _XTAL_FREQ 16000000
void board_initialization (void);
#define absled port
#define epsled port
#define motore port
__interrupt(low_priority) void ISR_Bassa(void) {
    //INTERRUPT CANBUS
    if ((PIR3bits.RXB0IF == HIGH) || (PIR3bits.RXB1IF == HIGH)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.identifier == ECU_STATE_ABS) {
                if (msg.RTR == 1){
                    led1 = 1;
                }
            }
            if (msg.identifier == DISTANCE_SET) {
                distance_set_value = msg.data[0];
                distance_set_counter_1 = 0;
                distance_set_counter_2 = 0;
                distance_set_flag = HIGH;
            }
        }
        PIR3bits.RXB0IF = LOW;
        PIR3bits.RXB1IF = LOW;
    }

void main(void){
    board_initialization();
    while (1){
        }
    }
    


void board_initialization(void) {
    //Inputs and Outputs Configuration
    LATA = 0x00;
    TRISA = 0b00011111; // X-Axis / Y-Axis / 3P Switch
    LATB = 0x00;
    TRISB = 0b11111011; //CAN BUS / ON-OFF SWITCH
    LATC = 0x00;
    TRISC = 0b11110000; //USART Tx and Rx / LCD
    LATD = 0x00;
    TRISD = 0x00; //LCD / Backlight ON/OFF
    LATE = 0x00;
    TRISE = 0x00;

    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON); //Canbus 125kHz (da cambiare)


    //Interrupt Flags
    PIR2bits.TMR3IF = LOW;
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0

    //Interrupts Priority
    RCONbits.IPEN = HIGH; //abilita priorità interrupt
    IPR3bits.RXB1IP = HIGH; //interrupt alta priorità per can
    IPR3bits.RXB0IP = HIGH; //interrupt alta priorità per can
    IPR2bits.TMR3IP = LOW; // interrupt bassa priorità TMR3

    //Configurazione ADC======================================================
    ADCON1 = 0b00001101; //RA0 and RA1 analogic, AVdd and AVss voltage reference (?)
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;
    ADCON2bits.ACQT2 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON2bits.ADFM = 0; //Left Justified
    ADCON0bits.ADON = HIGH;
    //========================================================================
    // LCD Initialize
    LCD_initialize(16);
    //LCD_backlight(0);
    LCD_clear();
    LCD_goto_line(1);
    //LCD_write_message("Wait...");
    //delay_ms(300);

    PORTDbits.RD2 = 0;
    PORTDbits.RD3 = 0;

    //Configurations
    TMR3H = 0x63;
    TMR3L = 0xC0;

    //Interrupts Enables
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    PIE2bits.TMR3IE = HIGH;


    T3CON = 0x01; //Timer Enable
    LCD_clear();
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;
}