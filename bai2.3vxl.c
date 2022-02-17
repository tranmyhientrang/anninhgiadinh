#include <xc.h>
#include <stdint.h>
//#include <config.h>

#include <pic16f877a.h>
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program M
#define PIR RB0

#define _XTAL_FREQ 16000000
#define SIM900_OK 1
#define SIM900_READY 2
#define SIM900_FAIL 3
#define SIM900_RING 4
#define SIM900_NC 5
#define SIM900_UNLINK 6

inline unsigned char _SIM900_waitResponse(void);
    int recv;
    char p =1;
 
void UARTTx_Init();
void ADC_Init();
void read_ldr();
void testpass();
unsigned int ADC_Read(unsigned char channel);
void UART_Send(uint8_t data);
void testpir();
void testwater();
uint8_t counter = 0;
uint8_t pass[4] = {0, 0, 0, 0};
int dem=0;
int count=0;
int count1=0;
void main()
{
    TRISA = 0xFF;
TRISC6=TRISC7=1;
TRISB=0xff;
TRISD=0x00;
PORTD=0x00;
    TMR1IE = 1;                 // Timer1 Interrupt enable bit
    TMR1IF = 0;                 // Clear the Interrupt flag bit for timer1        
    PEIE = 1;                   // Peripherals Interrupts enable bit 
    GIE = 1;                    // Global Interrupt Enable bit
    
    // Config Timer1
    // Clear the Timer1 register to start counting from 0
    TMR1 = 0;                   
    // Clear the Timer1 clock select bit to choose local clock source
    TMR1CS = 0;                 
    // Prescaler ratio 1:1
    T1CKPS0 = 0;
    T1CKPS1 = 0;
    
    // Switch ON Timer1
    TMR1ON = 1;
INTEDG = 1;
RBIE = 1;
GIE = 1;  
ADC_Init();

while (1)
{

testpir();
testpass();

read_ldr();
testwater();
//UARTTx_Init();

}
}

void __interrupt() ISR(void){
     if(RBIF == 1) {
        if(RB5 == 0 ) {
            pass[counter] = 1;
            counter++;
        }
        
      
        if(RB6 == 0) {
            pass[counter] = 3;
            counter++;
        }
        
      
        RBIF = 0;
    }    
     if(TMR1IF==1)
     {
         if(RB1==1)
         {
             RD7=1;
             count++;
             if(count>=30)
             {
                 RD1=1;
                 RD7=0;
                 
                
             }
         }
     }
     TMR1IF=0;
     
}
void testpass(){
    
    
     if(counter == 4) {
            if(pass[0] == 3 && pass[1] == 3
               && pass[2] == 3 && pass[3] == 3) {
                RD4=0;
                RD5=1;
                RD3=1;
                counter = 0;
                
            } else {
                dem++;
                if(dem >= 2){
                    RD5=0;
                RD4=1;
            }
                counter = 0;
            }
        }
    
}
int light;
void ADC_Init()
{
  ADCON0 = 0x81;               //Turn ON ADC and Clock Selection
  ADCON1 = 0x00;               //All pins as Analog Input and setting Reference Voltages
}
unsigned int ADC_Read(unsigned char channel)
{
  if(channel > 7)              //Channel range is 0 ~ 7
    return 0;

  ADCON0 &= 0xC5;              //Clearing channel selection bits
  ADCON0 |= channel<<3;        //Setting channel selection bits
  __delay_ms(2);               //Acquisition time to charge hold capacitor
  GO_nDONE = 1;                //Initializes A/D conversion
  while(GO_nDONE !=0);             //Waiting for conversion to complete
  return ((ADRESH<<8)+ADRESL); //Return result
}
void read_ldr()
{
unsigned int adc_value=0;
adc_value= ADC_Read(0);
light = 100 - adc_value/(1024);
if(light<=50) // SWITCH of the light when light is 80 percent
{
RD2=1;

}
else
{
RD2=0;

}
} 
void testpir(){
    if(PIR==1)
    {
        RD0=1;
    }
    else
    {
        RD0=0;
    }
    }
void testwater(){
    if(RB1 == 0){
        count = 0;
        RD1=0;
        RD7=0;
        
    }
    
}  
    
    
    

void UARTTx_Init(){
    // Baud rate configuration
    BRGH = 1;
    SPBRG = 129;
    // Enable Asynchronous Serial Port
    SYNC = 0;
    SPEN = 1;
    // Configure Rx-Tx pin for UART 
    TRISC6 = 0;
    TRISC7 = 1;
    // Enable UART Transmission
    TXEN = 1;// cho phep truyen UART, TRUY?N du lieu
    CREN  = 1;
    TX9   = 0;    // 8-bit reception selected
    RX9   = 0;    // 8-bit reception mode selected
}
void _SIM900_putch(char bt) 
{
    while(!TXIF);  // hold the program till TX buffer is free
    TXREG = bt; //Load the transmitter buffer with the received value
}
char _SIM900_getch()  
{
    if(OERR) // check for Error
    {
        CREN = 0; //If error -> Reset
        CREN = 1; //If error -> Reset
    }
   while(!RCIF);  // hold the program till RX buffer is free
    return RCREG; //receive the value and send it to main function
}
void SIM900_send_string(char* st_pt)
{
    while(*st_pt) //if there is a char
        _SIM900_putch(*st_pt++); //process it as a byte data
}
void _SIM900_print(unsigned const char *ptr) {
    while (*ptr != 0) {
        _SIM900_putch(*ptr++);
    }
}