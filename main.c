
#include <lpc17xx.h>
#include <stdio.h>

// --- Pin Definitions ---
#define TRIG       (1 << 0)     // P0.0
#define ECHO       (1 << 1)     // P0.1
#define SEGMENT    (0x00000FF0) // P0.4 to P0.11
#define SEG_CTRL   (1 << 19)    // P0.19

#define BUZZER     (0x03000000)    // P3.24

char uart_buffer[100];
unsigned int i, j, k;
unsigned int car_count = 0;
// --- Segment Display Lookup Table ---
const unsigned int seg_disp[16] = {
    0x000003F0, 0x00000060, 0x000005B0, 0x000004F0,
    0x00000660, 0x000006D0, 0x000007D0, 0x00000070,
    0x000007F0, 0x000006F0, 0x00000770, 0x000007C0,
    0x00000390, 0x000005E0, 0x00000790, 0x00000710
};

unsigned char int3_flag=0;
void EINT3_IRQHandler(void);
void external_int_init(void);

void seven_seg_init(void);
void display_count(unsigned int cnt);

void UART0_Init(void);
void UART0_SendChar(char c);
void UART0_SendString(const char *str);

void buzzer_init(void);
void activateBuzzer(void);
void activateBuzzerVIP(void);

void delayUS(unsigned int us);
void delayMS(unsigned int ms);

void ultrasonic_init(void);
float measureDistance(void);

void startTimer0(void);
unsigned int stopTimer0(void);

void stepper_init(void);
void stepper_clockwise(void);
void stepper_anticlockwise(void);


int main(){

    UART0_Init();
    buzzer_init();
    ultrasonic_init();
    stepper_init();
    seven_seg_init();
    external_int_init();
    UART0_SendString("System Initialized\n\r\n");
    while(1){
        float distance = measureDistance();
        sprintf(uart_buffer, "Distance = %.2f cm\n\r\n", distance);
        UART0_SendString(uart_buffer);
        NVIC_EnableIRQ(EINT3_IRQn);    

        if(distance <= 15.0){
            activateBuzzer();
            for(j=0;j<10;j++)
                stepper_clockwise();
            
            for(k=0;k<65000;k++); 
            display_count(++car_count);

            sprintf(uart_buffer, "Car Count = %u\n\r\n", car_count);
            UART0_SendString(uart_buffer);

            for(j=0; j<10; j++)
                stepper_anticlockwise();

            for(k=0;k<65000;k++);  
        }
        delayMS(2000);
    }
    return 0;
}

// --- Stepper Motor Control ---
void stepper_init(void) {
    LPC_GPIO2->FIODIR = 0x0F;
}

void stepper_clockwise(void) {
    unsigned long pattern = 0x1;
    for (i = 0; i < 4; i++) {
        LPC_GPIO2->FIOCLR = 0x0F;
        LPC_GPIO2->FIOSET = pattern;
        pattern <<= 1;
        for(k=0;k<15000;k++);     
    }
}


void stepper_anticlockwise(void) {
    unsigned long pattern = 0x8;
    for (i = 0; i < 4; i++) {
        LPC_GPIO2->FIOCLR = 0x0F;
        LPC_GPIO2->FIOSET = pattern;
        pattern >>= 1;
        for(k=0;k<15000;k++); 	
    }
}

void startTimer0(void) {
    LPC_TIM0->TCR = 0x02;
    LPC_TIM0->TCR = 0x01;
}

unsigned int stopTimer0(void) {
    LPC_TIM0->TCR = 0x00;
    return LPC_TIM0->TC;
}

// --- Ultrasonic Sensor Functions ---
void ultrasonic_init(void) {
    LPC_GPIO0->FIODIR |= TRIG;
    LPC_GPIO0->FIODIR &= ~ECHO;
    LPC_GPIO0->FIOCLR |= TRIG;
}

float measureDistance(void) {
    unsigned int echoTime;
    LPC_GPIO0->FIOPIN |= TRIG;
    delayUS(10);
    LPC_GPIO0->FIOCLR |= TRIG;

    while (!(LPC_GPIO0->FIOPIN & ECHO));
    startTimer0();
    while (LPC_GPIO0->FIOPIN & ECHO);
    echoTime = stopTimer0();

    return (0.0343 * echoTime) / 2.0;
}

void EINT3_IRQHandler(void)
{
    LPC_SC->EXTINT = 0x00000008;
    UART0_SendString("VIP Vehicle Detected\n\r\n");
    for(j=0; j<10; j++) 
        stepper_clockwise();
    activateBuzzerVIP();

    for(j=0; j<10; j++)
        stepper_anticlockwise();
}


void external_int_init(void){
    LPC_PINCON->PINSEL4 |= 0x04000000;     //P2.13 as EINT3
    LPC_PINCON->PINSEL4 &= 0xFCFFFFFF;     //P2.12 GPIO for LED
    LPC_SC->EXTINT = 0x00000008;           //clears the interrupt
    LPC_SC->EXTMODE = 0x00000008;          //EINT3 edge sensitive
}

// --- Delay Functions ---
void delayUS(unsigned int us) {
    LPC_TIM0->TCR = 0x02;
    LPC_TIM0->TCR = 0x01;
    while (LPC_TIM0->TC < us);
    LPC_TIM0->TCR = 0x00;
}

void delayMS(unsigned int ms) {
    delayUS(ms * 1000);
}

// --- Buzzer Control ---
void buzzer_init(void) {
    LPC_GPIO0->FIODIR = BUZZER;    
}

void activateBuzzer(void) {
    LPC_GPIO0->FIOSET = BUZZER;
    delayMS(100); 
    LPC_GPIO0->FIOCLR = BUZZER;
}


void activateBuzzerVIP(void) {
    LPC_GPIO0->FIOSET = BUZZER;
    delayMS(5000);  // Buzzer active for 5 seconds
    LPC_GPIO0->FIOCLR = BUZZER;
}

// --- UART Functions ---
void UART0_Init(void) {
    LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6);
    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 12;
    LPC_UART0->DLM = 0;
    LPC_UART0->FCR |= 0x07;
    LPC_UART0->FDR = (15 << 4) | 2;
    LPC_UART0->LCR = 0x03;
}

void UART0_SendChar(char c) {
    while (!(LPC_UART0->LSR & 0x20));
    LPC_UART0->THR = c;
}

void UART0_SendString(const char *str) {
    while (*str) {
        UART0_SendChar(*str++);
    }
}

// --- Seven Segment Display ---
void seven_seg_init(void) {
    LPC_GPIO0->FIODIR |= (SEG_CTRL | SEGMENT);
}

void display_count(unsigned int cnt) {
    LPC_GPIO0->FIOSET = SEG_CTRL;
    LPC_GPIO0->FIOCLR = SEGMENT;
    LPC_GPIO0->FIOSET = seg_disp[cnt % 16];
}
