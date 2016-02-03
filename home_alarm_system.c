/* Project 4 Home Alarm System
 * Authors: Ben Mazur, Alex Addeo
 * Inputs: Keypad, microphone
 * Outputs: 2 Seven segment displays
 */

#include <p32xxxx.h>
#include <plib.h>
#include <stdlib.h>
#include "dsplib_dsp.h"
#include "fftc.h"

/* SYSCLK = 8MHz Crystal/ FPLLIDIV * FPLLMUL/ FPLLODIV = 80MHz
PBCLK = SYSCLK /FPBDIV = 80MHz*/
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

#define DEBOUNCE_TIME 5

/* Input array with 16-bit complex fixed-point twiddle factors.
 this is for 16-point FFT. For other configurations, for example 32 point FFT,
 Change it to fft16c32 */
#define fftc fft16c1024

/* defines the sample frequency*/
#define SAMPLE_FREQ 2048

/* number of FFT points (must be power of 2) */
#define N 1024

// Seven Segment Display JA and JB Jumpers
// Segments of the display
#define SegA_R LATEbits.LATE0
#define SegB_R LATEbits.LATE1
#define SegC_R LATEbits.LATE2
#define SegD_R LATEbits.LATE3
#define SegE_R LATGbits.LATG9
#define SegF_R LATGbits.LATG8
#define SegG_R LATGbits.LATG7
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel_R LATGbits.LATG6

// Seven Segment Display JC and JB jumpers
#define SegA_L LATGbits.LATG12
#define SegB_L LATGbits.LATG13
#define SegC_L LATGbits.LATG14
#define SegD_L LATGbits.LATG15
#define SegE_L LATDbits.LATD7
#define SegF_L LATDbits.LATD1
#define SegG_L LATDbits.LATD9
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel_L LATCbits.LATC1

//Keypad with The J port
#define C4 PORTBbits.RB0
#define C3 PORTBbits.RB1
#define C2 PORTBbits.RB2
#define C1 PORTBbits.RB3
#define R4 LATBbits.LATB4
#define R3 LATBbits.LATB5
#define R2 LATBbits.LATB8
#define R1 LATBbits.LATB9


int off = 0;    //variable that switches on and off when ssd's need to flash in alarm mode

int left_display = 1;   //used in the timer to flash back and forth between left and right ssd's quickly

int i;

int key_listener = 1;   //are we listening for keypress
int was_key_pressed = 0;
int was_key_released = 0;

int key_value;  //variable for which key is pressed

int key_val_stored;     //stores value of keypress

int button_lock = 0;    //prevents holding a button from changing states

int passcode=0;
int guess=0;
int holdLength=0;   //length of time a button is held for

int timer_started =0;   //used in mode 3 to go to mode 4 if correct password was not entered in 5 seconds
int samp = 0;

enum Mode {setup, unlocked, locked, alarm};
enum Mode mode = setup;

int timer_counter = 0;  //counts timer interrupt iterations
int seconds = 0;    //counts seconds

int samples = 0;    //counts samples taken

/* log2(1024)=10 */
int log2N = 10;


int16c sampleBuffer[N];

/* intermediate array */
int16c scratch[N];

/* intermediate array holds computed FFT until transmission*/
int16c dout[N];

/* intermediate array holds computed single side FFT */
int singleSidedFFT[N];

/* array that stores the corresponding frequency of each point in frequency domain*/
short freqVector[N];

/* indicates the dominant frequency */
int freq = 0;

// Function definitions
int computeFFT(int16c sampleBuffer[]);

unsigned char SSD_number[] = {
    0b0111111, //0
    0b0000110, //1
    0b1011011, //2
    0b1001111, //3
    0b1100110, //4
    0b1101101, //5
    0b1111101, //6
    0b0000111, //7
    0b1111111, //8
    0b1101111, //9
    0b1110111, //A
    0b1111100, //B
    0b0111001, //C
    0b1011110, //D
    0b1111001, //E
    0b1110001, //F
    0b1110110, //H
    0b0111000, //L
    0b0111110, //U
    0b0000000 //clear
};


//Timer 5 is used for taking in microphone data and performing the fft f

void __ISR(_TIMER_5_VECTOR, ipl5) _T5Interrupt(void)
{
    sampleBuffer[samples].re = readADC();   //get in real value of ADC
    samples++;  //increment number of samples taken

    if (samples == 1024)    //we have our desired number of samples so compute fft
    {
        freq = freqVector[computeFFT(&sampleBuffer)];

        samples = 0;
    }


    IFS0CLR = 0x100000; // Clear Timer5 interrupt status flag 
}

//Timer 2+3 for the SSD's and triggers from a certain length of time. 80 Hz.
//Note that this uses interrupt flag for Timer 3

void __ISR(_TIMER_3_VECTOR, ipl6) _T3Interrupt(void)
{
    left_display = !(left_display); //so SSD flashes quickly between left and right so we see both displays
    timer_counter++;
    //Then we need to set up counting such that every 80th interrupt it increases the seconds timer:
    if (timer_counter % 80 == 0)    //80 interrupts have happened so a second has passed because timer is 80Hz
    {       
        off = !off; //affects mode 4 where flashing the "AAAA" at 1Hz
        seconds++;
        timer_counter = 0;
    }
    if (mode == setup)
    {
        displaySSD(passcode,5);
    }
    else if (mode == unlocked)
    {
        displaySSD(0,18);
    }
    else if (mode == locked)
    {
        displaySSD(guess,17);
    }

    else if (mode == alarm)
    {
        displaySSD(0,4);
    }
    IFS0bits.T3IF = 0; // Clear Timer4 interrupt status flag
}


//Change Notice interrupt ISR for keypad
void __ISR(_CHANGE_NOTICE_VECTOR, ipl4) ChangeNotice_Handler(void)
{
    
    for(i=0;i<DEBOUNCE_TIME;i++);   //ignore bouncing signals
    

    key_value = 16;

    R1 = 0;
    R2 = R3 = R4 = 1;

    if (C1 == 0)
    {
        key_value = 1;
    }
    if (C2 == 0)
    {
        key_value = 2;
    }
    if (C3 == 0)
    {
        key_value = 3;
    }
    if (C4 == 0)
    {
        key_value = 10; // A
    }

    R2 = 0;
    R1 = R3 = R4 = 1;

    if (C1 == 0)
    {
        key_value = 4;
    }
    if (C2 == 0)
    {
        key_value = 5;
    }
    if (C3 == 0)
    {
        key_value = 6;
    }
    if (C4 == 0)
    {
        key_value = 11; //B
    }

    R3 = 0;
    R1 = R2 = R4 = 1;

    if (C1 == 0)
    {
        key_value = 7;
    }
    if (C2 == 0)
    {
        key_value = 8;
    }
    if (C3 == 0)
    {
        key_value = 9;
    }
    if (C4 == 0)
    {
        key_value = 12; //C
    }
    R4 = 0;
    R1 = R3 = R2 = 1;

    if (C1 == 0)
    {
        key_value = 0;
    }
    if (C2 == 0)
    {
        key_value = 15; //F
    }
    if (C3 == 0)
    {
        key_value = 14; //E
    }
    if (C4 == 0)
    {
        key_value = 13; //D
    }

    if (key_value != 16 && !(button_lock))  //if a key is pressed turn on buttonlock
    {
        button_lock=1;
        seconds = 0;    //we start looking at how long the button is held
        holdLength=0;
        timer_counter = 0;
        key_val_stored = key_value; //store pressed key value
        was_key_pressed=1;  //mark that key was pressed
        key_listener = 1; //mark that there must be an action from key press
    }
    if (key_value == 16 && button_lock) // If key release turn off buttonlock
    {
        button_lock=0;       
        holdLength = seconds;
        key_listener = 1;
        was_key_released = 1;       
    }
    R1 = R2 = R3 = R4 = 0;
    PORTB;
    
    IFS1CLR = 0x0001; //Clears interrupt flag
   
}


int readADC()
{
    AD1CON1bits.SAMP = 1; 
    for (TMR1=0; TMR1<100; TMR1++); 
    AD1CON1bits.SAMP = 0; 
    while (!AD1CON1bits.DONE); 
    return ADC1BUF0; 
}



void displayDigit(unsigned char value, unsigned int left_ssd, unsigned int leftdig)
{
   
    if (left_ssd == 1) // if left ssd
    {
        if (leftdig == 1) // if left dig
        {
            SegA_L = value & 1;
            SegB_L = (value >> 1) & 1;
            SegC_L = (value >> 2) & 1;
            SegD_L = (value >> 3) & 1;
            SegE_L = (value >> 4) & 1;
            SegF_L = (value >> 5) & 1;
            SegG_L = (value >> 6) & 1;
            DispSel_L = 0;
        } 
        else
        {
            SegA_L = value & 1;
            SegB_L = (value >> 1) & 1;
            SegC_L = (value >> 2) & 1;
            SegD_L = (value >> 3) & 1;
            SegE_L = (value >> 4) & 1;
            SegF_L = (value >> 5) & 1;
            SegG_L = (value >> 6) & 1;
            DispSel_L = 1;
        }

    } 
    else
    {
        if (leftdig == 1)
        {
            SegA_R = value & 1;
            SegB_R = (value >> 1) & 1;
            SegC_R = (value >> 2) & 1;
            SegD_R = (value >> 3) & 1;
            SegE_R = (value >> 4) & 1;
            SegF_R = (value >> 5) & 1;
            SegG_R = (value >> 6) & 1;
            DispSel_R = 0;
        } 
        else
        {
            SegA_R = value & 1;
            SegB_R = (value >> 1) & 1;
            SegC_R = (value >> 2) & 1;
            SegD_R = (value >> 3) & 1;
            SegE_R = (value >> 4) & 1;
            SegF_R = (value >> 5) & 1;
            SegG_R = (value >> 6) & 1;
            DispSel_R = 1;
        }

    }
}

void resetDisplay() //reset display
{
    displayDigit(0b0000000, 0, 0);
    displayDigit(0b0000000, 1, 0);
    displayDigit(0b0000000, 0, 1);
    displayDigit(0b0000000, 1, 1);
}


void showNumber(int digit, unsigned int left_ssd_num, unsigned int leftdisp)
{
    displayDigit(SSD_number[digit % 20], left_ssd_num, leftdisp);
}

void displaySSD(int input, int mode_input)
{
    if(input==1000) //if it's 1000, clear it basically
    {
        input=10;
    }
    if((mode == setup) || (mode == locked)) // show input in these modes
    {
        if (left_display)
        {
            showNumber(input % 10, 0, 1);
            showNumber((input / 100) % 10, 1, 1);
        }
        else
        {
            showNumber((input / 10) % 10, 0, 0);
            showNumber(mode_input, 1, 0);
        }
    }
    if(mode == unlocked)//only show a U
    {
        showNumber(18, 1, 0);
    }
    if(mode == alarm) //Flash AAAA at 1Hz
    {
        if(off) // constantly switching every second so it flashes
        {
            if (left_display)
            {
                showNumber(10, 0, 1);
                showNumber(10, 1, 1);
            }
            else
            {
                showNumber(10, 0, 0);
                showNumber(10, 1, 0);
            }
        }
        else{
            resetDisplay();
        }
    }
}

main() {


    INTDisableInterrupts();


    //ADC manual config
    AD1PCFG = 0xF7FF; // all PORTB = digital but RB7 = analog
    AD1CON1 = 0; // manual conversion sequence control
    AD1CHS = 0x000B0000; // Connect RB7/AN7 as CH0 input
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3 = 0x1F02; // Tad = 128 x Tpb, Sample time = 31 Tad
    AD1CON1bits.ADON = 1; // turn on the ADC

    TRISB = 0x80F;

    //Configure ports C~G to be output ports
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISF = 0;
    TRISG = 0;
    // initialize B~G to 0
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    PORTF = 0x00;
    PORTG = 0x00;
    
    int i;

    
    // Configure Timer for the ADC sampling
    // This will be a type A timer
    T5CONbits.ON = 0; // Stop timer, clear control registers
    TMR5 = 0; // Timer counter
    PR5 = 0x9869; //Timer count amount for interupt to occur - 2048Hz frequency
    IPC5bits.T5IP = 5; //prioirty 5
    IFS0bits.T5IF = 0; // clear interrupt flag
    T5CONbits.TCKPS = 0; // prescaler at 1:256, internal clock sourc
    T5CONbits.ON = 1; // Timer 5 module is enabled
    IEC0bits.T5IE = 1; //enable Timer 5


    //Configure Timer for the SSD display and 1 and 5 second timers respectively
    //This will be a Type B timer of TMR 2 and TMR 3
    T2CONbits.ON = 0; //Turn Timer 2 off
    T3CONbits.ON = 0; //Turn Timer 3 off
    T2CONbits.T32 = 1; //Enable 32 bit mode
    T3CONbits.TCKPS = 0; //Select prescaler = 1
    TMR2 = 0; //Clear Timer 2 register
    T3CONbits.TCS = 0; //Select internal clock as source
    PR2 = 1000000; //Load period Register - 80Hz frequency
    IFS0bits.T3IF = 0; //Clear Timer 3 interupt flag
    IPC3bits.T3IP = 6; //Set priority level to 6
    IPC3bits.T3IS = 2;
    IPC2bits.T2IP = 6;
    IPC2bits.T2IS = 2;
    IEC0bits.T3IE = 1; // Enable Timer 2
    T2CONbits.ON = 1; //Turn Timer  2 on.



    //Configure Change Notice for the keypad
    // 1. Configure CNCON, CNEN, CNPUE
    //turn on Cn interrupts
    CNCON = 0x8000;
    //CN enable on pins 2,3,4,5
    CNEN = 0x003C;
    //enable weak pull up resistors
    CNPUE = 0x003C;

    
    PORTB;

    // 3. Configure IPC5, IFS1, IEC1
    
    IPC6bits.CNIP=4; // priorities need to be lower than those of the timer interrupts
    IPC6bits.CNIS=3;
    //Clear the Interrupt flag status bit
    IFS1CLR = 0x0001;
    //Enable Change Notice Interrupts
    IEC1SET = 0x0001;

    // 4. Enable vector interrupt
    INTEnableSystemMultiVectoredInt();
    //initialize rows to listen for inputs from keypad
    R1 = R2 = R3 = R4 = 0;



    //place values in the sample buffer
    for (i = 0; i < N; i++) {
        sampleBuffer[i].re = i;
        sampleBuffer[i].im = 0;
    }
    //convert time domain to freq domain of data points
    for (i = 0; i < N / 2; i++) {
        freqVector[i] = i * (SAMPLE_FREQ / 2) / ((N / 2) - 1);
    }

    while (1) {
    //Next state logic

        if(mode == setup)
        {
            //Digits enter passcode, C clears, D deletes, E enters if valid
            if (key_listener)
            {
                if(was_key_pressed)
                {
                    was_key_pressed=0;
                    key_listener = 0;
                }
                else if(was_key_released)
                {
                    if (key_val_stored>=0 && key_val_stored<=9) // if digit
                        {
                            if (passcode == 0)
                            {
                                passcode = key_val_stored;
                            } else if (passcode > 0 && passcode < 10) //handling multiple digits
                            {
                                passcode = (10 * passcode) + key_val_stored;
                            }
                            else if (passcode >=10 && passcode <100)
                            {
                                passcode = (10 * passcode) + key_val_stored;
                            }
                        }

                    else //must be a letter
                    {

                        if (key_val_stored == 13) // Delete
                        {
                            passcode = (passcode - (passcode % 10)) / 10;
                        }
                        if ( key_val_stored == 12) //Clear
                        {
                            passcode = 0;
                        }
                        if (key_val_stored == 14) // Enter
                        {
                            if (passcode >= 300 && passcode <= 999)
                            {
                                guess = 0;
                                mode = unlocked;
                                resetDisplay();
                            }
                        }
                    }
                    was_key_released=0;
                    key_listener = 0;
                }
            }
        }
        if(mode == unlocked)
        {
            
            if (key_listener)
            {
                if (was_key_released)
                {
                    if(holdLength<1) // if hold for less than a second go to locked
                    {
                        mode = locked;
                        key_listener=0;
                        was_key_released=0;
                        guess = 0;
                    }
                    if(holdLength >= 1) // if hold for more than or equal to second go to setup
                    {
                        mode = setup;
                        key_listener=0;
                        was_key_released=0;
                        guess = 0;
                    }
                }
                else
                {
                    key_listener=0;
                    was_key_pressed=0;
                }
            }
        }
        if(mode == locked)
        {
            if(readADC() > 200 && freq > 300)
            {
                if(freq <= (passcode*1.03) && freq >= (passcode*.97))   //correct frequency go to unlocked
                {
                    mode = unlocked;
                    resetDisplay();
                }
                else
                {   //wrong frequency go to alarm
                    mode = alarm;
                    timer_counter = 0;
                    seconds = 0;
                }
            }
            if (readADC() > 450)
            {   //loud noise set off alarm
                mode = alarm;
                timer_counter = 0;
                seconds = 0;
            }

            
            if (key_listener)
            {
            
                if(was_key_released)
                {
                    timer_counter = 0;
                    timer_started = 1;
                    if (key_val_stored>=0 &&key_val_stored<=9)
                    {
                        if (guess == 0)
                        {
                            guess = key_val_stored; 
                        } 
                        else if (guess > 0 && guess < 10)
                        {
                            guess = (10 * guess) + key_val_stored; 
                        }
                        else if (guess >=10 && guess <100)
                        {
                            guess = (10 * guess) + key_val_stored; 
                        }
                    }
                    else 
                    {

                        if (key_val_stored == 13) 
                        {
                            guess = (guess - (guess % 10)) / 10;
                        }
                        if (key_val_stored == 12) 
                        {
                            guess = 0;
                        }
                        if (key_val_stored == 14) 
                        {
                            if (guess == passcode)
                            {
                                mode = unlocked;
                                resetDisplay();
                            }
                            else
                            {
                                mode = alarm;
                                timer_counter = 0;
                                seconds = 0;
                            }
                        }
                    }
                    key_listener = 0;
                    was_key_released = 0;
                    }
                    else if(was_key_pressed)
                    {
                        was_key_pressed=0;
                        key_listener=0;
                    }

                }
                if(timer_started && seconds == 5)
                {
                        mode = alarm;
                        seconds = 0;
                        timer_started = !timer_started;

                }
        }
        if(mode == alarm)
        {
            
            if(seconds==5)// if ssd's have been flashing for 5 seconds we should go back to locked mode
            {
                mode = locked;
                guess = 0;
                timer_counter = 0;
                seconds = 0;
            }
        }

    }
}

int computeFFT(int16c *sampleBuffer)
{
    int i;
    int dominant_freq = 1;

    
    mips_fft16(dout, sampleBuffer, fftc, scratch, log2N);

    /* compute single sided fft */
    for (i = 0; i < N / 2; i++)
    {
        singleSidedFFT[i] = 2 * ((dout[i].re * dout[i].re) + (dout[i].im * dout[i].im));
    }

    /* find the index of dominant frequency, which is the index of the largest data points */
    for (i = 1; i < N / 2; i++)
    {
        if (singleSidedFFT[dominant_freq] < singleSidedFFT[i])
        {
            dominant_freq = i;
        }
    }

    return dominant_freq;
}