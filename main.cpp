#include "mbed.h"
#include "SDFileSystem.h"

#include "bits.h"
#include "types.h"
#include "sequences.h"
#include "dim_steps.h"

// #define FT_FT_DEBUG 0
#ifdef FT_FT_DEBUG
#warning "Using debug mode."
#endif

// #define USB_TEST
#ifdef USB_TEST
#warning "Using USB test mode."
#endif


/* Serial debug port. */
Serial pc(P1_13, P1_14); // tx, rx

DigitalOut Test_RXD(P1_26);
DigitalOut Test_TXD(P1_27);

InterruptIn int_ZCD(P0_2);
Ticker tkr_Timer;
Ticker tkr_FastInt;

/* Determines the fastest and slowest sequence step timing. Times are in
    1/60th of a second (one clock).*/
#define FASTEST_TIME 10.0
#define SLOWEST_TIME 300.0
/* The AnalogIn function scales the voltage input to a float 0.0-1.0. */
#define SLOPE (SLOWEST_TIME - FASTEST_TIME)

/* These coefficients are used to convert the potentiometer input to a 
exponetial curve that mimics the desired response. */
#define A_COEFF 0.0207
#define B_COEFF 3.9
#define C_COEFF -0.0207

#define MAX_SLICE 250   // how many slices to allow in a half AC cycle
#define HALF_CYCLE 8333     // usec for one half cycle of 60Hz power
#define SLICE 65       // usec for slices of a half AC cycle
#define DURATION_OF_ISR 300            // how long it takes the ZCD_SD ISR to execute

/* The potentiometer input port to select the speed of the sequence steps. */
AnalogIn potentiometer(P0_11);

/* Setup the output pins. */
DigitalOut C0(P0_16);
DigitalOut C1(P0_20);
DigitalOut C2(P0_17);
DigitalOut C3(P0_21);
DigitalOut C4(P0_18);
DigitalOut C5(P0_22);
DigitalOut C6(P0_19);
DigitalOut C7(P0_23);

BusOut lights(P0_23, P0_19, P0_22, P0_18, P0_21, P0_17, P0_20, P0_16);

/* Setup the dipswitch input port. */
BusInOut dipswitch(P1_23, P0_12, P0_13, P0_14, P0_7, P0_8, P0_9, P1_24);
DigitalInOut master_slave(P0_4);
DigitalInOut test(P0_5);

/* Setup the reset switch as an input to keep it from being a reset */
DigitalInOut reset(P0_0);

/* Setup the SD card detect input */
DigitalInOut sd_present(P1_15);

float speed;            /* The selected speed for chases. */
word dimmer_speed = 1;      /* The selected speed for dimming */
int speed_clks;         /* speed in clocks (1/60th sec). */
int clocks = 1;             /* Incremented everytime the zero cross interrupt is called. */
int total_clocks_per_step = 1;
byte pattern;           /* The current output pattern. */
byte *ptrSequence;      /* A pointer to the desired sequence. */

word sequenceLength;    /* The length of the desired sequence. */
word step;              /* The step in the current sequence. */
char line[100];
byte master_sequence; 
byte C = 0;
byte R = 0;
byte Z = 0;
byte MASTER = 0;        // assume slave unless master is enabled

float old_pot, new_pot;

sDimStep *ptrDimSequence;
sDimStep *ptrDimSeq = NULL;
unsigned int DimSeqLen;

byte ticks = 1;
byte zc_slice = 0;

/* The dimmer timers for each channel. */
byte Dimmer[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void master_timer_isr (void);
void slave_timer_isr(void);
void slice_timer_isr(void);
void master_zcross_isr(void);
void slave_zcross_isr(void);
void vfnLoadSequencesFromSD(byte);
void vfnGetLine(void);
void vfnSlaveReceiveData(byte);


void master_timer_isr(void) {
    // as the master running a chase sequence from internal flash, execute this every time the step timer expires
    // to make steps longer in this mode, simply add duplicate channel bitmaps.  tick means nothing here.
    clocks++;
    if(clocks > speed_clks) {
        clocks = 0;
        step++;
        if(step >= sequenceLength) {
            step = 0;
            R = 1;
        }
        else {
            Z = 1;
        }
        pattern = ~ptrSequence[step];
        lights = pattern;  
    }
}

void slave_timer_isr(void) {
    // as a slave running a chase sequence from internal flash, execute these sync instructions every time the step timer expires
    // to make steps longer in this mode, simply add duplicate channel bitmaps.  tick means nothing here.
    // R = restart, Z = step to next
    if (R) {
        step = 0;
    }
    else if (Z) {
        step++;
    }
    
    if ((R == 1) or (Z == 1)) {
        pattern = ~ptrSequence[step];
        lights = pattern;

        R = 0;
        Z = 0;      
    }
}

void slice_timer_isr(void) {
    // while in dimmer mode, execute this routine every delta-T slice to evaluate whether to active a channel

    if (zc_slice > 240) {                              // if nearing the end of a full AC cycle, reset everything for the next cycle
        lights = 0xFF;                                 // C0-C7 all off (but they'll stay on until the ZC occurs)
        zc_slice = 0;                                  // clear the slice counter for the next half cycle
        tkr_FastInt.detach();                          // disable this timer interrupt
        
        if (MASTER) {
            int_ZCD.fall(&master_zcross_isr);      // enable the zero crossing interrupt since we're done dimming for this half cycle
        } 
        else {
            int_ZCD.fall(&slave_zcross_isr);
        }
        return;
    }
    
    zc_slice++;
    
    if (Dimmer[0] != 0) Dimmer[0]--; else C0 = 0;
    if (Dimmer[1] != 0) Dimmer[1]--; else C1 = 0;
    if (Dimmer[2] != 0) Dimmer[2]--; else C2 = 0;
    if (Dimmer[3] != 0) Dimmer[3]--; else C3 = 0;
    if (Dimmer[4] != 0) Dimmer[4]--; else C4 = 0;
    if (Dimmer[5] != 0) Dimmer[5]--; else C5 = 0;
    if (Dimmer[6] != 0) Dimmer[6]--; else C6 = 0;
    if (Dimmer[7] != 0) Dimmer[7]--; else C7 = 0;    
}

void master_zcross_isr(void) {
    // as the master running a dimmer sequence loaded from the SD card, execute this every time a rising AC zero crossing occurs.
    int i;
    
    if (int_ZCD.read() == 0) {                     // the AC line just crossed to positive
        int_ZCD.fall(NULL);                        // disable the ZCD interrupt otherwise it will trigger on the negative edge also due to some bug. noise?
    }

    clocks--;                                      // a clock is a zero cross (1/60 second)
    
    if(clocks == 0) {                              // we need count until clocks rolls over to zero
        step++;
            
        if(step >= sequenceLength) {               // once we step past the end of a sequence, restart the sequence
            step = 0;
            R = 1;
        }
        else {
            Z = 1;
        }    

        total_clocks_per_step = dimmer_speed * ptrDimSequence[step].ticks;
        clocks = total_clocks_per_step;
    }
        
    for(i=0; i<8; i++) {
        Dimmer[i] = 255 - (ptrDimSequence[step].Chan[i].start + ((ptrDimSequence[step].Chan[i].stop - ptrDimSequence[step].Chan[i].start) * clocks) / total_clocks_per_step);
    }   
    
    /* Timer for the 255 step dimmer routine. */
    zc_slice = 0;
    tkr_FastInt.attach_us(&slice_timer_isr, SLICE);
}

void slave_zcross_isr(void) {
    // as a slave running a dimmer sequence receieved from the master, execute these sync instructions every time a rising AC zero crossing occurs
    int i;
    
    if (int_ZCD.read() == 0) {                     // the AC line just crossed to positive
        int_ZCD.fall(NULL);                        // disable the ZCD interrupt otherwise it will trigger on the negative edge also due to some bug. noise?
    }

    if (R) {
        step = 0;
    }
    else if (Z) {
        step++;
    }
    
    if (R or Z) {
        total_clocks_per_step = dimmer_speed * ptrDimSequence[step].ticks;
        clocks = total_clocks_per_step;
        R = 0;
        Z = 0;
    }
    
    if (clocks > 0) {
        clocks--;
    
        for(i=0; i<8; i++) {
            Dimmer[i] = 255 - (ptrDimSequence[step].Chan[i].start + ((ptrDimSequence[step].Chan[i].stop - ptrDimSequence[step].Chan[i].start) * clocks) / total_clocks_per_step);
        }   
    }
    /* Timer for the 255 step dimmer routine. */
    zc_slice = 0;
    tkr_FastInt.attach_us(&slice_timer_isr, SLICE);
}

void vfnLoadSequencesFromSD(byte sequence) {

    FILE *fp;
    int steps;
    sDimStep *ptr = NULL;
    SDFileSystem sd(P1_22, P1_21, P1_20, P1_19, "sd"); // the pinout on the FT33 controller
    unsigned int ticks;
    unsigned int ChanStart[8];
    unsigned int ChanStop[8];
    unsigned int i;
    unsigned int sequence_num;
    
    fp = fopen("/sd/seq.txt", "r");
    if(fp == NULL) {
        // if the SD card is present but not responding, reset and try again
        NVIC_SystemReset();
    }
    else {
        pc.printf("\n");    // in case the "No SD card found" message is sent, be sure to start with a fresh line buffer
        while(fgets(line, 100, fp) != NULL) {
            pc.printf(line);        // transmit to the slaves
            
            if(line[0] == 'Q') {
                sscanf(line, "%*s %d %d", &sequence_num, &steps);
                if(sequence_num == sequence) {
                    ptr = (sDimStep *) malloc(sizeof(sDimStep) * steps);
                    if (ptr == NULL) {
                        break;
                    }
                    ptrDimSeq = ptr;
                    DimSeqLen = steps;
                }
            }
            else if(line[0] == 'S') {
                if(sequence_num == sequence) {
                    sscanf(line, "%*s %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                            &ticks,
                            &ChanStart[0], &ChanStop[0],
                            &ChanStart[1], &ChanStop[1],
                            &ChanStart[2], &ChanStop[2],
                            &ChanStart[3], &ChanStop[3],
                            &ChanStart[4], &ChanStop[4],
                            &ChanStart[5], &ChanStop[5],
                            &ChanStart[6], &ChanStop[6],
                            &ChanStart[7], &ChanStop[7]);

                    ptr->ticks = (unsigned char)(ticks & 0x000000FF);
                
                    for (i = 0; i < 8; i++) {
                        ptr->Chan[i].start = (unsigned char)(ChanStart[i] & 0x000000FF);
                        ptr->Chan[i].stop  = (unsigned char)(ChanStop[i]  & 0x000000FF);
                    }
                    ptr++;
                }
            }  
        }
        fclose(fp);
    }
}

void vfnGetLine(void) {
    
    int num = 0;
    char c;

    while(((c = pc.getc()) != '\n') && num < 98) {
        line[num] = c;
        num++;
    }
    line[num] = 0x00;
}

void vfnSlaveReceiveData(byte sequence) {

    int steps;
    int step;
    sDimStep *ptr = NULL;
    unsigned int ticks;
    unsigned int ChanStart[8];
    unsigned int ChanStop[8];
    unsigned int i;
    unsigned int sequence_num;

    while(1) {
        vfnGetLine();
        
        if(line[0] == 'Q') {
            sscanf(line, "%*s %d %d", &sequence_num, &steps);
            if(sequence_num == sequence) {
                ptr = (sDimStep *) malloc(sizeof(sDimStep) * steps);
                if (ptr == NULL) {
                    break;
                }
                ptrDimSeq = ptr;
                DimSeqLen = steps;
                step = 0;
            }
        }
        else if(line[0] == 'S') {
            if(sequence_num == sequence) {
                sscanf(line, "%*s %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                        &ticks,
                        &ChanStart[0], &ChanStop[0],
                        &ChanStart[1], &ChanStop[1],
                        &ChanStart[2], &ChanStop[2],
                        &ChanStart[3], &ChanStop[3],
                        &ChanStart[4], &ChanStop[4],
                        &ChanStart[5], &ChanStop[5],
                        &ChanStart[6], &ChanStop[6],
                        &ChanStart[7], &ChanStop[7]);

                ptr->ticks = (unsigned char)(ticks & 0x000000FF);
            
                for (i = 0; i < 8; i++) {
                    ptr->Chan[i].start = (unsigned char)(ChanStart[i] & 0x000000FF);
                    ptr->Chan[i].stop  = (unsigned char)(ChanStop[i]  & 0x000000FF);
                }
                
                ptr++;                
                step++;
                
                if (step >= steps) {
                    break;
                }
            }
        }              
    }
}

int main() {

    byte sequence;
    byte sd;
    byte command_char;

    // Initialize the unused RAM to track heap usage
    for (uint32_t i = 0x10001200; i < 0x10002000; i++) {
        *(volatile uint8_t *)i = 0xCD;
    }

    // Initialize the unused USB RAM to track stack usage
    for (uint32_t i = 0x20004000; i < 0x20004700; i++) {
        *(volatile uint8_t *)i = 0xCD;
    }

    /* Basic initialization. */
    lights = 0xFF; /* all off */
    
    speed_clks = FASTEST_TIME;
    
    //      4-position DIP switch
    //    1       2       3       4
    //  SLAVE   TEST    WIRED    NOT       ON  ^
    //  MASTR   NORM    RADIO    USED      OFF v
    
    master_slave.mode(PullUp);
    master_slave.input();

    test.mode(PullUp);
    test.input();

    int_ZCD.mode(PullUp); 
    
    dipswitch.mode(PullUp);
    dipswitch.input();
    sequence = dipswitch.read();

#if 0
    /* Wait for the XBEE radio to get ready. It takes a while. */
    for(byte i=0xFF; i>=0xF4; i--) {
        wait(1.0);
//        lights = i;
    }
#endif
    
    if (!test) {
        while (1) {
            wait(0.1);
            lights = potentiometer.read_u16() >> 8;
        }
    }
    
    sd_present.mode(PullUp);
    sd_present.input();
    sd = !sd_present.read();
    
    MASTER = master_slave.read();
    
    wait(1.0);

    if (MASTER) {
        if(sequence < 240) {
            ptrSequence = (byte *) ptrSequences[sequence];
            sequenceLength = sequenceLengths[sequence];
            tkr_Timer.attach_us(&master_timer_isr, HALF_CYCLE);
            
            clocks = SLOWEST_TIME;
            
            /******************************************************** MASTER CHASE LOOP ********************************************************/
            while(1) {
                if (R) {
                    pc.putc('R');
                    R = 0;
                }
                else if(Z) {
                    pc.putc('Z');
                    Z = 0;

                    speed = A_COEFF * exp(B_COEFF * (1.0 - potentiometer)) + C_COEFF;       // read the potentiometer
                    __disable_irq();    // Disable Interrupts
                    speed_clks = SLOPE * speed + FASTEST_TIME;      // convert the analog speed voltage to a time in clocks
                    __enable_irq();     // Enable Interrupts 
                }
            }
            /****************************************************** END MASTER CHASE LOOP ******************************************************/
        }
        else {
            if (sd) {
                vfnLoadSequencesFromSD(sequence);
            }
            
            ptrDimSequence = ptrDimSeq;
            sequenceLength = DimSeqLen;
            
            clocks = dimmer_speed;
            new_pot = potentiometer;
            old_pot = new_pot;

            int_ZCD.fall(&master_zcross_isr);
            
            /******************************************************** MASTER DIMMER LOOP ********************************************************/
            while(1) {
//                Test_RXD = 1;
                new_pot = potentiometer;
                if (fabs(old_pot - new_pot) > 0.1) {
                    old_pot = new_pot;
                    total_clocks_per_step = dimmer_speed * ptrDimSequence[step].ticks;
                    clocks = total_clocks_per_step;
                }
                dimmer_speed = FASTEST_TIME + (SLOPE * (A_COEFF * exp(B_COEFF * (1.0 - new_pot)) + C_COEFF));
//                Test_RXD = 0;
                if (R) {
                    pc.printf("R\n");
                }
                else if(Z) {
                    pc.printf("Z\n");
                }
                if (R or Z) {
                    pc.printf("C %i\n", dimmer_speed);                // send the new speed to the slaves so they can dim at the correct rate
                    R = 0;
                    Z = 0;
                }
            }
            /****************************************************** END MASTER DIMMER LOOP ******************************************************/
        }
    }
    else {
        // this is a slave
        if(sequence < 240) {
            ptrSequence = (byte *) ptrSequences[sequence];
            sequenceLength = sequenceLengths[sequence];
            tkr_Timer.attach_us(&slave_timer_isr, HALF_CYCLE);

            clocks = SLOWEST_TIME;

            /******************************************************** SLAVE CHASE LOOP ********************************************************/
            while(1) {
                command_char = pc.getc();
                if (command_char == 'R') {
                    R = 1;
                }
                else if (command_char == 'Z') {
                    Z = 1;
                }
            }
            /***************************************************** END SLAVE CHASE LOOP ********************************************************/
        }
        else {
            vfnSlaveReceiveData(sequence);
            
            ptrDimSequence = ptrDimSeq;
            sequenceLength = DimSeqLen;
            
            clocks = dimmer_speed;

            int_ZCD.fall(&slave_zcross_isr);
            
            /********************************************************* SLAVE DIMMER LOOP ********************************************************/
            while(1) {
                vfnGetLine();
                
                if(line[0] == 'R') {
                    R = 1;
                }
                else if (line[0] == 'Z') {
                    Z = 1;
                }
                else if (line[0] == 'C') {
                    sscanf(line, "%*s %i", &dimmer_speed);
                }
            }
            /***************************************************** END SLAVE DIMMER LOOP ********************************************************/
        }
    }
}

