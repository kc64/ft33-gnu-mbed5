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

// #define VERBOSE 0
#ifdef VERBOSE
#warning "Using verbose mode."
#endif

// #define USB_TEST
#ifdef USB_TEST
#warning "Using USB test mode."
#endif


/* Serial debug port. */
Serial pc(P1_13, P1_14); // tx, rx

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

#define SLICE 65  // usec for 256 slices of a full AC cycle
#define MAX_SLICE 200   // how many slices to allow in a full AC cycle
#define HALF_CYCLE 8333     // usec for one half cycle of 60Hz power

/* The potentiometer input port to select the speed of the sequence steps. */
AnalogIn potentiometer(P0_11);

/* Setup the output pins. */
DigitalOut C0(P0_16);
DigitalOut C1(P0_17);
DigitalOut C2(P0_18);
DigitalOut C3(P0_19);
DigitalOut C4(P0_20);
DigitalOut C5(P0_21);
DigitalOut C6(P0_22);
DigitalOut C7(P0_23);

BusOut lights(P0_23, P0_19, P0_22, P0_18, P0_21, P0_17, P0_20, P0_16);

/* Setup the dipswitch input port. */
BusInOut dipswitch(P1_23, P0_12, P0_13, P0_14, P0_7, P0_8, P0_9, P1_24);
DigitalInOut master_slave(P0_4);
DigitalInOut local_slave_data(P0_5);

/* Setup the reset switch as an input to keep it from being a reset */
DigitalInOut reset(P0_0);

/* Setup the SD card detect input */
DigitalInOut sd_present(P1_15);

float speed;            /* The selected speed. */
int speed_clks;         /* speed in clocks (1/60th sec). */
int clocks;             /* Incremented everytime the zero cross interrupt is called. */
byte pattern;           /* The current output pattern. */
byte *ptrSequence;      /* A pointer to the desired sequence. */

word sequenceLength;    /* The length of the desired sequence. */
word step;              /* The step in the current sequence. */
byte num_ticks_per_step;  /* Each step can have one or more ticks before it changes. */
char line[100];
byte master_sequence; 
int got_Z = 0;
int got_R = 0;
char stmp[10];
byte current_step, slave_channel;
sDimStep *ptrDimSequence;
sDimStep *ptrDimSeq = NULL;
unsigned int DimSeqLen;

byte ticks = 0;
byte zc_slice = 0;

/* The dimmer timers for each channel. */
byte Dimmer[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void ZCD_SD(void);

/* Scaled version of the the dimmer for fixed point calculations. */
int Dimmer_sc[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void ZCD(void) {

    clocks++;
    if(clocks > speed_clks) {
        clocks = 0;
        step++;
        if(step >= sequenceLength) {
            step = 0;
            pc.putc('R');
        }
        else {
            pc.putc('Z');
        }
        pattern = ~ptrSequence[step];
        #ifdef VERBOSE
        //pc.printf("P:%02x ", ptrSequence[step]);
        #endif
        lights = pattern;  
    }
    
    #ifdef VERBOSE
    //pc.printf("Z. clocks: %d, speed_clks: %d\n\r", clocks, speed_clks);
    #endif
}

void ZCD_Slave(void) {
    if(got_R == 1) {
        step = 0;
    }
    else if(got_Z == 1) {
        step++;
        if(step >= sequenceLength) {
            step = 0;
        }
    }
    
    if ((got_R == 1) or (got_Z == 1)) {
        pattern = ~ptrSequence[step];
        #ifdef VERBOSE
        //pc.printf("P:%02x ", ptrSequence[step]);
        #endif
        lights = pattern;

        got_R = 0;
        got_Z = 0;      
    }
    
    #ifdef VERBOSE
    //pc.printf("Z. clocks: %d, speed_clks: %d\n\r", clocks, speed_clks);
    #endif
}

/* This routine is called about 255 times every 1/60 of a second. It is our chance to turn on a 
    channel based on the dimmer value. Dimmer is calculated at each good zero cross in ZCD_SD. */
void tmr_Main(void) {

    if (zc_slice > MAX_SLICE) {
        lights = 0xFF;              // C0-C7 all off
        zc_slice = 0;               // clear the slice counter for the next cycle
        tkr_FastInt.detach();    // disable this timer interrupt
        int_ZCD.fall(&ZCD_SD);      // enable the zero crossing interrupt since we're done dimming for this cycle
    }
    else {                          // we're still dimming so adjust every channel's slice counter
        zc_slice++;
        
        if (Dimmer[0] != 0) {
            Dimmer[0]--;
        } else {
            C0 = 0;
        }
        if (Dimmer[1] != 0) {
            Dimmer[1]--;
        } else {
            C1 = 0;
        }
        if (Dimmer[2] != 0) {
            Dimmer[2]--;
        } else {
            C2 = 0;
        }
        if (Dimmer[3] != 0) {
            Dimmer[3]--;
        } else {
            C3 = 0;
        }
        if (Dimmer[4] != 0) {
            Dimmer[4]--;
        } else {
            C4 = 0;
        }
        if (Dimmer[5] != 0) {
            Dimmer[5]--;
        } else {
            C5 = 0;
        }
        if (Dimmer[6] != 0) {
            Dimmer[6]--;
        } else {
            C6 = 0;
        }
        if (Dimmer[7] != 0) {
            Dimmer[7]--;
        } else {
            C7 = 0;
        }
    }
}

/* Switch all channels off. A zero crossing has just passed. Check the speed setting, find next ticks and steps. 
   Calculate the new dimmer setting based on the start and stop values. */
   
void ZCD_SD(void) {
    int i;
    
    int_ZCD.fall(NULL);                    // disable this ZCD interrupt
    
    /* A clock is a zero cross (1/60 second). */
    clocks++;
    /* We need speed_clks number of clocks before we tick. speed_clks is from the speed pot. */
    if(clocks > speed_clks) {
        clocks = 0;
        ticks++;
        /* If we tick enough times for this step, goto the next step. */
        if (ticks >= ptrDimSequence[step].ticks) {
            step++;
            ticks = 0;
        }
        /* If we step past the end of a sequence, restart the sequence. */
        if(step >= sequenceLength) {
            step = 0;
        }
        /* Put out the Z sync to the slaves. This tells them to step their sequence. */
        pc.putc('Z');

        for(i=0; i<8; i++) {
            Dimmer[i] = 255 - ptrDimSequence[step].Chan[i].start;   // this is a down counter so start high for low values
        }
    }
    else {
        /* If we don't need to tick or step, then find the new dimmer values for the next 1/60 second clock. This calcuation is a simple linear interperloation
            between the start and stop. At each 1/60 second interval we recalc the dimmer value along the line. */
        for(i=0; i<8; i++) {
            Dimmer[i] = 255 - (ptrDimSequence[step].Chan[i].start + (((clocks << 8)/speed_clks * (ptrDimSequence[step].Chan[i].stop - ptrDimSequence[step].Chan[i].start)) >> 8));
        }   
    }    
    
    /* Timer for the 255 step dimmer reoutine. */
    tkr_FastInt.attach_us(&tmr_Main, SLICE);

    #ifdef VERBOSE
    //pc.printf("Z. clocks: %d, speed_clks: %d\n\r", clocks, speed_clks);
    #endif

}

void ZCD_SD_Slave(void) {
    
    clocks++;
    if(got_Z == 1) {
        got_Z = 0;
        clocks = 0;
        ticks++;
        if (ticks >= ptrDimSequence[step].ticks) {
            step++;
            ticks = 0;
        }
        if(step != current_step) {
            step = current_step;
        }
        if(step >= sequenceLength) {
            step = 0;
        }
        //pattern = ~ptrDimSequence[step].Chan[slave_channel];
        #ifdef VERBOSE
        //pc.printf("P:%02x %02x %02x, %02x", ptrDimSequence[step].Chan[0], step, ticks, current_step);
        #endif
        lights = pattern;
    } 
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
        pc.printf("No file. Restarting...\n");
        // if the SD card is present but not responding, reset and try again
        NVIC_SystemReset();
    }
    else {
        while(fgets(line, 100, fp) != NULL) {
            pc.printf(line);        // transmit to the slaves
            
            if(line[0] == 'Q') {
                sscanf(line, "%*s %d %d", &sequence_num, &steps);
                if(sequence_num == sequence) {
                    ptr = (sDimStep *) malloc(sizeof(sDimStep) * steps);
                    if (ptr == NULL) {
                        pc.printf("Out of memory while loading sequence %d from SD card\n", sequence_num);
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
                    pc.printf("Out of memory while loading sequence %d from SD card\n", sequence_num);
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
    byte sync_char;

    /* Basic initialization. */
    lights.write(0xFF); /* all off */
    
    clocks = 0;
    speed_clks = FASTEST_TIME;
    
    master_slave.mode(PullUp);
    master_slave.input();

    local_slave_data.mode(PullUp);
    local_slave_data.input();

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
    
    sd_present.mode(PullUp);
    sd_present.input();
    sd = !sd_present.read();
    
    wait(1.0);

    /* Check master/slave and if a slave should use it's own sequence selection or get 
        it from a master. */
    if(master_slave.read() == 1) {
        if (sd) {
            vfnLoadSequencesFromSD(sequence);
        }

        if(sequence < 240) {
            ptrSequence = (byte *) ptrSequences[sequence];
            sequenceLength = sequenceLengths[sequence];
            tkr_Timer.attach_us(&ZCD, HALF_CYCLE);
        }
        else {
            ptrDimSequence = ptrDimSeq;
            sequenceLength = DimSeqLen;
            
            /* This sets an interupt when a zero cross is detected. */
            int_ZCD.fall(&ZCD_SD);
        }

        clocks = SLOWEST_TIME;
        while(1) {
            /* Read the potentiometer. */
            
            speed = A_COEFF * exp(B_COEFF*(1.0-potentiometer)) + C_COEFF;
    
            __disable_irq();    // Disable Interrupts
            /* Changes the analog speed voltage to a time in clocks. */
            speed_clks = SLOPE * speed + FASTEST_TIME;
            __enable_irq();     // Enable Interrupts 
    
            //pc.printf("C %i\n", speed_clks);
            wait(0.5);
        }
    }
    else {
        /* This is a slave board. */
        #ifdef FT_DEBUG
        pc.printf("Slave\n");
        #endif

        if(sequence < 240) {
            ptrSequence = (byte *) ptrSequences[sequence];
            sequenceLength = sequenceLengths[sequence];
            
            tkr_Timer.attach_us(&ZCD_Slave, HALF_CYCLE);
        }
        else {
            vfnSlaveReceiveData(sequence);
            ptrDimSequence = ptrDimSeq;
            sequenceLength = DimSeqLen;
                                  
            tkr_Timer.attach_us(&ZCD_SD_Slave, HALF_CYCLE);
        }

        clocks = SLOWEST_TIME;

        while(1) {
            // __disable_irq();
            sync_char = pc.getc();
            if (sync_char == 'R') {
                got_R = 1;
            }
            else if (sync_char == 'Z') {
                got_Z = 1;
            }
            // __enable_irq();
        }
    }
}

