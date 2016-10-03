//****************************************************************************
// EC450
// Christina Howard
// Homework #1
// Due 9/23/16
//****************************************************************************
/*	I use two buttons (S1 and S2) to change between 3 states
 *	state 0: rest (steady Green LED)
 *	state 2: record (the Red LED blinks along with the pattern)
 *	state 1: playback (the Blue LED blinks along with the pattern recorded)
 *
 * Connections and Ports
 *	  Red LED is on P2.6
 *    Green LED is on P2.4
 *    Blue LED is on P5.6
 *    S2 (Switch State) is on P3.5
 *	  S1 (Record) is on P5.1
 *
 *
 *
 * The WDT is run off the SMCLK which by default is the same speed as the CPU clk
 * which, unless we change the digital oscillator parameters is ~3MHz.
 * In the code below the WDT is set to trigger every 2**13=8K clock cycles,
 * so the WDT interrupt occurs approximately every 8192/(3Mz) ~ 2.7ms
 *
 *Questions:
 * 1. I implemented the recording by keep an array that incremented along with the WDT for 3704 ticks which is close to 10 seconds. For every
 * start button pushes in the recording state 1 is stored in the time array for that tick. To play this recording I just iterate through this
 * array and if the value stored was 1 the blue playback light displays and when it was 0 the light was off.
 *
 * 2. I transitioned through states by having a button that controlled the state transitions. The state machine would look like:
 * Rest(state = 0, lastState =2) ->(Button)-> Record(state=2, lastState =0) -> (Ten Seconds) ->
 * Rest (state =0, lastState = 1) ->(Button)->  Playback (state = 1, lastState =0) ->(Ten Seconds) -> Rest(state =0, lastState =1)
 *
 *3. Limitations:
 *	-can only record for ten seconds
 *	- you can't fix your recording before it is played
 *	-
 */

#include "msp.h"

/* Variables used by the WDT handler must maintain their values between interrupts.
 * The easiest way to achieve this is to declare them globally
 */


volatile unsigned char stateSwitchButton;//P3.5 S2
volatile unsigned char startButton;//P5.1 S1, P1.1 Button 1
volatile unsigned char state; // state is 0,1,2;
volatile unsigned char lastState; // stores the value of the last state so we know to move from playback to record (vice versa)
volatile unsigned char recordTime[11111];//KEEPS TRACK OF BUTTON PRESSES FOR DISPLAY
volatile unsigned char toggle1x2 = 0;//set the value of toggle1x2 to 0, so that I can toggle between 0 and 1

/*
 * a procedure to change states (called by the WDT handler)
 */

//function to change the states
void change_state(){
    switch(state){
        case 0: {//rest state
            //blue(playback light) is off
            P5->OUT &= ~BIT6;
            //red (record light) is off
            P2 -> OUT &= ~BIT6;
            //green (rest light) is on non blinking
            P2->OUT |= BIT4;
            //printf(" The color is: green \n"); for debugging purposes
            lastState = state;//store the value of the current state to last state
            toggle1x2 ^= 1;//toggling toggle1x2
            state = 1 + toggle1x2;//add 1 so that I actually toggle between states 2 and 1, starting at 2
            break;
        }
        case 2: {//record state
        		//blue(playback light) is off
        	    P5->OUT &= ~BIT6;
        	    //green (rest light) is off
        	    P2->OUT  &= ~BIT4;
            //red(record light) is on, blinking along with inputs of the button
        		//P2 -> OUT |= BIT6;
        		///printf(" The color is: red \n"); for debugging purposes
            // checks the value of the button for each WDT tick that are in 10s
        		//play record that turns on the red light for each down button push
        		//10s == 3204 ticks
        		int i=3704;//3704
        		while (i > 0){
        			printf("The start button is: %d \n",P1 ->IN & BIT1);
        			if((P1 ->IN & BIT1) == 0){
        				recordTime[i] = 1;//time[i] = true to indicate that the light was on
        				P2 -> OUT  |=BIT6;//blinks the red light along with pattern
        			}else{
        				recordTime[i] = 0;//time[i]=0 just so we don't have garbage in the array
        				P2 -> OUT  &= ~BIT6;//blinks the red light along with the pattern stored
        			}
        			printf("the value of i is: %d \n\n", i);
        			i--;
        		}
        		lastState = state;
            	state = 0;
            break;
        }
        case 1: {//playback state
            //red(record light) is off
        		P2 -> OUT &= ~BIT6;
            //green (rest light) is off
        		P2->OUT  &= ~BIT4;
        		//blue (playback light) is on
        		//P5->OUT |= BIT6;
        		printf(" The color is: blue \n");
            //*make this function* call the playback function that turns on the LED for each true value in the time array
        		int i = 0;
        		while (i < 3704){//3704
        			printf("The value: %d \n",recordTime[i]);
        		    if(recordTime[i] == 1){
        		        	P5 -> OUT  |=BIT6;//turns on when time[i] = 1
        		    }else if (recordTime[i] == 0){
        		        	P5 -> OUT  &= ~BIT6;//turns off otherwise
        		   }else {

        		   }
        		        printf("the value of i is: %d \n\n", i);
        		        i++;
        		}
        		lastState = state;
        		state = 0;
        	break;
        }

    }

}

//not in use
//function that turns on the red light for each down button push (Display input function)
void shineBrightRecord(int buttonDownPresses){
//    if(buttonDownPresses)
//    {
        //toggle the red led
    		P2 -> OUT  ^=BIT6;
    		recordArray(buttonDownPresses);
//    }
}

//not in use
//function that turns on the LED for each true value in the time array (Playback function)
void shineBrightPlayback(){
	int i;
    for(i=0; i < 3704; i++){
        if(recordTime[i]){
            //turn on the green button for each true value
        		P2 -> OUT |= BIT4;
        }else {
        		P2->OUT  &= ~BIT4;
        }
    }
}

//not in use
//function that checks the value of the button for each WDT ticks that are in 10s (Record input function)
void recordArray(int recordButtonPresses){
	int i=10;
    while(i > 0){
        if(recordButtonPresses){
            recordTime[i] = 1;
        }else{
        		recordTime[i] = 0;
        }
        i--;
    }
}

/*
 * Watchdog Timer interrupt service routine
 * (Note: the function name for this handler is declared as an external routine in one of the
 * automatically loaded startup c files from the Code Composer system).
 */
void WDT_A_IRQHandler(void){
	//declaring variables
	unsigned char stateSwitchButton;
	stateSwitchButton = P3 ->IN & BIT5;
	startButton = P1 ->IN & BIT1;

	if(state == 2){ // if it is in record state
//		shineBrightRecord(startButton);
//		recordArray(startButton);
		change_state();
	} else if(state == 1){ // in the playback state
//		shineBrightPlayback();
		change_state();
	} else if(state == 0){ // in the rest state
		if(stateSwitchButton == 0){
			change_state();
		}else{
			P2->OUT |= BIT4;
			P5->OUT &= ~BIT6;  // Blue LED off to start with
			P2->OUT &= ~BIT6;  // Red LED off to start with
		}

	}else {
		//do nothing
	}


}




/*
 * The main program just initializes everything and (more or less) waits for events
 */

void main(void)
{

    /* Configure watch dog -- see the documentation on the WDT control register
     * - SMCLK as clock source
     * - Interval timer mode
     * - Clear WDT counter (initial value = 0)
     * - Watchdog interval = 8K = 2^13 ticks
     *      so that the interval is ~ 8K/(3Mhz) ~ 2.7ms
     */
    WDT_A->CTL = WDT_A_CTL_PW |				// 'password' to enable access
            WDT_A_CTL_SSEL__SMCLK |         // clock source = SMCLK
            WDT_A_CTL_TMSEL |               // this bit turns on interval mode
            WDT_A_CTL_CNTCL |               // clear the internal WDT counter
            WDT_A_CTL_IS_5;                 // specifies the divisor.  value 5 => 8K

    // Setup the output pins
    P2->DIR |= BIT4;   // Direction is output
    P2->OUT |= BIT4;  // Green LED on to start with

    P5->DIR |= BIT6;   // Direction is output
    P5->OUT &= ~BIT6;  // Blue LED off to start with

    P2->DIR |= BIT6;   // Direction is output
    P2->OUT &= ~BIT6;  // Red LED off to start with

    //Setup the input pins
        //state switch button
        P3 -> DIR |= BIT5;
        P3 -> IN & BIT5;//MAKE GLOBAL VARIABLE
        //start button
        P1 -> DIR |= BIT1;
        P1 -> IN & BIT1;//MAKE GLOBAL VARIABLE
        state = 0;
        lastState = 2;


    /*
     * setup so that the system sleeps until interrupted
     */

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Specify that after an interrupt, the CPU wakes up

    __enable_interrupt();					// allow the CPU to respond to interrupt signals.
    NVIC->ISER[0] = 1 << ((WDT_A_IRQn) & 31); // enable WDT to send interrupt signals

    /*
     * A short 'forever' loop that we can look at in the debugger
     * which spends only a tiny time with the CPU active.
     */
    while (1)
    {
        /* Go to LPM0 mode (Low power mode with CPU powered off */
        __sleep();		  //
        __no_operation(); //  For debugger
    }
}
