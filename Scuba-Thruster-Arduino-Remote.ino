// WORKS 1/1/2022!

/* Scuba-Thruster-Arduino-Remote - Chris Drake 29 Nov 2018. 
 *  
 *  Reads a button.
 *  Writes a servo (ESC) position
 *
 * See: https://www.ztwoem.com/wp-content/uploads/2020/01/ZTW-Shark-Series-boat-ESC-User-Manual.pdf
 * and  https://www.aliexpress.com/item/1005002532106112.html

USAGE:

	0. Power on (not holding down any buttons)
	1. Press button to go forward (hold to go faster)
	2. Let go and quicky press again to lock in the current speed.  To accelerate from a locked-in speed, Let go and quicky press again (logically jumps back to step 1 above)
	3. When at full speed, let go and quicky press again to keep going full speed - must be done within 5 seconds, or the stuck-button code will stop the motor.
	4. Let go to slow down.  Press anytime when let go to resume accelerating.
	5. From stopped - Press and quicky let go then quickly press again to acclelerate in reverse (steps 2 though 4 apply still to lock in a speed etc)
	
CALIBRATION:

	1. Power off unit
	2. Power on while holding down the button.
	3. LED will blick fast (150ms on, 50ms off) and throttle will be output at 100%
	4. Let got and quickly press again to toggle output between 0% and 100% back to 0% etc etc.  LED will blick fast (50ms on, 150ms off for low) to indicate if throttle is high or low.
	5. Let go to center the throttle and exit calibration.
	*. NOTE:  If button stays pressed down for 10+ seconds, the Hardware-Fault code kicks in, and controller will be disabled (power off and on to reset).

ZTW "Shark" 2-way ESC setup:

	1. Enter Calibration (see above)
	2. Wait for "dash dash dash dash" system-reset tone
	3. Toggle (this will reset the ESC)
	4. Power off.
	5. Enter Calibration (see above)
	6. Wait for 2 beeps
	7. Let go (exit calibration)
	8. Wait for "armed" chime
	9. Test FWD/REV

*/


#include <Servo.h>	// To control the ESC
#include <EEPROM.h>	// for saving settings etc.

#include <SerialID.h>	// See https://github.com/gitcnd/SerialID
SerialIDset("\n#\tv1.20 " __FILE__ "\t" __DATE__ " " __TIME__); // So we know what code and version is running inside our MCUs

/*******************
 *   Pin settings  *
 *******************/
#define button_pin 2  		// unit-mounted backup power button and on/off switch
#define esc_pin 3     		// Speed controller servo PWM output
#define SERIAL_OUT 1  		// Comment this out to exclude allmost all the serialport code

Servo escservo;			// create servo object to control a servo
int escpos = 90;		// variable to store the servo position

#define state_buf_size 8
#define num_buttons 3
unsigned char button_pinno[num_buttons]={ button_pin, 255, 255 };	// 255 means ignore
volatile byte but_state_buf[num_buttons][state_buf_size]={ 42,42,42,42,42,42,42,42,  42,42,42,42,42,42,42,42,  42,42,42,42,42,42,42,42 };	// initially set to anything tht is not 0 or 1
volatile byte but_state_ptr[num_buttons]={0,0,0};			// points into the state_buf circular array at the current button state
volatile unsigned long but_millis[num_buttons][state_buf_size]={0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 }; // the millis() at the start of this state change - millis() wraps after ~ 50 days
byte state_pressed[num_buttons]={0,0,0};		// easy to reverse meaning for different kinds of buttons
byte state_released[num_buttons]={1,1,1};		// easy to reverse meaning for different kinds of buttons
volatile unsigned long but_timer[num_buttons]={0,0,0};	// how many millis since the last state change
volatile unsigned long but_prestate[num_buttons]={0,0,0};// how many millis since the last button-debounce change
volatile unsigned long but_mtrans[num_buttons]={0,0,0};	// the millis() for the start of a state transition, for debounce code
volatile int newstate=1;				// set to 1 in the ISR when a button state has changed. Starts at 1 to run the initial loop first
volatile int show=1;					// debugging trigger to show the current state
int min_doubleclick[num_buttons]={120, 120, 120};	// must be longer than this to be considered a double-click
int max_doubleclick[num_buttons]={350, 350, 350};	// must be shorter than this to be considered a double-click
int but_debounce[num_buttons]={5,5,5};			// needs to be more than this before we even care that it happened
int but_accel[num_buttons]={33,33,33};			// How fast to incriment the throttle (we go +1 every this-number-of-millis) - 33 is 1000/30 (range is +/-90) and thus stop to full in 3 seconds.
int but_decel[num_buttons]={15,15,15};			// How fast to decriment the throttle (we go -1 every this-number-of-millis)
unsigned long speed_millis;				// How many millis() to wait before we do
unsigned long motor_lock_millis;			// For timing stuck buttons
enum { idle, pre_reverse, reverse, ramping_up, pre_up_lock, locked, winding_down, but_stuck } motor_state; // idle=stopped. pre_reverse is <max_doubleclick from start, before ramping_up
boolean motor_reverse=false;
unsigned char esc_revspeed = 0;			// maximum speed in reverse - for ESC calibration ( 0 is full speed backwards ) - only used for calibration - is required to be equidistant from esc_zerospeed to esc_maxspeed
unsigned char esc_zerospeed = 90;		// Set this to neutral when the ESC understands reverse
unsigned char esc_maxspeed = 180;		// for ESC calibration (180 is full speed)
unsigned char esc_curspeed=esc_zerospeed;	// Current speed
unsigned char esc_lastspeed=42;			// Previous speed - init to anything besides esc_curspeed
unsigned char invert = 0;			// Set this to reverse the entire channel
unsigned char calibration = 0;			// If powered up with button down (and button released within 2s) - sets this mode.
#define slack_speed_rev 5			// For calibration, we add 5 degrees to what we consider full reverse (not used)
#define slack_speed_fwd 5			// For calibration, we subtract 5 degrees to what we consider full speed (so we can be certain that the ESC does go 100% when we want 100%)
#define calibration_delay 10000			// if held for longer that this - calibation is cancelled
#define safety_lock 5000			// if held for longer that this when already at max speed (without a double-click) - power back down.
int lastbut=0;					// Last active button index
bool pressed=false;

/* Measurement notes:
  Fastest possible click-and-release takes over 55ms
  Very fast up,down from in the middle of a down is 150ms
  Very slow up,down ................................350ms

ptr:0 buf:0(8771) 1(14912) 0(152) 1(886) 0(21554) 1(3375) 0(111) 1(4294917535)  t:199114 190343 175431 175279 174393 152839 149464 149353 



/***************************
 *   LED Indicator Init.   *
 ***************************/
byte cnd_ledPin = 13; 		// We use blink-codes to indicate OK (breif flashes) or ERROR states (rapid blinking)
byte cnd_ledtoggle=0; 		// holds LED state
int cnd_ledon=20;     		// Can lengthen this (e.g. 200) to indicate error states
int cnd_ledoff=480;  		// Can shorten this (e.g. 200) to indicate errors
unsigned long cnd_ledtimer=millis(); // for timing the LED flashes
unsigned long run_number=0;	// stats


// get (and optionally update) a big number of ouf EEPROM
unsigned long use_EEPROM(int address,byte incriment) {
  unsigned long res=0;
  for(int i=address*4; i < (address+1)*4; i++) {
    res=(res<<8) + EEPROM.read(i);
  }
  if(incriment) {
    res++;
    unsigned long res2=res;
    for(int i=address*4+3; i >= address*4; i--) {
      //int out=res&255;
      EEPROM.write(i,res2&255);
      res2=res2>>8;
    }
  }
  return res;
} // use_EEPROM



/******************
 *   Timer ISR.   *
 ******************/
ISR(TIMER2_COMPA_vect) {	// Interrupt service: runs when Timer/Counter1 reaches OCR1A (your own top value). OCR1A = 61; means 1008 interrupts per second
  serviceLED();                	// short blinks on the LED occasionally so we know we are alive - NB - Do not remove - we need the above bit-tap to be separated from the below un-tap
  for(int b=0;b<num_buttons;b++) {
    if(!newstate) but_check(b);
  }
} // Timer ISR



/*******************
 * ESC Calibration *
 *******************/
void calibrate() { // Calibration - button is pulled high, so, 0 here means it's held down at boot time...
  unsigned long calstart=millis();
  int b=0;
  unsigned long prevpres=calibration_delay+1;
  esc_curspeed=esc_maxspeed-slack_speed_fwd; servoWrite(esc_curspeed);
  cnd_ledon=150; cnd_ledoff=50;
#ifdef SERIAL_OUT
  Serial.print(F("Calibration mode: Release and quickly press to toggle 0% / 100%.  Release for neutral.  speed="));
  Serial.println(esc_curspeed);
#endif
  esc_curspeed=esc_revspeed+slack_speed_rev; // reverse the logic of the first toggle (so it stays 100% at start)
  
  while(( (millis()-calstart) < calibration_delay ) && ( calstart != calibration_delay)) {
    serviceLED();
    for(int bc=0;bc<num_buttons;bc++) {
      if(!newstate) but_check(bc);					// Was done in the ISR... but that screws with the servo library...
    }

    if(newstate) {							// Set inside but_check() Interrupt Service Routine(ISR)
      b=newstate-1;
      calstart=millis();						// Stay in calibrate loop until timer runs out
      newstate=0;
    }
    byte q=but_state_ptr[b];						// current state pointer
    unsigned long press0=millis()-but_millis[b][q];                     // How long has this button been held down for so far?
    pressed=(but_state_buf[b][q]==state_pressed[b]);                    // Button state placeholder

    if(!pressed) {
      prevpres=press0;							// count how long they have it let go for
      if(press0>max_doubleclick[b]) {					// They let go - calibration over.
        esc_curspeed=esc_zerospeed; servoWrite(esc_curspeed); calstart=calibration_delay; // Exits calibration
#ifdef SERIAL_OUT
  	    Serial.print(F("Calibration over. New speed="));
        Serial.println(esc_curspeed);
#endif
      }
    } else if(prevpres<=max_doubleclick[b]) {				// Toggle speed
      if( esc_curspeed > esc_zerospeed ) { esc_curspeed=esc_revspeed+slack_speed_rev; cnd_ledon=50; cnd_ledoff=150;}
      else { esc_curspeed=esc_maxspeed-slack_speed_fwd; cnd_ledon=150; cnd_ledoff=50; }
#ifdef SERIAL_OUT
      Serial.print(F("Toggle. New speed="));
      Serial.println(esc_curspeed);
#endif
      servoWrite(esc_curspeed);
      prevpres=calibration_delay;
    } // pressed
  } // calibration_delay 

  // Hardware Problem:- button stuck down for 10+ seconds.
  // if( digitalRead(button_pinno[b]) == state_pressed[b] ) {                    // *Still* pressed....
  if(pressed) {
    esc_curspeed=esc_zerospeed; servoWrite(esc_curspeed);
    cnd_ledon=450; cnd_ledoff=50;
    escservo.detach();
#ifdef SERIAL_OUT
    Serial.println(F("Halted: Button stuck"));
#endif
    while(1) { serviceLED(); } 
  } // Button stuck

  cnd_ledon=20; cnd_ledoff=480; // Back to normal at end
} // calibrate



/****************
 *   setup().   *
 ****************/
void setup() {
  int b=0;	// Button 0 is our calibration button
  motor_state=idle;
 
  //run_number=use_EEPROM(0,1); // EEPROM Addresses 0,1,2,3 are our run counter.
#ifdef SERIAL_OUT
  SerialIDshow(115200); // starts Serial.
  //Serial.print(F("Run number:")); Serial.println(run_number);
#endif

  for(int br=0;br<num_buttons;br++) {
    if(button_pinno[br] < 255) { // Real pin
      pinMode(button_pinno[br], INPUT_PULLUP);	// We have a button on D2 for power, and so divers with broken bite-controlls can still activate the unit
    }
  } // [br]
  pinMode(4, OUTPUT); digitalWrite(4,0);// Send GND out of another pin (the button connects to this) so we can use the Nano GND pin for power.
  pinMode(esc_pin, OUTPUT);		// The PWM output to the ESC

  escservo.attach(esc_pin);		// attaches the servo on pin 3 to the servo object

  if( digitalRead(button_pinno[b]) == state_pressed[b] ) calibrate();	// Power on with button pressed => enter calibration mode.

  servoWrite(esc_curspeed);		// neutral (esc_zerospeed) by default
  speed_millis=millis();
} // setup


void servoWrite(int spd_in) {
  int spd;
  if(motor_reverse) spd=180-spd_in; else spd=spd_in;	// Get desired direction form global var

  if(spd != esc_lastspeed) {		// Don't re-send what it is already doing...
    if(invert) {
      escservo.write(180-spd);		// 0 is full speed, 180 is full reverse, 90 is stopped.
    } else {
      escservo.write(spd);
    }
  }
  esc_lastspeed=spd;
} // servoWrite

/***************************************************
 *   but_check() - see if the button has changed.  *
 ***************************************************/
void but_check(int b) {
  if(button_pinno[b] < 255) { // Real pin
    byte but_now = digitalRead(button_pinno[b]);
    if( but_now == but_state_buf[b][but_state_ptr[b]] ) {	// No change
      but_timer[b] = millis()-but_millis[b][but_state_ptr[b]];	// Update/Remember how long this was pressed for
      but_mtrans[b]=0;						// debounce weirdness to ignore
    } else {							// Changed
      if(but_mtrans[b]==0) {					// First time we've seen the change
        but_mtrans[b]=millis();					// so we can start timing how long it has been held/released for
      } else if(millis() > ( but_mtrans[b] + but_debounce[b] )) {// DeBounce wait is over
        but_state_ptr[b] = (but_state_ptr[b]+1)%state_buf_size;	// move to next place in queue
        but_state_buf[b][but_state_ptr[b]] = but_now;		// remember the buttone state
        but_millis[b][but_state_ptr[b]]=but_mtrans[b];		// Remember the hit-time, so we can work out the how-long part later
        but_timer[b]=millis()-but_mtrans[b];			// Timer starts at the debounce delay approx
        newstate=1+b;						// Tell the main loop outside this ISR that we found a state-change.
        but_mtrans[b]=0;					// finish debouncing
      } // else { /* ignorred in debounce */ }
    }
  } // real button pin (not a 255 placeholder)

} // but_check



/**************
 *   loop().  *
 **************/
unsigned short foo=1;
void loop() {
  serviceLED();							// short blinks on the LED occasionally so we know we are alive - NB - Do not remove - we need the above bit-tap to be separated from the below un-tap
  for(int b=0;b<num_buttons;b++) {
    if(!newstate) but_check(b);					// Was done in the ISR... but that screws with the servo library...  (skip rest when we hit any newstate, so all buttons get their chance...)
  }

  if(newstate) {						// Set inside but_check() Interrupt Service Routine(ISR)
    lastbut=newstate-1;
    newstate=0;
    show=1;
  }


  /* motor_state :-
   *1   idle		no power for > max_doubleclick
   *10   pre_reverse	after idle, sits in this mode for upto max_doubleclick, getting ready to go backwards.  Goes forwards otherwise
   *10   reverse		after pre_reverse, waiting for them to click again in under max_doubleclick ms, so we can transition into backwards-ramping_up
   * 0  ramping_up	ramping_up button is held down to change speed (forwards or backwards)
   *10   pre_up_lock	after ramping_up, they just let go of the button will go to locked or winding_down after max_doubleclick ms
   * 0  locked		after pre_up_lock, to lock in the current speed
   *1   winding_down	after pre_up_lock, do slow down
   *10   but_stuck	after ramping_up, if they held the button down too long (5000s+)
   */

  if(1) { // Logic here to decide the mode...	enum { idle, pre_reverse, reverse, ramping_up, pre_up_lock, locked, winding_down, but_stuck } motor_state;
    int b=lastbut;
    byte q=but_state_ptr[b]; 						// current state pointer
    byte p=(q+(state_buf_size-1))%state_buf_size; 			// previous state pointer
    unsigned long press0=millis()-but_millis[b][q];			// How long has this button been held down for so far?
    pressed=(but_state_buf[b][q]==state_pressed[b]);			// Button state placeholder

    if(pressed && motor_state == idle) {
      motor_state = pre_reverse; show=1;				// Get ready to power-up in reverse if a double-click happens from idle...
      motor_reverse = false;

    } else if(pressed && motor_state == pre_reverse) {
      if(press0 > max_doubleclick[b]) {					// If it was held down too long from start - it is not a double-click now...	Go Faster
	motor_state = ramping_up;show=1;
	speed_millis=millis()-but_accel[b]*2;				// Start accelerating from right now
      }

    } else if(!pressed && motor_state == pre_reverse) {
      motor_state = reverse; show=1;					// Get ready to go backwards

    } else if(!pressed && motor_state == reverse) {
      if(press0 >max_doubleclick[b]) {					// Released for too long...							Go Slower
        motor_state = winding_down;
	motor_reverse = false;
	speed_millis=millis()-but_decel[b]*2;				// Start accelerating from right now
	show=1;
      }

    } else if(pressed && motor_state == reverse) {
      motor_state = ramping_up; show=1;					// Clicked again quickly at start						Go Faster (backwards)
      motor_reverse = true;
      speed_millis=millis()-but_accel[b]-1;				// Start accelerating from right now

    } else if(!pressed && motor_state == locked) {
      motor_state = winding_down; show=1;				// If they just let go from a locked-in speed, slow down now
      speed_millis=millis()-but_decel[b]-1;				// Start accelerating from right now

    } else if(!pressed && motor_state == ramping_up) {
      motor_state = pre_up_lock; show=1;				// If they just let go from acceleration, get ready to lock in the speed

    } else if(!pressed && motor_state == pre_up_lock) {
      if(press0>max_doubleclick[b]){					// They had it "let go" for longer than a double-click...			Go Slower
	motor_state = winding_down;
	show=1;
	speed_millis=millis()-but_decel[b]*2;				// Start accelerating from right now
      }

    } else if(pressed && motor_state == pre_up_lock) {
      motor_state = locked; show=1;					// They just double-clicked. Lock in the speed now.

    } else if(pressed && motor_state == winding_down) {
      motor_state = ramping_up;	show=1;					// Clicking again from slowing-down means 					Go Faster (again, after slowing down some)
      speed_millis=millis()-but_accel[b]-1;				// Start accelerating from right now

    // } else if(!pressed && motor_state == winding_down) {		// Gets set to motor_state=idle when we hit zero speed below...

    } else if(pressed && motor_state == but_stuck) {			// Gets set to this below if button stays down for > 5000 ms (safety_lock) after full speed reached
      esc_curspeed=esc_zerospeed; servoWrite(esc_curspeed);

    } else if(!pressed && motor_state == but_stuck) {
      cnd_ledon=20; cnd_ledoff=480;					// Reset the safety_lock coutout LED indicator
      motor_state = idle; show=1;

    // } else if(pressed && motor_state != idle) {
    }
  } // mode logic


  for(int b=0;b<num_buttons;b++) {
    byte q=but_state_ptr[b]; 				// current state pointer
    byte p=(q+(state_buf_size-1))%state_buf_size; 	// previous state pointer	

    // Logic here to decide:
    //  safety cutoff when no double-click locked in
    //  double-click to lock in a speed
    //  reverse... etc.


    if(but_state_buf[b][q]==state_pressed[b]) {							// TEMP - Go faster
      digitalWrite(cnd_ledPin, 1); 							 	// visual feedback of button push
      if(motor_state == ramping_up) {
        while( (esc_curspeed<esc_maxspeed) &&  (millis()-speed_millis) > but_accel[b] )  {	// Means we need to incriment to go faster
  	  speed_millis+=but_accel[b]; 
  	  //speed_millis=millis();//temp
	  esc_curspeed++;
	  motor_lock_millis=millis();								// For detecting stuck buttons
	  show=1;
        }
      }
      if((esc_curspeed==esc_maxspeed) && ( motor_state==ramping_up )) {
	if((millis()-motor_lock_millis) > safety_lock) {					// 5000 ms
	  cnd_ledon=200; cnd_ledoff=100;
	  motor_state=but_stuck;
	  esc_curspeed=esc_zerospeed; servoWrite(esc_curspeed);
#ifdef SERIAL_OUT
	  Serial.println(F("Safety lock: motor cut"));
#endif
        }
      }
    } // faster

    if(but_state_buf[b][q]==state_released[b]) { // TEMP - go Slower
      if(motor_state == winding_down) {
        while( (esc_curspeed!=esc_zerospeed) &&  (millis()-speed_millis) > but_decel[b] )  {

	  speed_millis+=but_decel[b];
          if(esc_curspeed<esc_zerospeed) esc_curspeed++; else esc_curspeed--; // go whichever way is needed to reach the zero
	  show=1;
        }
        if(esc_curspeed==esc_zerospeed) motor_state=idle;
      }
    } // slower

#ifdef SERIAL_OUT
    if(!foo++){b=0;show++;} // do this every 65535 ms

    if(show){
      if(pressed) Serial.print(F("P1 ")); else Serial.print(F("P0 "));
      if(motor_reverse) Serial.print(F("Rev ")); else Serial.print(F("Fwd "));
      if     (motor_state == idle)		Serial.print(F(" idle        "));
      else if(motor_state == pre_reverse)	Serial.print(F(" pre_reverse "));
      else if(motor_state == reverse)		Serial.print(F(" reverse     "));
      else if(motor_state == ramping_up)	Serial.print(F(" ramping_up  "));
      else if(motor_state == pre_up_lock)	Serial.print(F(" pre_up_lock "));
      else if(motor_state == locked)		Serial.print(F(" locked      "));
      else if(motor_state == winding_down)	Serial.print(F(" winding_dwn "));
      else if(motor_state == but_stuck)		Serial.print(F(" but_stuck   "));
      byte q=but_state_ptr[b]; // current state pointer
      byte p=(q+(state_buf_size-1))%state_buf_size; // previous state pointer	
      Serial.print(F("spd:")); Serial.print(esc_curspeed);
      Serial.print(F(" ptr")); Serial.print(b); Serial.print(F(":")); Serial.print(q);  Serial.print(F("p")); Serial.print(p); // ptr0:4.3
      Serial.print(F(" buf:")); Serial.print(but_state_buf[b][q]); Serial.print(F("(")); Serial.print(millis()-but_millis[b][q]); Serial.print(F(")   "));
      for(int i=1;i<state_buf_size;i++) {
        Serial.print( but_state_buf[b][p]);
	Serial.print( F("("));
        Serial.print( but_millis[b][q]-but_millis[b][p]); Serial.print(F(") "));

        q=(q+(state_buf_size-1))%state_buf_size; // Go backwards
        p=(p+(state_buf_size-1))%state_buf_size;
      }
      Serial.println();
      show=0;
    }
#endif

  } // [b]

  servoWrite(esc_curspeed);	// This is a NOP if the speed has not changed...

} // loop


void serviceLED() { // ISR call to blink the LED
  // Blink the LED so we know we are alive
  if(cnd_ledtimer<millis()) {
    cnd_ledtoggle++; digitalWrite(cnd_ledPin, cnd_ledtoggle&1);
    cnd_ledtimer=millis();
    if(cnd_ledtoggle&1) { cnd_ledtimer+=cnd_ledon; } else {cnd_ledtimer+=cnd_ledoff;} // NB: LED is wired backwards (low=on)
  }
}
