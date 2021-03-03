#include<Wire.h>

#define LED LED_BUILTIN /* builtin LED indicating when we are a master */
#define OUT 10 /* Our OUT line */
#define OtherMASTER  13 /* Our OtherMaster line */

////////////////////* Bus arbitrator FSM states */////////////////////////
#define NoMasterNoTrigger 0     //idle state
#define TriggerNoMaster 1       //no-one is master and a trigger has been detected
#define AttemptToBeMaster 2     //we will attempt to claim master
#define TriggerIamMaster 3      //we are master after a successful claim
#define TriggerOtherMaster 4    //we want to be a master but the other has gone in first
#define TriggerCollision 5      //both micro-controllers are masters, i.e. a collision
#define NoTriggerOtherMaster 6  //we do not want to be a master and the other is already a master

#define ARDUINO_ADDRESS     8

#define TEMP_OUT_H 0x41                                 // register address for temperature
#define TEMP_OUT_L 0x42                                 // register address for temperature

#include <avr/wdt.h>
#define wdt_disable()
#define wdto_15ms 0

void resetArd(){
  wdt_disable();          
  wdt_enable(wdto_15ms);  
  while(1); 
}

//////////////////////* to display the 7 segment without using sgift register *///////////////////
#define SEG_A   9    
#define SEG_B   8
#define SEG_D   A1
#define SEG_DP  A2
#define SEG_C   A3

////////////////////// * traffic lights by using bare metal and the normal way */////////////////////////
#define RED1      B00000100 
#define YELLOW1   B00001000 
#define GREEN1    B00010000 
#define RED2      B00100000 
#define YELLOW2   B01000000 
#define GREEN2    B10000000 

#define Red1      2
#define Yellow1   3
#define Green1    4
#define Red2      5
#define Yellow2   6   
#define Green2    7

#define mainRED 2
#define mainYELL 3
#define mainGREEN 4
#define secRED 5
#define secYELL 6   
#define secGREEN 7

/////////////////////////* buttons decleration *//////////////////////////////////////////
#define _B1 B00001000                               
#define _B2 B00010000
#define B1_PRESSED  !(PINB & _B1)                     // button 1 is low 
#define B1_RELEASED ! B1_PRESSED                      // button 1 is high 
#define B2_PRESSED  !(PINB & _B2)                     // button 2 is low 
#define B2_RELEASED ! B2_PRESSED                      // button 2 is high 
#define debounce 300    
#define pelican_debounce 50                           // pelican debounce duration 


enum priority_t { Equal_Priority, Set1_Priority, Set2_Priority };             
priority_t module0_priority, module0_new_priority; 
enum button_State_t { Idle, pP, nP, S };                         // type for the debouncer FSM state variables                      
enum b_State_t { notPRESSED, partialPRESS, normalPRESS, Stuck }; // type for the debouncer output variables               
enum pelican_Controller_State_t { f_State, a_State, b_State, c_State, d_State_off, d_State_on, e_State_off, e_State_on, i_State, ii_State };
enum BarrierState_t { No_Trains, Trains, Critical };                      
enum F1State_t { Idle_1, Start_Sequence, GO, Red_Flag, Abort, Green };                  
enum maintenance_mode_state_t {State_off, State_on};


unsigned long timestamp;
int count = 0;
bool raceStarted;             
long randNumber;
char state;
bool trigger;
int priority;
int Priority1;
int Priority2;
int activeSet = 0;
const int MPU_addr=0x68;                                                // I2C address of the MPU-6050 
const int PWR_MGMT_1 = 0x6B; 
const int ACCEL_XOUT_HI = 0x3B; 
char result;         
int AcX,AcY,AcZ,TEMP_OUT;
float T; 
bool FirstTime = true;
byte Command = 0x00;
byte LightStatus = 0x00;
int Status;
char ACK[16];
byte TiltStatus = 0x00;
int i = 0;
byte CurrentCycle =0x00;
byte NextCycle = 0x00;
int total_SW1_counts;
int total_SW2_counts;
int SW1_waiting_counts;
int SW2_waiting_counts;
BarrierState_t BarrierState; 
F1State_t F1_State; 

/////////////////// module0 is my traffic light controller////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// module0 variables    
bool init_module0_clock; 
unsigned long module0_time, module0_delay; 
bool module0_doStep; 
unsigned char module0_i;                         
unsigned char module0_count;  

//////////////// module1 is my button 1 debouncer and driver////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// module1 variables    
bool init_module1_clock; 
unsigned long module1_time, module1_delay; 
unsigned long debounce_count1; 
bool module1_doStep; 
unsigned char module1_i; 
b_State_t B1_State, B2_State;

////////////// module2 is the second button driver and debouncer//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// module2 variables 
bool init_module2_clock; 
unsigned long module2_time, module2_delay; 
unsigned long debounce_count2; 
bool module2_doStep; 
unsigned char module2_i; 
button_State_t Button2_State;
b_State_t b2_State; 

///////////////////////////// module 3 maintenance //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool init_module3_clock; 
unsigned long module3_time, module3_delay; 
bool module3_doStep; 
unsigned char module3_i;                                             
unsigned char module3_count;                                           


//////////////////// module4 is the heart beat module //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//module4 variables    
bool init_module4_clock; 
unsigned long module4_time, module4_delay; 
bool module4_doStep; 
unsigned char module4_i; 
unsigned char ledState;

///////////////////////////////////// module5 barrier button controller////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool init_module5_clock; 
unsigned long module5_time, module5_delay; 
//unsigned long debounce_count1, debounce_count2; 
bool module5_doStep; 
unsigned char module5_i;      

////////////////// module6 barrier light controller//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// module6 variables    
bool init_module6_clock; 
unsigned long module6_time, module6_delay; 
bool module6_doStep; 
unsigned char module6_i; 

///////////////////////////////////////// module7 F1 button ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool init_module7_clock; 
unsigned long module7_time, module7_delay; 
//unsigned long debounce_count1, debounce_count2; 
bool module7_doStep; 
unsigned char module7_i;   

/////////////////////////////////// module8 F1 light controller/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// module6 variables    
bool init_module8_clock; 
unsigned long module8_time, module8_delay; 
bool module8_doStep; 
unsigned char module8_i; 

////////module9 variables pelican button controller/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool init_module9_clock;
unsigned long module9_time, module9_delay;
bool module9_doStep;
unsigned char module9_i; // state variable for the module       

/////////////////module10 variables pelican controller//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool init_module10_clock;
unsigned long module10_time, module10_delay;
bool module10_doStep;
pelican_Controller_State_t module10_i; // state variable for the module

//////////////////// module 11 is my scheduler /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// module11 variables    
bool init_module11_clock; 
unsigned long module11_time, module11_delay; 
bool module11_doStep; 
unsigned char module11_i; 

//////////////////////////////////// module 12 is accelerometer ////////////////////////////////////////////////////
//module12 variables. It is used as a test bench for the bus arbitrator 
bool init_module12_clock; 
unsigned long module12_time, module12_delay, module12_timestamp; 
bool module12_doStep; 
unsigned char module12_i; // state variable for module 12 
boolean demand, granted; // demand is an output variable, granted is the reply from the pelican controller

void displayB(){    // barrier
    digitalWrite(SEG_A, HIGH);
    digitalWrite(SEG_B, HIGH);
    digitalWrite(SEG_C, HIGH);
    digitalWrite(SEG_D, HIGH);
}

void displayP(){  // pelican
    digitalWrite(SEG_A, HIGH);
    digitalWrite(SEG_B, HIGH);
    digitalWrite(SEG_C, LOW);
    digitalWrite(SEG_D, LOW);    
}

void displayt(){  //trafic light and maintenance
    digitalWrite(SEG_A, LOW);
    digitalWrite(SEG_B, LOW);
    digitalWrite(SEG_C, LOW);
    digitalWrite(SEG_D, HIGH);
}
    

void displayb(){  // emergency barrier
    digitalWrite(SEG_A, LOW);
    digitalWrite(SEG_B, LOW);
    digitalWrite(SEG_C, HIGH);
    digitalWrite(SEG_D, HIGH);
}

void displayF(){  // formula1
    digitalWrite(SEG_A, LOW);
    digitalWrite(SEG_D, LOW);
    digitalWrite(SEG_C, LOW);
    digitalWrite(SEG_B, HIGH);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);  
  
  pinMode(SEG_DP, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);    
  pinMode(SEG_B, OUTPUT);    
  pinMode(SEG_A, OUTPUT);
  
  pinMode(LED, OUTPUT);
  pinMode(OUT, OUTPUT);
  pinMode(OtherMASTER, INPUT_PULLUP);
  state = NoMasterNoTrigger;
  trigger = false;
    
  init_module0_clock=true; 
  init_module1_clock=true; 
  init_module2_clock=true;
  init_module3_clock=true;
  init_module4_clock =true;
  init_module5_clock=true;
  init_module6_clock=true; 
  init_module7_clock=true;
  init_module8_clock=true;
  init_module9_clock=true;
  init_module10_clock=true; 
  init_module11_clock=false;
  init_module12_clock=true;
  
  module11_i=0;
  
 B1_State = B2_State = notPRESSED;
 Serial.begin(115200);
 randomSeed(analogRead(A0));                     /* it is assumed here A0 is not connected */
 Wire.begin(ARDUINO_ADDRESS); 
 Wire.onReceive(receiveEvent);                   /* register receive event */
 Wire.onRequest(requestEvent);                   /* register request event */
}

bool Othermaster(){
  return(digitalRead(OtherMASTER)!=HIGH);
}

void loop() {
// arbitrator state machine
  switch (state) {
    case NoMasterNoTrigger: 
      digitalWrite(OUT, HIGH);
      digitalWrite(LED,LOW);
       timestamp = millis() + random(10);
      if   (trigger && !Othermaster()) {state =  AttemptToBeMaster;}
      else {state = NoMasterNoTrigger;}
      break;

     case AttemptToBeMaster: 
      digitalWrite(OUT, LOW);
      digitalWrite(LED,LOW);
      if   (!trigger) {state = NoMasterNoTrigger;}
      else if (Othermaster()) {state = TriggerCollision;}  
      else if ((long)(millis() - timestamp) < 0) state = AttemptToBeMaster;
      else state = TriggerIamMaster;
      break;
        
    case TriggerIamMaster: 
      digitalWrite(OUT, LOW);
      digitalWrite(LED,LOW);  // or high
      if   (!trigger) {state = NoMasterNoTrigger;}
      if   (trigger && !Othermaster()) {state = TriggerIamMaster;}  // or trigeermomaster
      if   (trigger && Othermaster()) {state = TriggerCollision;}
      break;
        
    case TriggerCollision:
      digitalWrite(OUT, HIGH);
      digitalWrite(LED,LOW);
      timestamp = millis() + random(10);
      /* no break here! why is that? */
       if ((long)(millis() - timestamp) < 0) state = TriggerCollision;
      else {
        if   (!trigger) {state = NoMasterNoTrigger;}
        if   (trigger && !Othermaster()) {state = AttemptToBeMaster;}
        if   (trigger && Othermaster()) {state = TriggerCollision;}
      }
      break;

    default:
      state = NoMasterNoTrigger;
      break;
  } 
{///////////////////////////////////////////// module0 - traffic light////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 if (init_module0_clock) { 
      module0_delay = 500; 
      module0_time=millis(); 
      module0_doStep=false; 
      init_module0_clock=false; 
      module0_i= 10; module0_count=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module0_time > module0_delay) { 
        module0_time = m; module0_doStep = true; 
      } 
      else module0_doStep = false; 
    } 

    if (module0_doStep) {
    displayt(); 
    switch(module0_i){ 
          case 10: // attention = yellow || yellow 
            PORTD = YELLOW1 | YELLOW2; 
            module0_delay = 3000; 
            module0_i = 0; 
            break; 
            
          case 0: // =red || red 
            activeSet = 1;
            PORTD = RED1 | RED2; 
            module0_delay = 1000; 
            module0_i = 1; 
            break; 
            
          case 1: // =red, yellow || red 
            PORTD = RED1 | YELLOW1 | RED2; 
            module0_delay = 2000; 
            module0_i = 2; 
            break; 
            
          case 2: // =green | red 
            PORTD = GREEN1 | RED2;
          if(Command == 0x61){
            module0_delay = 7000;
          }
          else if(Command == 0x62){
             module0_delay = 9000;         
          }
          else{(Command == 0x63);
            module0_delay = 5000;
          }
            module0_i = 3; 
            break; 
            
          case 3: // =yellow | red 
            PORTD = YELLOW1 | RED2; 
            module0_delay = 2000; 
            module0_i = 4; 
            break; 
            
          case 4: // =red | red
            activeSet = 2; 
            PORTD = RED1 | RED2; 
            module0_delay = 1000; 
            module0_i = 5; 
            break; 
            
          case 5: // =red | red, yellow 
            PORTD = RED1 | RED2 | YELLOW2; 
            module0_delay = 2000; 
            module0_i = 6; 
            break; 
            
          case 6: // =red | green 
               PORTD = RED1 | GREEN2; 
            if(Command == 0x61){
              module0_delay = 7000;
            }
          else if(Command == 0x62){
             module0_delay = 5000;         
           }
          else{ (Command == 0x63);
            module0_delay = 9000;
          }
            module0_i = 7; 
            break; 
            
          case 7: // =red | yellow 
               PORTD = RED1 | YELLOW2; 
               module0_delay = 2000; 
               module0_i = 8; 
               break; 
            
          default: 
          module0_i = 0; 
          break; 
      } 
  } 
}
{ ///////////////////////////////// module 1 - Switch 1 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    if (init_module1_clock) { 
      module1_delay = 17; 
      module1_time=millis(); 
      module1_doStep=false; 
      init_module1_clock=false; 
      module1_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module1_time > module1_delay) { 
        module1_time = m; module1_doStep = true; 
      } 
      else module1_doStep = false; 
    } 

    if (module1_doStep) { 
      switch(module1_i){ 
          case 0:         
            B1_State=notPRESSED; 
            total_SW1_counts++; 
            debounce_count1 = module1_time; 
            if (B1_RELEASED) module1_i = 0; 
            else module1_i = 1; 
            break; 
            
          case 1: 
            B1_State=partialPRESS; 
            total_SW1_counts++; 
            if (B1_RELEASED) module1_i = 0; 
            else if (millis()-debounce_count1 < debounce) module1_i = 1; 
            else module1_i = 2; 
            break; 
            
          case 2: 
            B1_State=normalPRESS; 
            total_SW1_counts++;
            if (B1_RELEASED) module1_i = 0; 
            else module1_i = 2; 
            break; 
            
          default: 
              module1_i = 0; 
          break; 
       } 
    } 
  } 
{ ////////////////////////////////////////////// module 2 - Switch 2 debounce/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    if (init_module2_clock) { 
      module2_delay = 17; 
      module2_time=millis(); 
      module2_doStep=false; 
      init_module2_clock=false; 
      module2_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module2_time > module2_delay) { 
        module2_time = m; module2_doStep = true; 
      } 
      else module2_doStep = false; 
    } 

    if (module2_doStep) { 
      
      switch(module2_i){ 
          case 0: 
            B2_State=notPRESSED; 
            total_SW2_counts++; 
            debounce_count2 = module2_time; 
            if (B2_RELEASED) module2_i = 0; 
            else module2_i = 1; 
            break; 
            
          case 1: 
            B2_State=partialPRESS; 
            total_SW2_counts++; 
            if (B2_RELEASED) module2_i = 0; 
            else if (millis()-debounce_count2 < debounce) module2_i = 1; 
            else module2_i = 2; 
            break; 
            
          case 2: 
            B2_State=normalPRESS; 
            total_SW2_counts++; 
            if (B2_RELEASED) module2_i = 0; 
            else module2_i = 2; 
            break; 
            
          
          default: 
          module2_i = 0; 
          break; 
        } 
    } 
  }
{/////////////////////////////////////////////// module 3 maintenance //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (init_module3_clock) { 
      module3_delay = 250; 
      module3_time=millis(); 
      module3_doStep=false; 
      init_module3_clock=false; 
      module3_i = State_off;
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module3_time > module3_delay) { 
        module3_time = m; module3_doStep = true; 
      } 
      else module3_doStep = false; 
    } 
    if (module3_doStep) { 
        displayt();
      switch(module3_i){static unsigned long flash_time;  
          case State_off: // attention = yellow || yellow off
            digitalWrite(Red1, LOW); digitalWrite(Yellow1, LOW); digitalWrite(Green1, LOW);
            digitalWrite(Red2, LOW); digitalWrite(Yellow2, LOW); digitalWrite(Green2, LOW);
            module3_delay = 500; 
            module3_i = State_on;
            break; 
            
         case State_on: // attention = yellow || yellow on
            digitalWrite(Red1, LOW); digitalWrite(Yellow1, HIGH); digitalWrite(Green1, LOW);
            digitalWrite(Red2, LOW); digitalWrite(Yellow2, HIGH); digitalWrite(Green2, LOW);
            module3_delay = 500; 
            module3_i = State_off; 
            break; 
            
          default: 
          module3_i =State_off; 
          break;    
    } 
  }
}   
{ ////////////////////////////////////////////////////////// module 4 - Heartbeat/////////////////////////////////////////////////////////////////////////////////////////////////////// 
    
    if (init_module4_clock) { 
      module4_delay = 340; 
      module4_time=millis(); 
      module4_doStep=false; 
      init_module4_clock=false; 
      module4_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module4_time > module4_delay) { 
        module4_time = m; module4_doStep = true; 
      } 
      else module4_doStep = false; 
    } 

    if (module4_doStep) { 
       switch (module4_i) { 
        case 0: 
          digitalWrite(SEG_DP, HIGH);
          module4_delay = 340;
          module4_i = 1;
          break; 
        case 1: 
          // single line of code here 
          digitalWrite(SEG_DP, LOW);
          module4_delay = 340;
          module4_i=0; 
          break; 
          
        default: 
           module4_i=0; 
       } 
    } 
 }
     
{ ///////////////////////////////// module 5 - barrier Switches //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    if (init_module5_clock) { 
      module5_delay = 20; 
      module5_time=millis(); 
      module5_doStep=false; 
      init_module5_clock=false; 
      module5_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module5_time > module5_delay) { 
        module5_time = m; module5_doStep = true; 
      } 
      else module5_doStep = false; 
    } 

    if (module5_doStep)
    {  
       switch(module5_i){ 
          case 0: 
            LightStatus = 0x49;
            BarrierState = No_Trains; 
            if (B1_State==normalPRESS) module5_i = 1; 
            else{ 
             module5_i = 0;
            }
            break; 

          case 1:
            LightStatus = 0x42;
            BarrierState = Trains; 
            count++;
            timestamp = (millis()+10000);
            module5_i = 21;
            break; 

          case 21:
            if (B1_State==normalPRESS) module5_i = 21;
            else module5_i = 2;
            break;            
            
          case 2: 
            LightStatus = 0x42;
            BarrierState = Trains; 
            if ((long)(millis() - timestamp) > 0) module5_i = 4;
            else
            {
              if (B1_State==normalPRESS) module5_i = 1;
              else if (B2_State==normalPRESS) module5_i = 3;
              else module5_i = 2;
            }
            break;        

          case 3: 
            LightStatus = 0x42;
            count--;
            if (count == 0) module5_i = 0; 
            else if (count > 0) module5_i = 31;
            break;          

          case 31:
            if (B2_State==normalPRESS) module5_i = 31;
            else module5_i = 2;
            break;             

          case 4:
            LightStatus = 0x62; 
            BarrierState = Critical; 
            module5_i = 4;
            break; 
            
          default: 
            module5_i = 0; 
         break; 
       } 
    }
}
{/////////////////////////////////////////////////////////// module 6 - barrier light //////////////////////////////////////////////////////////////////////////////////////////////////
  if (init_module6_clock) { 
      module6_delay = 250; 
      module6_time=millis(); 
      module6_doStep=false; 
      init_module6_clock=false; 
      module6_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module6_time > module6_delay) { 
        module6_time = m; module6_doStep = true; 
      } 
      else module6_doStep = false; 
    } 

    if (module6_doStep) { 
        displayB();
      switch (module6_i) {
    case 0: 
      digitalWrite(Red1, LOW); 
      digitalWrite(Yellow1, LOW); 
      digitalWrite(Green1, LOW);  
      digitalWrite(Red2, LOW); 
      digitalWrite(Yellow2, LOW); 
      digitalWrite(Green2, LOW); 
      if (BarrierState == No_Trains) module6_i = 0; 
      else if (BarrierState == Trains )module6_i = 1;
      break; 
        
    case 1: 
      digitalWrite(Yellow1,HIGH);
      digitalWrite(Yellow2,HIGH);
      module6_delay = 5000;
      module6_i = 2;
      break; 
        
    case 2: 
      digitalWrite(Red1,HIGH);
      digitalWrite(Red2,HIGH);
      digitalWrite(Yellow1,LOW);
      digitalWrite(Yellow2,LOW);
      module6_delay = 500;
      module6_i = 3;
      break; 

    case 3: 
      digitalWrite(Red1,HIGH);
      digitalWrite(Red2,LOW);
      module6_delay = 500;
      module6_i = 4;
      break; 

    case 4: 
      digitalWrite(Red2,HIGH);
      digitalWrite(Red1,LOW);
      module6_delay = 500;
      module6_i = 3;
      if (BarrierState == No_Trains) module6_i = 0;
      if (BarrierState == Critical) module6_i = 5;
      break; 

    case 5:   // reset not occurred
      digitalWrite(Red1,HIGH);
      digitalWrite(Yellow1,HIGH);
      digitalWrite(Red2,LOW);
      digitalWrite(Yellow2,HIGH);
      module6_delay = 500;
      module6_i = 6;
      break;

     case 6:   // reset not occurred
      digitalWrite(Red1,LOW);
      digitalWrite(Red2,HIGH);
      module6_delay = 500;
      module6_i = 5;
      break;              
          
    default:  
      module6_i = 0; 
     break; 
    } 
  }
 }  
{ ///////////////////////////////// module 7 - F1 Switches ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    if (init_module7_clock) { 
      module7_delay = 20; 
      module7_time=millis(); 
      module7_doStep=false; 
      init_module7_clock=false; 
      module7_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module7_time > module7_delay) { 
        module7_time = m; module7_doStep = true; 
      } 
      else module7_doStep = false; 
    } 

    if (module7_doStep)
    {     
      switch(module7_i){ 
          case 0: // Idel
            LightStatus = 0x46;
            F1_State = Idle_1; //Start_Sequence; 
            if(B1_State==normalPRESS) module7_i = 1; 
            else{ 
             module7_i = 0;
            }
            break; 

          case 1: // Start_Sequence
            LightStatus = 0x53;
            F1_State = Start_Sequence;
            if (B1_State == notPRESSED) module7_i = 11; 
            break;

          case 11:
            if(B1_State==normalPRESS) module7_i= 5;
            else if (raceStarted == true) module7_i = 3;
            else module7_i=11;
            break;         
            
          case 2:// Go 
           F1_State = GO;
           module7_i= 3;         
            break;        

          case 3:  
             LightStatus = 0x47;
             F1_State = Green; 
             if(B1_State==normalPRESS) module7_i= 4;
             else if(B2_State==normalPRESS) module7_i=0;
             else module7_i= 3;
             break;          

          case 4: 
              LightStatus = 0x52;
              F1_State = Red_Flag; 
              if(B2_State==normalPRESS) module7_i=0;
              else  module7_i = 4;
            break; 

          case 5:
              LightStatus = 0x41;
              F1_State = Abort;
              if(B2_State==normalPRESS) module7_i=0;
              break;
            
          default: 
            module7_i = 0; 
         break; 
       } 
    }
}

{ ///////////////////////////////// module8 - F1 light ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    if (init_module8_clock) { 
      module8_delay = 250; 
      module8_time=millis(); 
      module8_doStep=false; 
      init_module8_clock=false; 
      module8_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module8_time > module8_delay) { 
        module8_time = m; module8_doStep = true; 
      } 
      else module8_doStep = false; 
    } 


    if (module8_doStep){
       Serial.print("MODULE8 = ");
       Serial.print(module8_i);
       Serial.print(", delay = ");
       Serial.println(module8_delay);
       displayF();
      switch(module8_i){ 
          case 0: 
          raceStarted = false;
          //Serial.println(B1_state,DEC);
            digitalWrite(Red1, LOW); 
            digitalWrite(Yellow1, LOW); 
            digitalWrite(Green1, LOW);  
            digitalWrite(Red2, LOW); 
            digitalWrite(Yellow2, LOW); 
            digitalWrite(Green2, LOW); 
            if (F1_State == Start_Sequence) module8_i = 1;
            else if(F1_State == Idle_1) module8_i = 0;
            break; 

          case 1:
            digitalWrite(Red1, HIGH); 
            digitalWrite(Yellow1, LOW); 
            digitalWrite(Green1, LOW);  
            digitalWrite(Red2, LOW); 
            digitalWrite(Yellow2, LOW); 
            digitalWrite(Green2, LOW); 
            module8_delay = 1000;
            if (F1_State == Abort) module8_i = 9;
            else module8_i = 2; 
            break;

          case 2:
            digitalWrite(Red1, HIGH); 
            digitalWrite(Yellow1, HIGH); 
            digitalWrite(Green1, LOW);  
            digitalWrite(Red2, LOW); 
            digitalWrite(Yellow2, LOW); 
            digitalWrite(Green2, LOW); 
            module8_delay = 1000;
            module8_i = 3;
            if (F1_State == Abort)   module8_i = 9; 
            break;
            
          case 3:
            digitalWrite(Red1, HIGH); 
            digitalWrite(Yellow1, HIGH); 
            digitalWrite(Green1, HIGH);  
          digitalWrite(Red2, LOW); 
          digitalWrite(Yellow2, LOW); 
          digitalWrite(Green2, LOW); 
            module8_delay = 1000;
            module8_i = 4;
            if (F1_State == Abort)   module8_i = 9; 
            break;

          case 4:
            digitalWrite(Red1, HIGH); 
            digitalWrite(Yellow1, HIGH); 
            digitalWrite(Green1, HIGH);  
            digitalWrite(Red2, HIGH); 
            digitalWrite(Yellow2, LOW); 
            digitalWrite(Green2, LOW);  
            module8_delay = 1000;
            module8_i = 5;
            if (F1_State == Abort)   module8_i = 9; 
            break;

          case 5:
            digitalWrite(Red1, HIGH); 
            digitalWrite(Yellow1, HIGH); 
            digitalWrite(Green1, HIGH);  
            digitalWrite(Red2, HIGH); 
            digitalWrite(Yellow2, HIGH); 
            digitalWrite(Green2, LOW);
            module8_delay = random(3000,7000);
            //Serial.println(randNumber); 
            module8_i = 6;
            if(F1_State == Abort)  module8_i = 9;
            break; 
            
          case 6: 
           raceStarted = true;
           digitalWrite(Red1, LOW); 
           digitalWrite(Yellow1, LOW); 
           digitalWrite(Green1, LOW);  
           digitalWrite(Red2, LOW); 
           digitalWrite(Yellow2, LOW); 
           digitalWrite(Green2, LOW);
           module8_delay = 2000; 
           module8_i = 7; 
           break;

         case 7:
           digitalWrite(Green1, HIGH);
           digitalWrite(Green2, HIGH);
           module8_delay = 500;
           if (F1_State == Red_Flag) module8_i = 8;
           else if (F1_State == Idle_1) module8_i = 0;
           else module8_i = 7;
           break; 

          case 8:  
            digitalWrite(Green1, LOW);
            digitalWrite(Green2, LOW);
            digitalWrite(Red1, HIGH); 
            digitalWrite(Red2, HIGH); 
            if (F1_State == Idle) module8_i = 0;
            else module8_i = 8;
            break; 

          case 9: 
            digitalWrite(Red1, LOW); 
            digitalWrite(Red2, LOW); 
            digitalWrite(Yellow1, HIGH);
            digitalWrite(Yellow2, HIGH);
            module8_delay = 500;
           if (F1_State == Idle_1) module8_i = 0;
            else module8_i = 10; 
            break; 

         case 10: 
           digitalWrite(Red1, LOW);
           digitalWrite(Yellow1, LOW); 
           digitalWrite(Green1, LOW);  
           digitalWrite(Red2, LOW); 
           digitalWrite(Yellow2, LOW); 
           digitalWrite(Green2, LOW);
           module8_delay = 500;
          if (F1_State == Idle_1) module8_i = 0;
           else module8_i = 9;    
           break;
           
          default: 
            module8_i = 0; 
         break; 
       } 
    }
}    
{ ////////////////////////////////////// module 9 pelican button controller/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if (init_module9_clock) {
      module9_delay = 3;
      module9_time=millis();
      module9_doStep=false;
      init_module9_clock=false;
      module9_i = 0; // this is the state variable for the button controller
      
    }
    else {
      unsigned long m;
      m=millis();
      if ((long)(m - module9_time) > module9_delay) {
        module9_time = m; module9_doStep = true;
      }
      else module9_doStep = false;
    }

    if (module9_doStep) {
      switch(module9_i){ 
          case 0: 
            demand = false;
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            if (B1_State == normalPRESS || B2_State == normalPRESS) module9_i = 1;
            else module9_i = 0;
            break;
          case 1: 
            demand = false;
            if (granted) module9_i = 0;
            else module9_i = 2;
            break;
          case 2:
            demand = true;
            if (granted) module9_i = 0;
            else module9_i = 2;
            break;
          default: module9_i = 0;
        }
    }
  }

  { ////////////////////////////// module 10 pelican light controller///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    static unsigned long phase_time_pelican_controller; // unsigned long t; in the FSM
    if (init_module10_clock) {
      module10_delay = 250;
      module10_time=millis();
      module10_doStep=false;
      init_module10_clock=false;
      granted = true;
      module10_i = i_State; // this is the state variable for the pelican controller as per specification
      phase_time_pelican_controller = module10_time; // the initial state must have the t=0, effectively t=millis()-phase_time_pelican_controller
    }
    else {
      unsigned long m;
      m=millis();
      if ((long)(m - module10_time) > module10_delay) {
        module10_time = m; module10_doStep = true;
      }
      else module10_doStep = false;
    }

    if (module10_doStep) {
        displayP();
     // Serial.print("MODULE10 ----> "); Serial.println((int)module10_i);
      switch(module10_i){ static unsigned long flash_time; // on-off time for flashing phases
       case i_State: // V=red, P=red
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, HIGH);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            if ((long)(millis()-phase_time_pelican_controller) > 1000) {
              module10_i = ii_State; 
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = i_State;
            break;

          case ii_State: // V=red, yellow, P=red
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, HIGH);digitalWrite(mainYELL, HIGH);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            if ((long)(millis()-phase_time_pelican_controller) > 2000) {
              module10_i = f_State; 
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = ii_State;
            break;
            
          case f_State: // V=green, P=red
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, HIGH);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 8000 && demand) {
              module10_i = a_State;
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
              else if ((long)(millis()-phase_time_pelican_controller) > 8000){
              module10_i = d_State_off;
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = f_State;
            break;
            
          case a_State: // V=amber, P=red
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, HIGH);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 2000) {
              module10_i = b_State; 
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = a_State;
            break;
            
          case b_State: // V=red, P=red
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, HIGH);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 1000) {
              module10_i = c_State; 
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = b_State;
            break;
            
          case c_State: // V=red, P=green
            activeSet = 1;
            LightStatus = 0x54;
            digitalWrite(mainRED, HIGH);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, LOW);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, HIGH);
            SW1_waiting_counts = 0;
            SW2_waiting_counts = 0;
            granted = true;
            if ((long)(millis()-phase_time_pelican_controller) > 4000) {
              module10_i = d_State_off; 
              flash_time = phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else module10_i = c_State;
            break;
            
          case d_State_off: // V=off, P=off
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, LOW);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 3000) {
              module10_i = e_State_off; 
              flash_time = phase_time_pelican_controller = module10_time; // t=0 in the FSM
              }
            else if ((long)(millis()-flash_time) > 500) {
                module10_i = d_State_on;
                flash_time = module10_time;
                }
            else module10_i = d_State_off;
            break;
            
          case d_State_on: // V=amber, P=green
            activeSet = 2; 
            LightStatus = 0x50;
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, HIGH);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, LOW);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, HIGH);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 3000) {
              module10_i = e_State_off; 
              flash_time = phase_time_pelican_controller = module10_time; // t=0 in the FSM
              }
            else if ((long)(millis()-flash_time) > 500) {
                module10_i = d_State_off;
                flash_time = module10_time;
              }
            else module10_i = d_State_on;
            break;
            
          case e_State_off: // V=off, P=red
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, LOW);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, LOW);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 2000) {
              module10_i = f_State;
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else if ((long)(millis()-flash_time) > 500) {
                module10_i = e_State_on;
                flash_time = module10_time;
              }
            else module10_i = e_State_off;
            break;
            
          case e_State_on: // V=amber, P=red
            activeSet = 1; 
            LightStatus = 0x54;
            digitalWrite(mainRED, LOW);digitalWrite(mainYELL, HIGH);digitalWrite(mainGREEN, LOW);
            digitalWrite(secRED, HIGH);digitalWrite(secYELL, LOW);digitalWrite(secGREEN, LOW);
            SW1_waiting_counts = 1;
            SW2_waiting_counts = 1;
            granted = false;
            if ((long)(millis()-phase_time_pelican_controller) > 2000) {
              module10_i = f_State; 
              phase_time_pelican_controller = module10_time; // t=0 in the FSM
            }
            else if ((long)(millis()-flash_time) > 500) {
                module10_i = e_State_off;
                flash_time = module10_time;
              }
            else module10_i = e_State_on;
            break;
            
          default: module10_i = f_State;
          phase_time_pelican_controller = module10_time; // the initial state must have the t=0, effectively t=millis()-phase_time_pelican_controller
        }
    }
  }
{ ////////////////////////////////////// module 11 - MAIN SCHEDULER////////////////////////////////////// 
    
    if (init_module11_clock) { 
      module11_delay = 10; 
      module11_time=millis(); 
      module11_doStep=false; 
      init_module11_clock=false;  
      module11_i=0; 
    } 
    else { 
      unsigned long m; 
      m=millis(); 
      if (m - module11_time > module11_delay) { 
        module11_time = m; module11_doStep = true; 
      } 
      else module11_doStep = false; 
    } 
    if (module11_doStep) {            
      switch (module11_i) {   
        case 0:
          Status = 'N'; 
          init_module0_clock = true;      // SET1 PRIORITY
          init_module1_clock = true; 
          init_module2_clock = true;                     
          init_module3_clock = true;                      
          init_module4_clock = true;      // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = true;                  
          init_module8_clock = true;  
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true; 
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
      }
      break;
      
                        
        case 1:
          Status = 'A';
          CurrentCycle = 0x81;
          NextCycle = 0x92; 
          init_module0_clock = false;      // EQUAL PRIORITY TRAFFIC LIGHT 
          init_module1_clock = true;       // SW1
          init_module2_clock = true;       // SW2              
          init_module3_clock = true;       // MAINTENANCE               
          init_module4_clock = false;      // HEART BEAT
          init_module5_clock = true;       // BARRIER SW             
          init_module6_clock = true;       // BARRIER LIGHT
          init_module7_clock = true;       // F1 SW              
          init_module8_clock = true;       // F1 LIGHT
          init_module9_clock = true;       // PELICAN SW             
          init_module10_clock = true;      // PELICAN LIGHT
          init_module12_clock = true;      // ACCELEROMETER
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }
      if(Command == 0x69) Status = 'A';
      break; 
 
         case 2: 
          Status = 'A';
          CurrentCycle = 0x82;
          NextCycle = 0x93;
          LightStatus = 0x31;
          init_module0_clock = false;      // SET1 PRIORITY
          init_module1_clock = true; 
          init_module2_clock = true;                     
          init_module3_clock = true;                      
          init_module4_clock = false;      // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = true;                     
          init_module8_clock = true;  
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true; 
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }  
      if(Command == 0x69) Status = 'A';
      break;
        
         case 3: 
          LightStatus = 0x32;
          Status = 'A';
          CurrentCycle = 0x83;
          NextCycle = 0x94;  
          init_module0_clock = false;       // SET2 PRIORITY
          init_module1_clock = true; 
          init_module2_clock = true;                     
          init_module3_clock = true;                      
          init_module4_clock = false;      // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = true;                     
          init_module8_clock = true;  
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true; 
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }
      if(Command == 0x69) Status = 'A';  
      break; 
         
        case 4: 
          LightStatus = 0x4D;
          Status = 'A';
          CurrentCycle = 0x84;
          NextCycle = 0x95; 
          init_module0_clock = true;  
          init_module1_clock = true; 
          init_module2_clock = true;                     
          init_module3_clock = false;      // MAINTENANCE                  
          init_module4_clock = false;      // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = true;                     
          init_module8_clock = true;  
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true;
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }
      if(Command == 0x69) Status = 'A';            
      break; 
              
        case 5: 
          Status = 'A';
          CurrentCycle = 0x85;
          NextCycle = 0x96;
          init_module0_clock = true;  
          init_module1_clock = false;    // SW1
          init_module2_clock = false;    // SW2                 
          init_module3_clock = true;                      
          init_module4_clock = false;    // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = true;                     
          init_module8_clock = true;  
          init_module9_clock = false;    // PELICAN SW                
          init_module10_clock = false;   // PELICAN LIGHT
          init_module12_clock = true; 
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
       }
      if(Command == 0x68){
         resetArd();
      }
      if(Command == 0x69) Status = 'A';
      break; 
         
        case 6: 
          Status = 'A';
          CurrentCycle = 0x86;
          NextCycle = 0x97; 
          init_module0_clock = true;  
          init_module1_clock = false;    // SW1
          init_module2_clock = false;    // SW2                 
          init_module3_clock = true;                      
          init_module4_clock = false;    // HEART BEAT
          init_module5_clock = false;    // BARRIER SW                
          init_module6_clock = false;    // BARRIER LIGHT
          init_module7_clock = true;                     
          init_module8_clock = true;  
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true;   
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }  
      if(Command == 0x69) Status = 'A';              
      break; 

        case 7: 
          Status = 'A';
          CurrentCycle = 0x87;
          NextCycle = 0x91;
          init_module0_clock = true;  
          init_module1_clock = false;   // SW1
          init_module2_clock = false;   // SW2                  
          init_module3_clock = true;                      
          init_module4_clock = false;   // HEART BEAT
          init_module5_clock = true;                    
          init_module6_clock = true;  
          init_module7_clock = false;   // F1 SW                  
          init_module8_clock = false;   // F1 LIGHT
          init_module9_clock = true;                    
          init_module10_clock = true; 
          init_module12_clock = true; 
      if((Command >= 0x61) && (Command <= 0x67)){
          module11_i = Command - 0x60;  
        }
      if(Command == 0x68){
         resetArd();
      }
      if(Command == 0x69) Status = 'A';                    
      break; 
            
          default: 
          module11_i = 0; 
      } 
    } 
  }



if( Status == 'A'){ 
     trigger = true;
   }
if( Status == 'N'){ 
    trigger = true;
  } 

  
{/////////////////////////////////////////////// module 12 accelerometer /////////////////////////////////////////////////////////////////////////////
      /* test bench */
      if (init_module12_clock) {
        module12_delay = 30; 
        module12_time=millis();
        module12_doStep=false;
        init_module12_clock=false;
        module12_i=0;    
      }   
      else {
        unsigned long m;
        m=millis();
        if ((long) (m - module12_time) > module12_delay ) {
          module12_time = m; module12_doStep = true;
        }
        else module12_doStep = false;
      }
  
      if (module12_doStep) {
        switch (module12_i) {
          case 0: 
            module12_timestamp = millis()+ 30;               //random(3)
            /* no break here! why is that? */
          case 1: 
            if ((long) (millis() - module12_timestamp) < 0) module12_i = 1;
            else module12_i = 2; 
            break;
          case 2:
            trigger = true;
            /* no break here! why is that? */
          case 3:
            if (state != TriggerIamMaster) module12_i = 3;
            else module12_i = 4; 
            break;
          case 4:
              if (FirstTime == true){
                Wire.begin();
                Wire.beginTransmission(MPU_addr); 
                Wire.write(0x6B);                                                         // PWR_MGMT_1 register 
                Wire.write(0);                                                            // set to zero (wakes up the MPU-6050) 
                Wire.endTransmission(true);
                Serial.print(9600);                
                FirstTime = false;
              }
                 
                Wire.beginTransmission(MPU_addr); 
                Wire.write(ACCEL_XOUT_HI);
                Wire.write(TEMP_OUT_H);                                                  // starting with register TEMP_OUT_H
                Wire.write(TEMP_OUT_L);                                                  // starting with register TEMP_OUT_L                                            // starting with register 0x3B (ACCEL_XOUT_H) 
                Wire.endTransmission(false); 
                Wire.requestFrom(MPU_addr,4,true);                                       // request a total of 4 registers             
                AcX=Wire.read()<<8|Wire.read();                                          
                AcY=Wire.read()<<8|Wire.read();  
                AcZ=Wire.read()<<8|Wire.read(); 
                T=Wire.read()<<8|Wire.read(); 
                T = (TEMP_OUT/340.00) + 36.53;
                
                Serial.print("AcX = "); Serial.print(AcX);     
                Serial.print(" | AcY = "); Serial.print(AcY); 
                Serial.print(" | AcZ = "); Serial.println(AcZ); 

          if(AcZ > 14000){  
                TiltStatus = 0x46;
                Serial.print("Good");  
                result = 'G'; 
               }
          else{
               TiltStatus = 0x46;
               Serial.print("Danger");
               result = 'D';
             } 

              trigger = false;
              module12_i = 0;
              break; 
        }
     }
  }   
}  
void requestEvent() {
        // Wire.write((char)result); /*send string on request */     
         ACK[0] = Status;
         ACK[1] = CurrentCycle;                          
         ACK[2] = NextCycle;
         ACK[3] = LightStatus; 
         ACK[4] = SW1_waiting_counts;
         ACK[5] = SW2_waiting_counts;
         ACK[6] =total_SW1_counts;
         ACK[7] =total_SW1_counts;
         ACK[8] = TiltStatus;
         ACK[9] = T;
         
         Wire.write(ACK[0]);
         Wire.write(ACK[1]);
         Wire.write(ACK[2]);
         Wire.write(ACK[3]);
         Wire.write(ACK[4]);
         Wire.write(ACK[5]);
         Wire.write(ACK[6]);
         Wire.write(ACK[7]);
         Wire.write(ACK[8]);
         Wire.write(ACK[9]);

}

void receiveEvent(int howMany){ 
  while (0 <Wire.available()){    
    Command = Wire.read();             /* receive byte as a character */    
    Serial.println(Command);           /* print the character */  
  }
}
