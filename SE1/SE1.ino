//INCLUDEs
#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"
HIB hib;
SO so;


/******************************************************************************/
/** DEFINE ********************************************************************/
/******************************************************************************/

#define PRIO_TASK_KeyDetector 2
#define PRIO_TASK_ShareAdcValue 3
#define PRIO_TASK_InsertTemp 4



/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile uint8_t key = 0;
volatile uint16_t adcValue = 0;

/********************************
  Declaration of flags and masks
*********************************/

// flag (set of bits) for external events
Flag fKeyEvent;
Flag fAdcEvent;

// Masks
const unsigned char maskKeyEvent = 0x01; // represents new adc value adquired
const unsigned char maskAdcEvent = 0x01; // represents new adc value adquired 


/*********
  Declaration of semaphores
*********/ 
Sem sUART;

/*********
  Declaration of mailboxes
*********/ 
MBox mbNumRoom;


/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/



/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/


/*****************
  KEYPAD hook
******************/ 
void keyHook(uint8_t newKey)
{
  
  key = newKey;

  so.setFlag(fKeyEvent, maskKeyEvent);  
}


/*****************
  ADCISR
******************/ 

void adcHook(uint16_t newAdcAdquiredValue)
{
  adcValue = newAdcAdquiredValue;

  hib.ledToggle(2); // for debugging

  // Awake task A by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
  so.setFlag(fAdcEvent, maskAdcEvent);

}

/******************************************************************************/
/** ISR *********************************************************************/
/******************************************************************************/



/******************************************
  TASKS declarations and implementations
*******************************************/ 

 /* Task states

  DESTROYED / uninitalized
  BLOCKED / WAITING FOR SEMAPHORE-MAILBOX-FLAG
  AUTO-SUSPENDED BY TIME
  ACTIVE / ELIGIBLE / READY
  RUNNING

*/

void State(int x){
  
}


void InsertTemp(){
  
  uint8_t NumRoom;
  uint8_t * rxNumRoomMessage;
  
    while(true){ 
      so.waitMBox(mbNumRoom,(byte**) &rxNumRoomMessage);
      hib.ledToggle(1);  
    }
}



void ShareAdcValue(){

  char str[16];

  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fAdcEvent, maskAdcEvent);

    // Print the new adc adquired value
    sprintf(str,"%u", adcValue);
    hib.lcdClear();
    hib.lcdPrint(str);

    
    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fAdcEvent, maskAdcEvent);
  }  
}



void KeyDetector(){
  
  char str[16];

  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fKeyEvent, maskKeyEvent);

    // Print the new adc adquired value
    sprintf(str,"%u", key);
    hib.lcdClear();
    hib.lcdPrint(str);
    
    if(key == 0 || key == 1 || key == 2 || key == 3){
      so.signalMBox(mbNumRoom, (byte*) &adcValue);
    }
    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);
  }  
}

/******************************************************************************/
/** Setup *********************************************************************/
/******************************************************************************/
void setup() {
  //Init
  Serial.begin(115200); // SPEED
  
  //Init terminal

  //Init hib
  hib.begin();

  //Init SO
  so.begin();
  
  //Clear LCD
  hib.lcdClear();
  
  // Init can bus

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.

  
}


void loop() {
  // TX consts and vars

  // Rx vars


  hib.keySetIntDriven(100, keyHook);
  
  //Set up adc
  //hib.adcSetTimerDriven(TIMER_TICKS_FOR_125ms,(tClkPreFactType) TIMER_PSCALER_FOR_125ms, adcHook);
  
  while (true){

    Serial.println(" ");
    Serial.println(" MAIN ");

    // Definition and initialization of flags
    fKeyEvent = so.defFlag();
    fAdcEvent = so.defFlag();
   
     // Definition and initialization of tasks
    so.defTask(KeyDetector, PRIO_TASK_KeyDetector);
    so.defTask(ShareAdcValue, PRIO_TASK_ShareAdcValue);
    so.defTask(InsertTemp, PRIO_TASK_InsertTemp);

    //
    mbNumRoom = so.defMBox();
    
  
    // Start mutltasking (program does not return to 'main' from hereon)
   so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
  }
  
}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
