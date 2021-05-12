//INCLUDEs
#include "HIB.h"
#include "SO.h"
HIB hib;
SO so;


/******************************************************************************/
/** DEFINE ********************************************************************/
/******************************************************************************/

#define PRIO_TASK_A 2

/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile uint16_t adcValue = 0;

// flag (set of bits) for external events
Flag fExtEvent;

// This mask refers to the LSB of fExtEvent
const unsigned char maskKeyEvent = 0x01; // represents new adc value adquired

/********************************
  Declaration of flags and masks
*********************************/



/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/



/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/



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


void SimulateTempInt(int x){
  
}



void InsertComandos(int x)
{
  
}



void KeyDetector(int x)
{
  char str[16];

  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fExtEvent, maskKeyEvent);

    // Print the new adc adquired value
    sprintf(str,"%u", adcValue);
    hib.lcdClear();
    hib.lcdPrint(str);
    
    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fExtEvent, maskKeyEvent);
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

  while (true){

    Serial.println(" ");
    Serial.println(" MAIN ");

    // Definition and initialization of flags
    fExtEvent = so.defFlag();
   
     // Definition and initialization of tasks
    so.defTask(KeyDetector, PRIO_TASK_A);
  
    // Start mutltasking (program does not return to 'main' from hereon)
   
  }
  
}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
