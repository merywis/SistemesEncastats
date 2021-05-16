//INCLUDEs
#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

HIB hib;
SO so;
MCP_CAN CAN(SPI_CS_PIN);


/******************************************************************************/
/** DEFINE ********************************************************************/
/******************************************************************************/

#define PRIO_TASK_PrintTemp 2
#define CAN_ID_TEMP_EXT 1
/*
#define PRIO_TASK_ShareAdcValue 3
#define PRIO_TASK_InsertTemp 4

*/


/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile uint8_t key = 0;
/*
volatile uint16_t adcValue = 0;
*/
/********************************
  Declaration of flags and masks
*********************************/
/*
// flag (set of bits) for external events
Flag fKeyEvent;
Flag fAdcEvent;

// Masks
const unsigned char maskKeyEvent = 0x01; // represents new adc value adquired
const unsigned char maskAdcEvent = 0x01; // represents new adc value adquired 

*/
 // flag (set of bits) for CAN reception events
Flag fCANPrintEvent;

  // refers to the LSB bit of fCANevent
const unsigned char maskRxPrintTExtEvent = 0x01; // represents reception of sensor via CAN
/*********
  Declaration of semaphores
*********/ 


/*********
  Declaration of mailboxes
*********/ 


/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/



/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/


/*****************
  KEYPAD hook
******************/ 
/*void keyHook(uint8_t newKey)
{
  
  key = newKey;

  so.setFlag(fKeyEvent, maskKeyEvent);  
}*/


/*****************
  ADCISR
******************/ 
/*
void adcHook(uint16_t newAdcAdquiredValue)
{
  adcValue = newAdcAdquiredValue;

  hib.ledToggle(2); // for debugging

  // Awake task A by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
  so.setFlag(fAdcEvent, maskAdcEvent);

}*/

/******************************************************************************/
/** ISR *********************************************************************/
/******************************************************************************/

/*****************
  CANISR
******************/ 

void isrCAN()
{
  const uint8_t RX_LED = 4;
  char auxSREG;

  // Save the AVR Status Register by software
  // since the micro does not do it automatically
  auxSREG = SREG;
////////

  if (CAN.rxInterrupt())
  {
    hib.ledToggle(RX_LED); // for debugging
    
    CAN.readRxMsg();
    switch(CAN.getRxMsgId())
    {
      case CAN_ID_TEMP_EXT:
        CAN.getRxMsgData((byte*) &key);
        so.setFlag(fCANPrintEvent, maskRxPrintTExtEvent);
        break;

      default:
        break;
    }
  }

/////
  // Restore the AVR Status Register by software
  // since the micro does not do it automatically
  SREG = auxSREG;
}

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

void PrintTemp()
{
  uint16_t auxKey;
  
  while(true){
     // Wait until any of the bits of the flag fCANevent
    // indicated by the bits of maskRxSensorEvent are set to '1'        
    so.waitFlag(fCANPrintEvent, maskRxPrintTExtEvent);
          // Clear the maskRxSensorEvent bits of flag fCANEvent to not process the same event twice
    so.clearFlag(fCANPrintEvent, maskRxPrintTExtEvent);
    auxKey = key;
    Serial.println("heyyyyy");
    Serial.println(auxKey);

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
  
  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");

  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, isrCAN, FALLING);
  
}


void loop() {
  // TX consts and vars

  // Rx vars


  while (true){

    Serial.println(" ");
    Serial.println(" MAIN ");

    // Definition and initialization of flags
    fCANPrintEvent = so.defFlag();
   
     // Definition and initialization of tasks
    so.defTask(PrintTemp, PRIO_TASK_PrintTemp);

   
    
  
    // Start mutltasking (program does not return to 'main' from hereon)
   so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
   
  }
  
}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
