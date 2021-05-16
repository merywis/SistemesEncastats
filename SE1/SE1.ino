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

#define PRIO_TASK_KeyDetector 2
#define PRIO_TASK_ShareAdcValue 3
#define PRIO_TASK_InsertTemp 4


#define CAN_ID_TEMP_EXT 1

/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile uint8_t key = 0;

// stores every new adc adquired value
// shared between: adhook (writes) and ShareAdcValue (reads)
volatile uint16_t adcValue = 0; // critical region beetween adcHook and ShareAdcValue

// stores the last sampled value of the sensor
// calculated upon adcValue
// shared between ShareAdcValue (writes) and InsertTemp (reads)
volatile float sampledAdc = 0.0; // critical region between ShareAdcValue and InsertTemp

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
Sem sSampledAdc;

/*********
  Declaration of mailboxes
*********/ 
MBox mbNumRoom;


/******************************************************************************/
/** Definition of own types ***************************************************/
/******************************************************************************/

struct structGoal
{
      uint8_t numRoom = 0; // Number of edition
      uint8_t tempGoal = 0; // Number of edition
};

typedef structGoal typeGoal;



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

  hib.ledToggle(4); // for debugging

  // Awake task A by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
  so.setFlag(fAdcEvent, maskAdcEvent);

}

/*****************
  TICKISR
******************/ 

// Hook FOR TICKISR
void timer5Hook ()
{
  so.updateTime(); // Call SO to update time (tick) and manage tasks
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
 
  uint8_t * rxNumRoomMessage;

  typeGoal Goal;
  char str[16];
  
    while(true){ 
      // Wait until receiving the new state from TaskState
      so.waitMBox(mbNumRoom,(byte**) &rxNumRoomMessage);
      hib.ledToggle(1);
      
      Goal.numRoom = *rxNumRoomMessage;
      Goal.numRoom = Goal.numRoom + 1;

      sprintf(str,"%u", Goal.numRoom);
      hib.lcdClear();
      hib.lcdPrint(str);
        
      // Read adcValue (shared with ShareAdcValue)
      so.waitSem(sSampledAdc);
      
      Goal.tempGoal = sampledAdc;
      hib.ledToggle(2);
      
      so.signalSem(sSampledAdc);

    Serial.print("Habitacion: ");
    Serial.println(Goal.numRoom);
    
    Serial.print("Temperatura: ");
    Serial.println(Goal.tempGoal);
    }
}



void ShareAdcValue(){

  char str[16];
  char floatBuffer[6];
  char charBuff[10];
  
  uint16_t auxAdcValue;
  float auxSampledAdc;

  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fAdcEvent, maskAdcEvent);

    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fAdcEvent, maskAdcEvent);
    
    auxAdcValue = adcValue;
    // Get value and map it in range [-10, 40] -> mapeam es valor q estava entre (0,1023) a [-10, 40]
    //1023 * x = 50 -> x = 0,04887586
    auxSampledAdc = (((float) auxAdcValue) * 0.04887585) - 10;
    
    //Serial.println(auxSampledAdc);

    /*
    dtostrf(auxSampledAdc, 3, 2, floatBuffer);
    sprintf(charBuff,"Temp : %d", floatBuffer);
    hib.lcdPrint(charBuff);

    delay(5000);
    hib.lcdClear();
    */
    
    so.waitSem(sSampledAdc);
    
    sampledAdc = auxSampledAdc;
       
    so.signalSem(sSampledAdc);

    
  }  
}



void KeyDetector(){
  
  char str[16];

  while(1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'        
    so.waitFlag(fKeyEvent, maskKeyEvent);

    
    if(key == 0 || key == 1 || key == 2 || key == 3){
      so.signalMBox(mbNumRoom, (byte*) &key);
    } else if (key == 8) { //Caso Tª exterior-> enviar valor key a través de CAN
      Serial.println("hola2");
      // Send sensor via CAN
    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      CAN.sendMsgBufNonBlocking(CAN_ID_TEMP_EXT, CAN_EXTID, sizeof(uint8_t), &key);
      Serial.println("hola");
    }else if (key == 7){
      
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
  
  // Init can bus : baudrate = 500k, loopback mode, enable reception and transmission interrupts
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, false, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.

  
}


void loop() {


 

  hib.keySetIntDriven(100, keyHook);
  

  
  while (true){

    Serial.println(" ");
    Serial.println(" MAIN ");

    // Definition and initialization of semaphores
  sSampledAdc = so.defSem(1); // intially accesible

  // Definition and initialization of mailboxes
   mbNumRoom = so.defMBox();

    // Definition and initialization of flags
    fKeyEvent = so.defFlag();
    fAdcEvent = so.defFlag();
   
     // Definition and initialization of tasks
    so.defTask(KeyDetector, PRIO_TASK_KeyDetector);
    so.defTask(ShareAdcValue, PRIO_TASK_ShareAdcValue);
    so.defTask(InsertTemp, PRIO_TASK_InsertTemp);

    //Set up adc
  hib.adcSetTimerDriven(TIMER_TICKS_FOR_125ms,(tClkPreFactType) TIMER_PSCALER_FOR_125ms, adcHook);
  
    // Start mutltasking (program does not return to 'main' from hereon)
   so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
  }
  
}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
