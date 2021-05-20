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


#define CAN_ID_PRINT_TEMP 1
#define CAN_ID_TEMP_EXT 2
#define CAN_ID_LIGHT 3


#define PRIO_TASK_DETECT_TEMP_EXT 3
#define PRIO_TASK_TIMETABLE_HEATING 4
#define PRIO_TASK_SHARE_TIME 5

#define PERIOD_TASK_DETECT_TEMP_EXT 5
#define PERIOD_TASK_TIMETABLE_HEATING 5



/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile float TempExt = 0.0;
volatile boolean momentDay;

/********************************
  Declaration of flags and masks
*********************************/

// flag (set of bits) for CAN reception events
Flag fCANPrintEvent;


// Masks
const unsigned char maskRxPrintTExtEvent = 0x01; // represents reception of sensor via CAN


/*********
  Declaration of semaphores
*********/
Sem sTempExt;
Sem sTime;
Sem sLCD;

/*********
  Declaration of mailboxes
*********/
MBox mbLight;

/******************************************************************************/
/** Definition of own types ***************************************************/
/******************************************************************************/

struct structMessageTemp
{
  byte typeInfo;
  byte numRoom;
  float tempInt;
};
typedef structMessageTemp typeMessageTemp;

typeMessageTemp rxMessageTemp;
/******************************************************************************/
/** Additional functions prototypes *******************************************/
/******************************************************************************/



/******************************************************************************/
/** Hooks *********************************************************************/
/******************************************************************************/


/*****************
  KEYPAD hook
******************/



/*****************
  ADCISR
******************/


/******************************************************************************/
/** ISR *********************************************************************/
/******************************************************************************/

/*****************
  TICKISR
******************/

// Hook FOR TICKISR
void timer5Hook ()
{
  so.updateTime(); // Call SO to update time (tick) and manage tasks
}

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
    switch (CAN.getRxMsgId())
    {
      case CAN_ID_PRINT_TEMP:
        CAN.getRxMsgData((byte*) &rxMessageTemp);
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
  //*************porq tenemos q usar una global.
  typeMessageTemp auxRxMessageTemp;

  float auxTempExt;
  boolean auxMomentDay;
  String momentOfDay;
 char strDay[3] = "day";
 char strNight[5] = "night";
 
  while (true) {
    // Wait until any of the bits of the flag fCANevent
    // indicated by the bits of maskRxSensorEvent are set to '1'
    so.waitFlag(fCANPrintEvent, maskRxPrintTExtEvent);
    // Clear the maskRxSensorEvent bits of flag fCANEvent to not process the same event twice
    so.clearFlag(fCANPrintEvent, maskRxPrintTExtEvent);

    auxRxMessageTemp = rxMessageTemp;
    if (auxRxMessageTemp.typeInfo == 8) { //tempExt

      so.waitSem(sTempExt);

      auxTempExt = TempExt;

      so.signalSem(sTempExt);



    } else if (auxRxMessageTemp.typeInfo == 7) {


    }


    so.waitSem(sTime);

    auxMomentDay = momentDay;

    so.signalSem(sTime);


 
  if(auxMomentDay == true){
    hib.lcdPrint("dia");
  }else{
   hib.lcdPrint("noche");
  }
  
    // The LCD is a critial region itself (shared between PrintTemp and Alarm)
    so.waitSem(sLCD);
   // hib.ledToggle(5);
    //sprintf(str, "%u", auxTempExt);
    
    hib.lcdClear();
    //substituir por un char 
    hib.lcdPrint("holi");
    
    so.signalSem(sLCD);

  }
}

void DetectTempExt()
{
  float celsius;
  unsigned long nextActivationTick;

  nextActivationTick = so.getTick();
  while (true)
  {

    celsius = hib.temReadCelsius(hib.LEFT_TEM_SENS);

    Serial.print("Left temperature sensor -> celsius = ");
    Serial.println(celsius);
    hib.ledToggle(2); // for debugging

    so.waitSem(sTempExt);

    TempExt = celsius;

    so.signalSem(sTempExt);


    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      CAN.sendMsgBufNonBlocking(CAN_ID_TEMP_EXT, CAN_EXTID, sizeof(float), (INT8U *)  &celsius);

    nextActivationTick = nextActivationTick +  PERIOD_TASK_DETECT_TEMP_EXT; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }

}
void TimetableHeating()
{
  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();
  uint16_t ldrAdcValue;
  boolean light;

  while (true)
  {
    // Sample left-handed LDR (Light-Dependant Resistor) sensor
    ldrAdcValue = hib.ldrReadAdc(hib.LEFT_LDR_SENS); //Aquí mos retorna el valor en el rang
    //de ADC i noltros haurem de mapear-ho en els valors que necessitem.
    Serial.print("Left ldr sensor -> adc = ");
    Serial.println(ldrAdcValue);
    if (ldrAdcValue < 512) {
      light = false;
    } else {
      light = true;
    }

    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      CAN.sendMsgBufNonBlocking(CAN_ID_LIGHT, CAN_EXTID, sizeof(boolean), (INT8U *) &light);

    nextActivationTick = nextActivationTick +  PERIOD_TASK_TIMETABLE_HEATING; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }

}

void ShareTime()
{

  boolean * rxLight;

  while (true) {
    so.waitMBox(mbLight, (byte**) &rxLight);

    so.waitSem(sTime);

    momentDay = *rxLight;

    so.signalSem(sTime);

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


  while (true) {

    Serial.println(" ");
    Serial.println(" MAIN ");

    // Definition and initialization of semaphores
    sTempExt = so.defSem(1); // intially accesible
    sTime = so.defSem(1); // intially accesible
    sLCD = so.defSem(1); // intially accesible

    // Definition and initialization of mailboxes
    mbLight = so.defMBox();

    // Definition and initialization of flags
    fCANPrintEvent = so.defFlag();

    // Definition and initialization of tasks
    so.defTask(PrintTemp, PRIO_TASK_PrintTemp);
    so.defTask(DetectTempExt, PRIO_TASK_DETECT_TEMP_EXT);
    so.defTask(TimetableHeating, PRIO_TASK_TIMETABLE_HEATING);
    so.defTask(ShareTime, PRIO_TASK_SHARE_TIME);


    //Set up timer 5 so that the SO can regain the CPU every tick
    hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);


    // Start mutltasking (program does not return to 'main' from hereon)
    so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER

  }

}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
