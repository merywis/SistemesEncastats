//INCLUDEs
#include "HIB.h"
#include "SO.h"
#include "timerConfig.h"
#include <mcp_can_uib.h>
#include <mcp_can_uib_dfs.h>
#include "Terminal.h"


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

HIB hib;
SO so;
Terminal term;

MCP_CAN CAN(SPI_CS_PIN);
/******************************************************************************/
/** DEFINE ********************************************************************/
/******************************************************************************/
#define NUM_ROOM 4
#define PRIO_TASK_STATE 1
#define PRIO_TASK_KeyDetector 2
#define PRIO_TASK_ShareAdcValue 3
#define PRIO_TASK_InsertTemp 4
#define PRIO_TASK_InsertComandos 5
#define PRIO_TASK_SHARE 6


#define CAN_ID_TEMP_EXT 1
#define CAN_ID_TEMP 2
#define CAN_ID_TIME 3


#define PERIOD_TASK_STATE 3
#define PERIOD_TASK_INSERTCOMANDOS 2


/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile uint8_t key = 0;
volatile float rxTemp = 0.0; //tienen q ser globales ? :(
volatile boolean rxTime = false; //tienen q ser globales ? :(

volatile float TempExt = 0.0; //tienen q ser globales ? :(
volatile boolean MomentDay = false;

// stores every new adc adquired value
// shared between: adchook (writes) and ShareAdcValue (reads)
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

Flag fCANEvent;
const unsigned char maskRxTempExtEvent = 0x01; // represents new adc value adquired
const unsigned char maskRxTimeEvent = 0x02; // represents new adc value adquired


/*********
  Declaration of semaphores
*********/
Sem sSampledAdc;
Sem sTime;
Sem sTempExt;
Sem sGoal;

/*********
  Declaration of mailboxes
*********/
MBox mbNumRoom;


/******************************************************************************/
/** Definition of own types ***************************************************/
/******************************************************************************/

struct structRoom
{
  float tempGoal = 0.0; //
  float tempInt = 0.0; //
  uint8_t actuacion = 0;
  float tempMaxDay = 0.0;
  float tempMinDay = 0.0;
  float tempMaxNight = 0.0;
  float tempMinNight = 0.0;
};
typedef structRoom typeRoom;


struct structState
{
  float tempExt = 0.0; // Number of edition
  boolean momentDay = true; // true = day & false = night
  typeRoom datosRoom1;
  typeRoom datosRoom2;
  typeRoom datosRoom3;
  typeRoom datosRoom4;
};
typedef structState typeState;


struct structGoal
{
  uint8_t numRoom = 0; // Number of edition
  uint8_t tempGoal = 0; // Number of edition
};
typedef structGoal typeGoal;

typeGoal Goal;

struct structLimitesRoom {
  uint8_t maxDay = 0; // Number of edition
  uint8_t maxNight = 0; // Number of edition
  uint8_t minDay = 0; // Number of edition
  uint8_t minNight = 0; // Number of edition
};
typedef structLimitesRoom typeLimitesRoom;


struct structLimites
{
  typeLimitesRoom limitesRoom1;
  typeLimitesRoom limitesRoom2;
  typeLimitesRoom limitesRoom3;
  typeLimitesRoom limitesRoom4;

};
typedef structLimites typeLimites;


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
      case CAN_ID_TEMP:
        CAN.getRxMsgData((byte*) &rxTemp);
        so.setFlag(fCANEvent, maskRxTempExtEvent);
        break;

      case CAN_ID_TIME:
        CAN.getRxMsgData((byte*) &rxTime);
        so.setFlag(fCANEvent, maskRxTimeEvent);
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

void State()
{
  typeState state;
  uint8_t numRoom = 0;
  unsigned long nextActivationTick;

  nextActivationTick = so.getTick();

  while (true)
  {

    // Read adcValue (shared with ShareAdcValue)
    so.waitSem(sTempExt);

    state.tempExt = TempExt;

    so.signalSem(sTempExt);

    so.waitSem(sTime);

    state.momentDay = MomentDay;

    so.signalSem(sTime);

    for (int i = 1; i < NUM_ROOM; i++) {

      so.waitSem(sGoal);

      numRoom = Goal.numRoom;

      so.signalSem(sGoal);

    if (numRoom == i){
       state.datosRoomi.tempGoal = Goal.tempGoal;
    }
     
    }






    nextActivationTick = nextActivationTick +  PERIOD_TASK_STATE; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}


void Share() {
  // define a mask to awake either by maskAlarm or maskNotAlarm
  unsigned char mask = (maskRxTempExtEvent | maskRxTimeEvent);
  // var to read flag value
  unsigned char flagValue;

  while (true)
  {
    // Wait until any of the bits of the flag fSincro
    // indicated by the bits of maskAlarm or maskNotAlarm are set to '1'
    so.waitFlag(fCANEvent, mask);

    // read flag Value
    flagValue = so.readFlag(fCANEvent);

    // Clear the mask bits of flag fSincro to not process the same event twice
    so.clearFlag(fCANEvent, mask);

    // distinguish whether the event has been alarm or not alarm
    switch (flagValue)
    {
      case maskRxTempExtEvent:
        Serial.println("maskRxTempExtEvent");

        // Read adcValue (shared with ShareAdcValue)
        so.waitSem(sTempExt);

        TempExt = rxTemp;
        hib.ledToggle(2);

        so.signalSem(sTempExt);
        break;

      case maskRxTimeEvent:
        Serial.println("maskRxTimeEvent");
        so.waitSem(sTime);

        MomentDay = rxTime;
        hib.ledToggle(2);

        so.signalSem(sTime);
        break;

      default:
        break;
    }

  }
}

void InsertTemp() {

  uint8_t * rxNumRoomMessage;

  char str[16];

  while (true) {
    // Wait until receiving the new state from TaskState
    so.waitMBox(mbNumRoom, (byte**) &rxNumRoomMessage);
    hib.ledToggle(1);

    Goal.numRoom = *rxNumRoomMessage;
    Goal.numRoom = Goal.numRoom + 1;

    sprintf(str, "%u", Goal.numRoom);
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



void ShareAdcValue() {

  char str[16];
  char floatBuffer[6];
  char charBuff[10];

  uint16_t auxAdcValue;
  float auxSampledAdc;

  while (1)
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



void KeyDetector() {

  char str[16];

  while (1)
  {
    // Wait until any of the bits of the flag fExtEvent
    // indicated by the bits of maskAdcEvent are set to '1'
    so.waitFlag(fKeyEvent, maskKeyEvent);


    if (key == 0 || key == 1 || key == 2 || key == 3) {
      so.signalMBox(mbNumRoom, (byte*) &key);
    } else if (key == 8) { //Caso Tª exterior-> enviar valor key a través de CAN
      // Send sensor via CAN
      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_TEMP_EXT, CAN_EXTID, sizeof(uint8_t), &key);
    } else if (key == 7) {

    }

    // Clear the flag fExtEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);
  }
}

void InsertComandos()
{
  char c;
  uint8_t arraylimitesRoom[4];
  uint8_t i = 0;
  unsigned long nextActivationTick;

  nextActivationTick = so.getTick();
  while (true)
  {

    // Check and read character received from UART if any
    c = term.getChar(false);

    // Check whether or not a byte has been received from UART
    if (c != term.NO_CHAR_RX_UART)
    {
      arraylimitesRoom[i] = c;
      i++;

      // Sent byte back to the PC
      term.print(c);
    }
    if (i == 4) {
      //Semaforo
      term.print( arraylimitesRoom[0]);
      term.print( arraylimitesRoom[1]);
      term.print( arraylimitesRoom[2]);
      term.print( arraylimitesRoom[3]);
      i = 0;
    }


    nextActivationTick = nextActivationTick +  PERIOD_TASK_INSERTCOMANDOS; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}

/******************************************************************************/
/** Setup *********************************************************************/
/******************************************************************************/
void setup() {
  //Init
  Serial.begin(115200); // SPEED

  //Init terminal
  term.begin(115200);
  term.clear();
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

  while (true) {

    Serial.println(" ");
    Serial.println(" MAIN ");
    //Prints para la tarea InsertComandos: ¿?¿?¿? maria: a mi no me gusta que esté aquí
    Serial.println("Hola :)! Introduce el número de habitación + límite superior + límite inferior + momento del día");
    Serial.println("Por ejemplo: 2 27 17 dia");

    // Definition and initialization of semaphores
    sSampledAdc = so.defSem(1); // intially accesible
    sTime = so.defSem(1); // intially accesible
    sTempExt = so.defSem(1); // intially accesible
    sGoal = so.defSem(1); // intially accesible


    // Definition and initialization of mailboxes
    mbNumRoom = so.defMBox();

    // Definition and initialization of flags
    fKeyEvent = so.defFlag();
    fAdcEvent = so.defFlag();
    fCANEvent = so.defFlag();

    // Definition and initialization of tasks

    so.defTask(State, PRIO_TASK_STATE);
    so.defTask(KeyDetector, PRIO_TASK_KeyDetector);
    so.defTask(ShareAdcValue, PRIO_TASK_ShareAdcValue);
    so.defTask(InsertTemp, PRIO_TASK_InsertTemp);
    so.defTask(InsertComandos, PRIO_TASK_InsertComandos);
    so.defTask(Share, PRIO_TASK_SHARE);


    //Set up adc
    hib.adcSetTimerDriven(TIMER_TICKS_FOR_125ms, (tClkPreFactType) TIMER_PSCALER_FOR_125ms, adcHook);

    //Set up timer 5 so that the SO can regain the CPU every tick
    hib.setUpTimer5(TIMER_TICKS_FOR_125ms, TIMER_PSCALER_FOR_125ms, timer5Hook);

    // Start mutltasking (program does not return to 'main' from hereon)
    so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
  }

}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
