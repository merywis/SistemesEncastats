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

#define CAN_ID_PRINT_TEMP 1
#define CAN_ID_TEMP_EXT 2
#define CAN_ID_LIGHT 3
#define CAN_ID_ALARM 4

#define PRIO_TASK_PrintTemp 2
#define PRIO_TASK_DETECT_TEMP_EXT 1
#define PRIO_TASK_TIMETABLE_HEATING 1
#define PRIO_TASK_SHARE_TIME 2
#define PRIO_TASK_ALARM 2

#define PERIOD_TASK_DETECT_TEMP_EXT 1
#define PERIOD_TASK_TIMETABLE_HEATING 1


#define DO 1
#define RE 3
#define MI 5
#define FA 6
#define SOL 8
#define LA 10
#define SI 12


/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/

volatile float TempExt = 0.0;
volatile boolean momentDay;
uint8_t rxNumRoom;

/********************************
  Declaration of flags and masks
*********************************/

// flag (set of bits) for CAN reception events
Flag fCANPrintEvent;
Flag fAlarmEvent;


// Masks
const unsigned char maskRxPrintEvent = 0x01; // represents reception of sensor via CAN
const unsigned char maskAlarmEvent = 0x01; // represents reception of sensor via CAN

/*********
  Declaration of semaphores
*********/
Sem sTempExt;
Sem sTime;
Sem sLCD;
Sem sCanCtrl;

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
    //hib.ledToggle(RX_LED); // for debugging

    CAN.readRxMsg();
    switch (CAN.getRxMsgId())
    {
      case CAN_ID_PRINT_TEMP:
        Serial.println("isr caso temp");
        CAN.getRxMsgData((byte*) &rxMessageTemp);
        so.setFlag(fCANPrintEvent, maskRxPrintEvent);
        break;

      case CAN_ID_ALARM:
        Serial.println("isr caso alarma");
        CAN.getRxMsgData((byte*) &rxNumRoom);
        so.setFlag(fAlarmEvent, maskAlarmEvent);
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
  //char strDay[3] = {'d','a','y'};
  //char strNight[5] ={'n','i','g','h','t'};
  char momDay[5];
  float arrayTempInt[4];

  char charBuff[10];
  char floatBuffer[6];

  while (true) {
    // Wait until any of the bits of the flag fCANPrintEvent
    // indicated by the bits of maskRxPrintEvent are set to '1'
    so.waitFlag(fCANPrintEvent, maskRxPrintEvent);

    // Clear the maskRxPrintEvent bits of flag fCANPrintEvent to not process the same event twice
    so.clearFlag(fCANPrintEvent, maskRxPrintEvent);
    auxRxMessageTemp = rxMessageTemp;

    so.waitSem(sTime);

    auxMomentDay = momentDay;

    so.signalSem(sTime);
    //Distinguimos qué tipo de Tª hay q imprimir:
    if (auxRxMessageTemp.typeInfo == 8) { //tempExt
      Serial.println("caso temp ext");
      so.waitSem(sTempExt);

      auxTempExt = TempExt;

      so.signalSem(sTempExt);

      // The LCD is a critial region itself (shared between PrintTemp and Alarm)
      so.waitSem(sLCD);

      hib.lcdClear();
      hib.lcdSetCursorFirstLine();
      if (auxMomentDay) {
        sprintf(charBuff, "%s", "day");
      } else {
        sprintf(charBuff, "%s", "night");
      }
      hib.lcdPrint(charBuff);

      hib.lcdSetCursorSecondLine();
      dtostrf(auxTempExt, 1, 2, floatBuffer);
      sprintf(charBuff, "Temp Ext: %s    ", floatBuffer);
      hib.lcdPrint(charBuff);

      so.signalSem(sLCD);


    } else if (auxRxMessageTemp.typeInfo == 7) {
      Serial.print("caso temp ints: +");

      arrayTempInt[auxRxMessageTemp.numRoom] = auxRxMessageTemp.tempInt;
      Serial.println(arrayTempInt[auxRxMessageTemp.numRoom]);

      // hib.ledToggle(1); // for debugging
      Serial.print("num rooms: +");
      Serial.println(auxRxMessageTemp.numRoom);

      if (auxRxMessageTemp.numRoom == 3) {
        // hib.ledToggle(4); // for debugging
        // The LCD is a critial region itself (shared between PrintTemp and Alarm)
        so.waitSem(sLCD);

        hib.lcdClear();
        hib.lcdSetCursorFirstLine();

        dtostrf(arrayTempInt[0], 1, 2, floatBuffer);
        sprintf(charBuff, "1:%s ", floatBuffer);
        hib.lcdPrint(charBuff);

        dtostrf(arrayTempInt[1], 1, 2, floatBuffer);
        sprintf(charBuff, "2:%s ", floatBuffer);
        hib.lcdPrint(charBuff);
        Serial.print("arrayTempInt[3]: ");
        Serial.println(arrayTempInt[1]);
        hib.lcdSetCursorSecondLine();

        dtostrf(arrayTempInt[2], 1, 2, floatBuffer);
        sprintf(charBuff, "3:%s ", floatBuffer);
        hib.lcdPrint(charBuff);

        dtostrf(arrayTempInt[3], 1, 2, floatBuffer);
        sprintf(charBuff, "4:%s ", floatBuffer);
        hib.lcdPrint(charBuff);

        so.signalSem(sLCD);
      }

    }
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

    so.waitSem(sTempExt);

    TempExt = celsius;

    so.signalSem(sTempExt);

    //Serial.print("celsius:");
    //Serial.println(celsius);

    so.waitSem(sCanCtrl);

    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      CAN.sendMsgBufNonBlocking(CAN_ID_TEMP_EXT, CAN_EXTID, sizeof(float), (INT8U *)  &celsius);

    so.signalSem(sCanCtrl);

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
    ldrAdcValue = hib.ldrReadAdc(hib.LEFT_LDR_SENS); //Aquí mos retorna el valor en el rang adc 0-1024
    //Serial.print("Left ldr sensor -> adc = ");
    // Serial.println(ldrAdcValue);

    //ara ho haurem de mapear en els valors que necessitem.
    if (ldrAdcValue < 512) {
      light = false;
    } else {
      light = true;
    }

    //Se lo enviamos a ShareTime
    so.signalMBox(mbLight, (byte*) &light);

    so.waitSem(sCanCtrl);

    //CAN
    if (CAN.checkPendingTransmission() != CAN_TXPENDING)
      CAN.sendMsgBufNonBlocking(CAN_ID_LIGHT, CAN_EXTID, sizeof(boolean), (INT8U *) &light);
    so.signalSem(sCanCtrl);

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


void Alarm()
{
  uint8_t  auxNumRoom;
  char charBuff[20];

  const uint8_t TAM = 3;
  const uint8_t Notes[TAM] = {SI, SI, DO};
  uint8_t c;

  while (true) {
    // Wait until any of the bits of the flag fAlarmEvent
    // indicated by the bits of maskAlarmEvent are set to '1'
    so.waitFlag(fAlarmEvent, maskAlarmEvent);

    // Clear the maskAlarmEvent bits of flag fAlarmEvent to not process the same event twice
    so.clearFlag(fAlarmEvent, maskAlarmEvent);
Serial.println("hola");
    auxNumRoom = rxNumRoom;

    //Simulate alarm
    hib.ledToggle(auxNumRoom - 1);

    for (int i = 0; i < TAM; i++) {
      c = Notes[i];
      playNote(c, 4, 500);
    }
    //falta hacer el print por el lcd de el num de habitacion en la q ha saltado la alarma

    so.waitSem(sLCD);
    hib.lcdClear();
    sprintf(charBuff, "ALARM in room: %d ", auxNumRoom);
    hib.lcdPrint(charBuff);
    so.signalSem(sLCD);
    //hib.lcdClear();
  }
}


void playNote(unsigned char note, unsigned char octave, unsigned int duration) {
  float freq;
  float potencia;
  potencia = pow(2, ((((float)note) - 10.0) / 12.0 + ((float) octave) - 4.0));
  freq = 440 * potencia;
  hib.buzzPlay(duration, freq);


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
    sCanCtrl = so.defSem(1); // intially accesible

    // Definition and initialization of mailboxes
    mbLight = so.defMBox();

    // Definition and initialization of flags
    fCANPrintEvent = so.defFlag();
    fAlarmEvent = so.defFlag();

    // Definition and initialization of tasks
    so.defTask(PrintTemp, PRIO_TASK_PrintTemp);
    so.defTask(DetectTempExt, PRIO_TASK_DETECT_TEMP_EXT);
    so.defTask(TimetableHeating, PRIO_TASK_TIMETABLE_HEATING);
    so.defTask(ShareTime, PRIO_TASK_SHARE_TIME);
    so.defTask(Alarm, PRIO_TASK_ALARM);



    //Set up timer 5 so that the SO can regain the CPU every tick
    hib.setUpTimer5(TIMER_TICKS_FOR_125ms, (tClkPreFactType) TIMER_PSCALER_FOR_125ms, timer5Hook);


    // Start mutltasking (program does not return to 'main' from hereon)
    so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER

  }

}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
