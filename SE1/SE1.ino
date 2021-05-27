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

#define PRIO_TASK_STATE 8
#define PRIO_TASK_SIMULATE_TEMP_INT 2
#define PRIO_TASK_KeyDetector 3
#define PRIO_TASK_ShareAdcValue 4
#define PRIO_TASK_InsertTemp 2
#define PRIO_TASK_InsertComandos 6
#define PRIO_TASK_SHARE 7


#define CAN_ID_PRINT_TEMP 1
#define CAN_ID_TEMP_EXT 2
#define CAN_ID_LIGHT 3
#define CAN_ID_ALARM 4


#define PERIOD_TASK_STATE 10
#define PERIOD_TASK_INSERTCOMANDOS 3

//prioridades - stacksize 256 - ir comentando y descomentando tasks - periodo
/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/
// stores every new adc adquired value
// shared between: adchook (writes) and ShareAdcValue (reads)
volatile uint16_t adcValue = 0; // critical region beetween adcHook and ShareAdcValue

// stores the last sampled value of the sensor
// calculated upon adcValue
// shared between ShareAdcValue (writes) and InsertTemp (reads)
volatile float sampledAdc = 0.0; // critical region between ShareAdcValue and InsertTemp

volatile uint8_t key;

volatile float rxTemp = 0.0; //tienen q ser globales ? :(
volatile boolean rxTime = false; //tienen q ser globales ? :(

volatile float TempExt = 24.0; //tienen q ser globales ? :(
volatile boolean MomentDay = false;

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
Sem sDatosTemp;
Sem sTempInt;
Sem sShare;
Sem sCanCtrl;


/*********
  Declaration of mailboxes
*********/
MBox mbNumRoom;
MBox mbInfoTemp;


/******************************************************************************/
/** Definition of own types ***************************************************/
/******************************************************************************/

struct structRoom
{
  float tempGoal = 21.0; //
  float tempInt = 23.0; //
  float actuacion = 25.0;
  float tempMaxDay = 0.0;
  float tempMinDay = 0.0;
  float tempMaxNight = 0.0;
  float tempMinNight = 0.0;
};
typedef structRoom typeRoom;

struct structState
{
  float tempExt; // Number of edition
  boolean momentDay = true; // true = day & false = night
  typeRoom datosRoom[4];
};
typedef structState typeState;


struct structGoal
{
  uint8_t numRoom = 1; // Number of edition
  float tempGoal = 21.00; // Number of edition
};
typedef structGoal typeGoal;

typeGoal Goal;

struct structLimitesRoom {
  float maxDay = 30.0; // Number of edition
  float maxNight = 28.9; // Number of edition
  float minDay = 15.0; // Number of edition
  float minNight = 17.0; // Number of edition
};
typedef structLimitesRoom typeLimitesRoom;

typeLimitesRoom arrayLimites[4];

float tempIntRoom[] = {26.0, 26.0, 26.0, 26.0};

struct structMessageTemp
{
  byte typeInfo;
  byte numRoom;
  float tempInt;
};
typedef structMessageTemp typeMessageTemp;

struct structTempInfo
{
  uint8_t numRoom;
  float actuacion;
  float tempExt;
  float tempInt;
};
typedef structTempInfo typeTempInfo;


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

  // Awake task A by setting to '1' the bits of fExtEvent indicated by maskAdcEvent
  so.setFlag(fAdcEvent, maskAdcEvent);

}

/*****************
  TICKISR
******************/

// Hook FOR TICKISR
void timer5Hook ()
{
  hib.ledToggle(2);
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
    CAN.readRxMsg();
    switch (CAN.getRxMsgId())
    {
      case CAN_ID_TEMP_EXT:
        CAN.getRxMsgData((byte*) &rxTemp);
        so.setFlag(fCANEvent, maskRxTempExtEvent);
        break;

      case CAN_ID_LIGHT:
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
  typeTempInfo infoSimulateTemp;
  typeGoal auxGoal;
  typeLimitesRoom auxStructLimites[4];

  uint8_t numRoom;
int j;

  unsigned long nextActivationTick;
  nextActivationTick = so.getTick();

  while (true)
  {
    // Read TempExt (shared with the task Share)
    so.waitSem(sShare);

    state.tempExt = TempExt;

    so.signalSem(sShare);

    // Read MomentDay (shared with the task Share)
    so.waitSem(sShare);

    state.momentDay = MomentDay;

    so.signalSem(sShare);

    // Read TempExt (shared with the task Share)
    so.waitSem(sDatosTemp);
    auxGoal = Goal;

    so.signalSem(sDatosTemp);

    //Actualizar el valor
    state.datosRoom[auxGoal.numRoom].tempGoal = auxGoal.tempGoal;

    //Update the inner temperature of each room

    so.waitSem(sTempInt);

    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].tempInt = tempIntRoom[i];

      //PRINTS
      Serial.print("Temperatura interior de la habitacion ");
      j = i + 1;
      Serial.print(j);
      Serial.print(" es ");
      Serial.println(state.datosRoom[i].tempInt);
      
    }

    so.signalSem(sTempInt);

    //actualizar limites con struct q viene de structLimites (InsertComandos) ************
    so.waitSem(sDatosTemp);

    for (int i = 0; i < 4; i++) {
      auxStructLimites[i] = arrayLimites[i];
    }

    so.signalSem(sDatosTemp);

    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].tempMaxDay = auxStructLimites[i].maxDay;
      state.datosRoom[i].tempMinDay = auxStructLimites[i].minDay;
      state.datosRoom[i].tempMaxNight = auxStructLimites[i].maxNight;
      state.datosRoom[i].tempMinNight = auxStructLimites[i].minNight;

      //PRINTS
      Serial.print("Los limites maximos de la habitacion ");
      j = i + 1;
      Serial.print(j);
      Serial.print(" es ");
      Serial.println(state.datosRoom[i].tempMaxDay);
    }

    for (int i = 0; i < 4; i++) {
      if (state.momentDay) { //caso límites día

        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxDay || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinDay) {

          //Enviar vía CAN el número de habitación en el que ha saltado la alarma:
          so.waitSem(sCanCtrl);

          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(uint8_t), (INT8U *) &i);

          so.signalSem(sCanCtrl);
        }

      }  else {

        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxNight || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinNight) {
          //Enviar vía CAN el número de habitación en el que ha saltado la alarma:
          so.waitSem(sCanCtrl);

          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(uint8_t), (INT8U *) &i);

          so.signalSem(sCanCtrl);
        }
      }
    }


    //Cálculo de la Actuación

    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].actuacion = (state.datosRoom[i].tempGoal - 0.2 * state.tempExt - 0.2 * state.datosRoom[i].tempInt) / 0.6;

      //PRINTS
      Serial.print("La Actuacion de la habitacion ");
      j = i + 1;
      Serial.print(j);
      Serial.print(" es ");
      Serial.println(state.datosRoom[i].actuacion);
    }

    //Serial.print("actuacion: ");
    //Serial.println( state.datosRoom1.actuacion);

    //Transmitir Actuación

    for (int i = 0; i < 4; i++) {

      infoSimulateTemp.numRoom = (i + 1);
      infoSimulateTemp.tempExt = state.tempExt;
      infoSimulateTemp.tempInt = state.datosRoom[i].tempInt;
      infoSimulateTemp.actuacion = state.datosRoom[i].actuacion;

      so.signalMBox(mbInfoTemp, (byte*) &infoSimulateTemp);
    }

    nextActivationTick = nextActivationTick +  PERIOD_TASK_STATE; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}

void SimulateTempInt()
{

  typeTempInfo infoSimulateTemp;
  typeTempInfo * rxStructInfoMessage;
  float newTempInt;
  int j;

  while (true) {
    so.waitMBox(mbInfoTemp, (byte**) &rxStructInfoMessage);
    infoSimulateTemp = *rxStructInfoMessage;

    //Formula de la nueva Tª interior
    newTempInt = 0.5 * infoSimulateTemp.actuacion + 0.25 * infoSimulateTemp.tempInt + 0.25 * infoSimulateTemp.tempExt;

    /*Serial.print("Habitacion: ");
      Serial.print(infoSimulateTemp.numRoom);

      Serial.print("   New Temp Int: ");
      Serial.println(newTempInt);
      Serial.println("******************");*/

    //Actualizamos el valor
    so.waitSem(sTempInt);

    tempIntRoom[infoSimulateTemp.numRoom] = newTempInt;

    so.signalSem(sTempInt);

       //PRINTS
      Serial.print("La NUEVA temperatura de la habitacion ");
      j = infoSimulateTemp.numRoom + 1;
      Serial.print(j);
      Serial.print(" es ");
      Serial.println(tempIntRoom[infoSimulateTemp.numRoom]);

  }

}

void Share() {

  // define a mask to awake either by maskRxTempExtEvent or maskRxTimeEvent
  unsigned char mask = (maskRxTempExtEvent | maskRxTimeEvent);
  // var to read flag value
  unsigned char flagValue;

  while (true)
  {
    // Wait until any of the bits of the flag fCANEvent
    // indicated by the bits of maskRxTempExtEvent or maskRxTimeEvent are set to '1'
    so.waitFlag(fCANEvent, mask);

    // read flag Value
    flagValue = so.readFlag(fCANEvent);

    // Clear the mask bits of flag fCANEvent to not process the same event twice
    so.clearFlag(fCANEvent, mask);
    // distinguish whether the event has been alarm or not alarm
    switch (flagValue)
    {
      case maskRxTempExtEvent:

        // Read rxTemp
        so.waitSem(sShare);

        TempExt = rxTemp;

        so.signalSem(sShare);
        break;

      case maskRxTimeEvent:

        // Read rxTime
        so.waitSem(sShare);

        MomentDay = rxTime;

        so.signalSem(sShare);
        break;

      default:
        break;
    }

  }
}

void KeyDetector()
{
  typeMessageTemp MessageTemp1, MessageTemp2, MessageTemp3, MessageTemp4;
  uint8_t auxKey;
  while (1)
  {
    // Wait until any of the bits of the flag fKeyEvent
    // indicated by the bits of maskKeyEvent are set to '1'
    so.waitFlag(fKeyEvent, maskKeyEvent);

    // Clear the flag fKeyEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);
    hib.ledToggle(3);

    //distinguimos los diferentes posibles casos:
    if (key == 0 || key == 1 || key == 2 || key == 3) {
      //mandamos la tecla hacia InsertTemp que se corresponde con el número de habitación del que el usuario quiere cambiar la tempGoal
      //Serial.print("******** K E Y !!!!!!!!: ");
      //Serial.println(key);
      auxKey = key;
      so.signalMBox(mbNumRoom, (byte*) &auxKey);

    } else if (key == 8) {
      //Caso Tª exterior-> enviar valor key a través de CAN
      //Rellenamos el campo necesario en el struct MessageTemp1 para indicar que lo que se desea imprimir es la Tª ext
      MessageTemp1.typeInfo = 8;
      // Send sensor via CAN

      so.waitSem(sCanCtrl);

      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp1);

      so.signalSem(sCanCtrl);

    } else if (key == 7) {
      //Aquí deseamos imprimir la Tª interior de las 4 habitaciones. Debido a que no cabe en un único "envío", vamos a enviar
      //un mensaje por cada habitación

      //Información habitación 1
      MessageTemp1.typeInfo = 7;
      MessageTemp1.numRoom = 1;

      so.waitSem(sTempInt);
      MessageTemp1.tempInt = tempIntRoom[0];
      so.signalSem(sTempInt);

      //Información habitación 2
      MessageTemp2.typeInfo = 7;
      MessageTemp2.numRoom = 2;

      so.waitSem(sTempInt);
      MessageTemp2.tempInt = tempIntRoom[1];
      so.signalSem(sTempInt);

      //Información habitación 3
      MessageTemp3.typeInfo = 7;
      MessageTemp3.numRoom = 3;

      so.waitSem(sTempInt);
      MessageTemp3.tempInt = tempIntRoom[2];
      so.signalSem(sTempInt);

      //Información habitación 4
      MessageTemp4.typeInfo = 7;
      MessageTemp4.numRoom = 4;

      so.waitSem(sTempInt);
      MessageTemp4.tempInt = tempIntRoom[3];
      so.signalSem(sTempInt);

      // Send sensor via CAN
      so.waitSem(sCanCtrl);

      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp1);
      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp2);
      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp3);
      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp4);

      so.signalSem(sCanCtrl);
    }
  }
}


void InsertTemp() {

  uint8_t auxNumRoom;
  uint8_t * rxNumRoomMessage;

  float auxSampledAdc;

  while (true) {
    //Serial.println("hola ");

    // Wait until receiving the new state from KeyDetector
    so.waitMBox(mbNumRoom, (byte**) &rxNumRoomMessage);

    auxNumRoom = (*rxNumRoomMessage) + 1;

    // Read adcValue (shared with ShareAdcValue)
    so.waitSem(sDatosTemp);

    auxSampledAdc = sampledAdc;

    so.signalSem(sDatosTemp);


    so.waitSem(sDatosTemp);

    Goal.numRoom = auxNumRoom;
    Goal.tempGoal = auxSampledAdc;

    so.signalSem(sDatosTemp);

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


    so.waitSem(sDatosTemp);

    sampledAdc = auxSampledAdc;

    so.signalSem(sDatosTemp);
  }
}



void InsertComandos()
{
  char c;
  int room;
  float tempMin;
  float tempMax;
  char moment;
  uint8_t turno = 0;
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
      switch (turno)
      {
        case 0:
          //1 = 49 2 = 50
          if (c == '1' || c == '2' || c == '3' || c == '4') {
            room = (int)c - 48;
            turno++;
            Serial.print("Habitacion Seleccionada Correctamente: ");
            Serial.println(room);
          } else {
            Serial.println("Error en la Seleccion de Habitación");
          }
          break;

        case 1:
          if (c == ' ') { //El espacio
            turno++;
            Serial.println("FIN Habitacion Seleccionada Correctamente");
          } else {
            turno = 0;
            Serial.println("Error en FIN Seleccion de Habitación");
          }
          break;

        case 2:
          if ( c == '.' || c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') {
            if (i == 0) { //cogemos el primer valor (decenas)
              tempMax = 0;
              tempMax = tempMax + (((int)c - 48) * 10);
              i = i + 1;
            } else if (i == 1) { //unidades
              tempMax = tempMax + (int)c - 48;
              i = i + 1;
            } else if (i == 2) { //'.'
              i = i + 1;
            } else {  //decimales
              tempMax = tempMax + (((float) ((int)c - 48)) / 10);
              i = 0;
              turno++;
              Serial.print("Temperatura Maxima Seleccionada Correctamente: ");
              Serial.println(tempMax);
            }

          } else {
            turno = 0;
            Serial.println("Error en la Temperatura Maxima Seleccion de Habitación");
          }

          break;

        case 3:
          if (c == ' ') { //El espacio
            turno++;
            Serial.println("FIN Temperatura Maxima Seleccionada Correctamente");
          } else {
            turno = 0;
            Serial.println("Error en FIN Seleccion de Temperatura Maxima");
          }
          break;

        case 4:
          if ( c == '.' || c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') {
            if (i == 0) {
              tempMin = 0;
              tempMin = tempMin + (((int)c - 48) * 10);
              i = i + 1;
            } else if (i == 1) {
              tempMin = tempMin + (int)c - 48;
              i = i + 1;
            } else if (i == 2) {
              i = i + 1;
            } else {
              tempMin = tempMin + (((float) ((int)c - 48)) / 10);
              i = 0;
              turno++;
              Serial.print("Temperatura Minima Seleccionada Correctamente: ");
              Serial.println(tempMin);
            }

          } else {
            turno = 0;
            Serial.println("Error en la Temperatura Minima Seleccion de Habitación");
          }

          break;
        case 5:
          if (c == ' ') { //El espacio
            turno++;
            Serial.println("FIN Temperatura Minima Seleccionada Correctamente");
          } else {
            turno = 0;
            Serial.println("Error en FIN Seleccion de Temperatura Minima");
          }
          break;

        case 6:
          if (c == 'd' || c == 'n') {
            turno = 0;

            //Actualizar el valor de los límites en la habitación seleccionada
            so.waitSem(sDatosTemp);
            if (c == 'd') {
              arrayLimites[room].maxDay = tempMax;
              arrayLimites[room].minDay = tempMin;
            } else {
              arrayLimites[room].maxNight = tempMax;
              arrayLimites[room].minNight = tempMin;
            }
            so.signalSem(sDatosTemp);

            Serial.println("FIN Momento del Dia Seleccionada Correctamente: ");
            Serial.println(c);
          } else {
            turno = 0;
            Serial.println("Error en el Momento del dia");
          }

          break;

        default:
          break;
      }
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
  while (CAN.begin(CAN_500KBPS, MODE_NORMAL, true, false) != CAN_OK) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  // Set CAN interrupt handler address in the position of interrupt number 0
  // since the INT output signal of the CAN shield is connected to
  // the Mega 2560 pin num 2, which is associated to that interrupt number.
  attachInterrupt(0, isrCAN, FALLING);

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
    sTempInt = so.defSem(1); // intially accesible
    sShare = so.defSem(1); // intially accesible
    sCanCtrl = so.defSem(1); // intially accesible
    sDatosTemp = so.defSem(1); // intially accesible

    // Definition and initialization of mailboxes
    mbNumRoom = so.defMBox();
    mbInfoTemp = so.defMBox();

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
    so.defTask(SimulateTempInt, PRIO_TASK_SIMULATE_TEMP_INT);

    //Set up adc
    hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, (tClkPreFactType) TIMER_PSCALER_FOR_500ms, adcHook);

    //Set up timer 5 so that the SO can regain the CPU every tick
    hib.setUpTimer5(TIMER_TICKS_FOR_125ms, (tClkPreFactType) TIMER_PSCALER_FOR_125ms, timer5Hook);

    // Start mutltasking (program does not return to 'main' from hereon)
    so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
  }

}





/******************************************************************************/
/** Additional functions ******************************************************/
/******************************************************************************/
