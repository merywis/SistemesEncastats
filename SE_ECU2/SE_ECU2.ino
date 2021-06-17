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
#define PRIO_TASK_SIMULATE_TEMP_INT 2
#define PRIO_TASK_KeyDetector 3
#define PRIO_TASK_ShareAdcValue 2
#define PRIO_TASK_InsertTemp 3
#define PRIO_TASK_InsertComandos 3
#define PRIO_TASK_SHARE 3

#define CAN_ID_PRINT_TEMP 1
#define CAN_ID_TEMP_EXT 2
#define CAN_ID_LIGHT 3
#define CAN_ID_ALARM 4

#define PERIOD_TASK_STATE 5
#define PERIOD_TASK_INSERTCOMANDOS 1

/******************************************************************************/
/** Global const and variables ************************************************/
/******************************************************************************/
// stores every new adc adquired value
// shared between: adchook (writes) and ShareAdcValue (reads)
volatile uint16_t adcValue = 0; // critical region beetween adcHook and ShareAdcValue

// stores the last sampled value of the sensor
// calculated upon adcValue
// shared between ShareAdcValue (writes) and InsertTemp (reads)
volatile float sampledAdc = 0.0;
volatile uint8_t key = 255;

volatile float rxTemp = 0.0;
volatile boolean rxTime = false;

volatile float TempExt = 24.0;
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
  float tempGoal = 21.0;
  float tempInt = 23.0;
  float actuacion = 25.0;
  float tempMaxDay = 0.0;
  float tempMinDay = 0.0;
  float tempMaxNight = 0.0;
  float tempMinNight = 0.0;
};
typedef structRoom typeRoom;

struct structState
{
  float tempExt;
  boolean momentDay = true;
  typeRoom datosRoom[4];
};
typedef structState typeState;

struct structGoal
{
  uint8_t numRoom = 1;
  float tempGoal = 21.00;
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
  key = newKey; //guardar el valor de la nueva tecla

  so.setFlag(fKeyEvent, maskKeyEvent);
}

/*****************
  ADCISR
******************/

void adcHook(uint16_t newAdcAdquiredValue)
{
  adcValue = newAdcAdquiredValue; //guardar el valor del nuevo ADC

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

/* La tarea State es la tarea controladora del sistema */
void State()
{

  typeState state;
  typeTempInfo infoSimulateTemp;
  typeGoal auxGoal;
  typeLimitesRoom auxStructLimites[4];

  uint8_t numRoom;
  float temp1, temp2;
  int j, x;

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

    so.waitSem(sTempInt);

    //Actualizar el valor de la temp interior de cada hab.
    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].tempInt = tempIntRoom[i];
    }

    so.signalSem(sTempInt);

    so.waitSem(sDatosTemp);

    //Actualizar el valor de los los límites en cada habitación.
    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].tempMaxDay = arrayLimites[i].maxDay;
      state.datosRoom[i].tempMinDay = arrayLimites[i].minDay;
      state.datosRoom[i].tempMaxNight = arrayLimites[i].maxNight;
      state.datosRoom[i].tempMinNight = arrayLimites[i].minNight;
    }

    so.signalSem(sDatosTemp);

    //Comprobar si las temp interiores han superado los límites en cada hab.
    for (int i = 0; i < 4; i++) {
      if (state.momentDay) { //caso límites día
        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxDay || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinDay) {
          so.waitSem(sCanCtrl);
          
          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(int), (INT8U *) &i);

          so.signalSem(sCanCtrl);

        }
      } else {
        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxNight || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinNight) {
          
          so.waitSem(sCanCtrl);

          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(int), (INT8U *) &i);

          so.signalSem(sCanCtrl);
        }
      }
    }

    //Cálculo de la Actuación en cada habitación
    for (int i = 0; i < 4; i++) {
      state.datosRoom[i].actuacion = (state.datosRoom[i].tempGoal - 0.2 * state.tempExt - 0.2 * state.datosRoom[i].tempInt) / 0.6;
    }

    //Transmitir Actuación
    for (int i = 0; i < 4; i++) {

      infoSimulateTemp.numRoom = i;
      infoSimulateTemp.tempExt = state.tempExt;
      infoSimulateTemp.tempInt = state.datosRoom[i].tempInt;
      infoSimulateTemp.actuacion = state.datosRoom[i].actuacion;

      so.signalMBox(mbInfoTemp, (byte*) &infoSimulateTemp);
    }

    //Imprimir por la terminal la información del sistema
    x = 1;
    term.move(6, x);
    for (int i = 0; i < NUM_ROOM; i++) {
      term.print("Room ");
      term.print(i + 1);
      term.move(7, x);
      term.print("Indoor Temp: ");
      term.move(8, x);
      term.print(state.datosRoom[i].tempInt);
      term.move(9, x);
      term.print("Temp Goal:  ");
      term.move(10, x);
      term.print(state.datosRoom[i].tempGoal);
      term.move(11, x);
      term.print("Day limits:  ");
      term.move(12, x);
      term.print(state.datosRoom[i].tempMinDay);
      term.move(13, x);
      term.print(state.datosRoom[i].tempMaxDay);
      term.move(14, x);
      term.print("Night limits:  ");
      term.move(15, x);
      term.print(state.datosRoom[i].tempMinNight);
      term.move(16, x);
      term.print(state.datosRoom[i].tempMaxNight);
      x = x + 25;
      term.move(6, x);
    }

    nextActivationTick = nextActivationTick +  PERIOD_TASK_STATE; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}

/* Tarea encargada de simular la temp interior de una habitación como si fuera un sensor */
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

    //Actualizamos el valor
    so.waitSem(sTempInt);

    tempIntRoom[infoSimulateTemp.numRoom] = newTempInt;

    so.signalSem(sTempInt);
  }

}

/* Tarea que recibe los mensajes del CAN y los guarda en variables globales*/
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

/* Detecta la tecla pulsada y realiza la acción correspondiente a la tecla */
void KeyDetector()
{
  
  typeMessageTemp MessageTemp[4];
  uint8_t auxKey;
  
  while (1)
  {
    // Wait until any of the bits of the flag fKeyEvent
    // indicated by the bits of maskKeyEvent are set to '1'
    so.waitFlag(fKeyEvent, maskKeyEvent);

    // Clear the flag fKeyEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);
    
    //distinguimos los diferentes posibles casos:
    if (key == 0 || key == 1 || key == 2 || key == 3) {
      //mandamos la tecla hacia InsertTemp que se corresponde con el número de habitación del que el usuario quiere cambiar la tempGoal
      auxKey = key;
      so.signalMBox(mbNumRoom, (byte*) &auxKey);

    } else if (key == 8) {
      //Caso Tª exterior-> enviar valor key a través de CAN
      //Rellenamos el campo necesario en el struct MessageTemp1 para indicar que lo que se desea imprimir es la Tª ext
      MessageTemp[0].typeInfo = 8;

      so.waitSem(sCanCtrl);

      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp[0]);
      so.signalSem(sCanCtrl);

    } else if (key == 7) {
      //Aquí deseamos imprimir la Tª interior de las 4 habitaciones. Debido a que no cabe en un único "envío", vamos a enviar
      //un mensaje por cada habitación
      for (int i = 0; i < 4; i++) {

        MessageTemp[i].typeInfo = 7;
        MessageTemp[i].numRoom = i;

        so.waitSem(sTempInt);

        MessageTemp[i].tempInt = tempIntRoom[i];

        so.signalSem(sTempInt);

        so.waitSem(sCanCtrl);
        if (CAN.checkPendingTransmission() != CAN_TXPENDING)
          CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp[i]);

        so.signalSem(sCanCtrl);
      }
    }
  }
}

/* tarea encarga de compartir con la tarea state la temp deseada de una habitación */
void InsertTemp() {

  uint8_t auxNumRoom;
  uint8_t * rxNumRoomMessage;

  float auxSampledAdc;

  while (true) {

    // Wait until receiving the new state from KeyDetector
    so.waitMBox(mbNumRoom, (byte**) &rxNumRoomMessage);

    auxNumRoom = *rxNumRoomMessage; //posibles valores son 0,1,2,3.

    // Read adcValue (shared with ShareAdcValue)
    so.waitSem(sDatosTemp);

    auxSampledAdc = sampledAdc;

    so.signalSem(sDatosTemp);


    so.waitSem(sDatosTemp);

    Goal.numRoom = auxNumRoom;
    Goal.tempGoal = auxSampledAdc;

    so.signalSem(sDatosTemp);
  }
}


/* tarea encargada de obtener el valor del ADC, mapearlo y compartirlo a la tarea Insert Temp */
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

    sampledAdc = auxSampledAdc; //Actualiza el valor del ADC compartido con la tarea InsertTemp

    so.signalSem(sDatosTemp);
    
  }
}


/* tarea encargada de obtener los limites introducidos por el usuario a través de la terminal */
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
      //La ejecución va por turnos. Cada turno es para introducir un elemento. El primero es 
      //la habitacion luego las temp max y mín y luego si de dia o de noche.
      switch (turno)
      {
        case 0:
          //1 = 49 2 = 50
          if (c == '1' || c == '2' || c == '3' || c == '4') {
            room = (int)c - 48;
            turno++;
          } else {
            //Serial.println("Error en la Seleccion de Habitación");
          }
          break;

        case 1:
          if (c == ' ') { //El espacio
            turno++;
          } else {
            turno = 0;
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
            }
          } else {
            turno = 0;
          }
          break;

        case 3:
          if (c == ' ') { //El espacio
            turno++;
          } else {
            turno = 0;
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
            }

          } else {
            turno = 0;
          }
          break;
          
        case 5:
          if (c == ' ') { //El espacio
            turno++;
            //Serial.println("FIN Temperatura Minima Seleccionada Correctamente");
          } else {
            turno = 0;
            //Serial.println("Error en FIN Seleccion de Temperatura Minima");
          }
          break;

        case 6:
          if (c == 'd' || c == 'n') {
            turno = 0;

            //Actualizar el valor de los límites en la habitación seleccionada
            so.waitSem(sDatosTemp);
            if (c == 'd') {
              arrayLimites[room - 1].maxDay = tempMax;
              arrayLimites[room - 1].minDay = tempMin;
            } else {
              arrayLimites[room - 1].maxNight = tempMax;
              arrayLimites[room - 1].minNight = tempMin;
            }
            so.signalSem(sDatosTemp);
          } else {
            turno = 0;
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
    so.defTask(SimulateTempInt, PRIO_TASK_SIMULATE_TEMP_INT);
    so.defTask(State, PRIO_TASK_STATE);
    so.defTask(KeyDetector, PRIO_TASK_KeyDetector);
    so.defTask(ShareAdcValue, PRIO_TASK_ShareAdcValue);
    so.defTask(InsertTemp, PRIO_TASK_InsertTemp);
    so.defTask(InsertComandos, PRIO_TASK_InsertComandos);
    so.defTask(Share, PRIO_TASK_SHARE);

    //Set up adc
    hib.adcSetTimerDriven(TIMER_TICKS_FOR_500ms, (tClkPreFactType) TIMER_PSCALER_FOR_500ms, adcHook);

    //Set up timer 5 so that the SO can regain the CPU every tick
    hib.setUpTimer5(TIMER_TICKS_FOR_125ms, (tClkPreFactType) TIMER_PSCALER_FOR_125ms, timer5Hook);

    // Start mutltasking (program does not return to 'main' from hereon)
    so.enterMultiTaskingEnvironment(); // GO TO SCHEDULER
  }

}
