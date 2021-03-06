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
#define PRIO_TASK_ShareAdcValue 2
#define PRIO_TASK_KeyDetector 3
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
Flag fCANEvent;

// Masks
const unsigned char maskKeyEvent = 0x01; // represents new key value adquired
const unsigned char maskAdcEvent = 0x01; // represents new adc value adquired
const unsigned char maskRxTempExtEvent = 0x01; // represents new tempExt value adquired
const unsigned char maskRxTimeEvent = 0x02; // represents new time value adquired


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
  typeRoom datosRoom[NUM_ROOM];
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
  float maxDay = 30.0; 
  float maxNight = 28.9; 
  float minDay = 15.0; 
  float minNight = 17.0; 
};
typedef structLimitesRoom typeLimitesRoom;
typeLimitesRoom arrayLimites[NUM_ROOM];


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
  key = newKey; //guardar el valor de la nueva tecla pulsada

  so.setFlag(fKeyEvent, maskKeyEvent);
}


/*****************
  ADCISR
******************/
void adcHook(uint16_t newAdcAdquiredValue)
{
  adcValue = newAdcAdquiredValue; //guardar el nuevo valor del ADC

  so.setFlag(fAdcEvent, maskAdcEvent);
}


/*****************
  TICKISR
******************/
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
  typeLimitesRoom auxStructLimites[NUM_ROOM];
  
  char charBuff[20];  
  char floatBuffer[6];
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

    // Read Goal (shared with the task InsertTemp)
    so.waitSem(sDatosTemp);
    
    auxGoal = Goal;

    so.signalSem(sDatosTemp);

    //Actualizar el valor
    state.datosRoom[auxGoal.numRoom].tempGoal = auxGoal.tempGoal;

    // Read tempIntRoom[] (shared with the task SimulateTempInt)
    so.waitSem(sTempInt);

    //Actualizar el valor de la temperatura interior de cada habitaci??n
    for (int i = 0; i < NUM_ROOM; i++) {
      state.datosRoom[i].tempInt = tempIntRoom[i];
    }

    so.signalSem(sTempInt);

    // Read arrayLimites[] (shared with the task InsertComandos)
    so.waitSem(sDatosTemp);

    //Actualizar el valor de los los l??mites en cada habitaci??n
    for (int i = 0; i < NUM_ROOM; i++) {
      state.datosRoom[i].tempMaxDay = arrayLimites[i].maxDay;
      state.datosRoom[i].tempMinDay = arrayLimites[i].minDay;
      state.datosRoom[i].tempMaxNight = arrayLimites[i].maxNight;
      state.datosRoom[i].tempMinNight = arrayLimites[i].minNight;
    }

    so.signalSem(sDatosTemp);

    //Comprobar si las temperaturas interiores han superado los l??mites en cada habitaci??n
    for (int i = 0; i < 4; i++) {
      
      if (state.momentDay) { //caso l??mites durante el d??a
        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxDay || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinDay) {
          
          //Caso de alarma, por tanto, transmitimos el n??mero de la habitaci??n afectada via CAN
          so.waitSem(sCanCtrl);
          
          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(int), (INT8U *) &i);

          so.signalSem(sCanCtrl);

        }
      } else { //caso l??mites durante la noche
        if (state.datosRoom[i].tempInt > state.datosRoom[i].tempMaxNight || state.datosRoom[i].tempInt < state.datosRoom[i].tempMinNight) {

          //Caso de alarma, por tanto, transmitimos el n??mero de la habitaci??n afectada via CAN
          so.waitSem(sCanCtrl);

          if (CAN.checkPendingTransmission() != CAN_TXPENDING)
            CAN.sendMsgBufNonBlocking(CAN_ID_ALARM, CAN_EXTID, sizeof(int), (INT8U *) &i);

          so.signalSem(sCanCtrl);
        }
      }
    }

    //C??lculo de la actuaci??n en cada habitaci??n
    for (int i = 0; i < NUM_ROOM; i++) {
      state.datosRoom[i].actuacion = (state.datosRoom[i].tempGoal - 0.2 * state.tempExt - 0.2 * state.datosRoom[i].tempInt) / 0.6;
    }

    //Transmitir Actuaci??n hacia la tarea SimulateTempInt
    for (int i = 0; i < NUM_ROOM; i++) {

      infoSimulateTemp.numRoom = i;
      infoSimulateTemp.tempExt = state.tempExt;
      infoSimulateTemp.tempInt = state.datosRoom[i].tempInt;
      infoSimulateTemp.actuacion = state.datosRoom[i].actuacion;

      so.signalMBox(mbInfoTemp, (byte*) &infoSimulateTemp);
    }

    //Imprimir por el terminal la informaci??n del sistema
    x = 1;
    term.move(6, x);
    for (int i = 0; i < NUM_ROOM; i++) {
      
      sprintf(charBuff, "Room: %d", (i+1));
      term.print(charBuff);
      term.move(7, x);
      
      sprintf(charBuff, "Indoor Temp:");
      term.print(charBuff);
      term.move(8, x);
      
      dtostrf(state.datosRoom[i].tempInt, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      term.move(9, x);
      
      sprintf(charBuff, "Temp Goal:");
      term.print(charBuff);
      term.move(10, x);
      
      dtostrf(state.datosRoom[i].tempGoal, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      term.move(11, x);
      
      sprintf(charBuff, "Day limits:");
      term.print(charBuff);
      term.move(12, x);
      
      dtostrf(state.datosRoom[i].tempMinDay, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      term.move(13, x);
      
      dtostrf(state.datosRoom[i].tempMaxDay, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      term.move(14, x);

      sprintf(charBuff, "Night limits:");
      term.print(charBuff);
      term.move(15, x);
      
      dtostrf(state.datosRoom[i].tempMinNight, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      term.move(16, x);
      
      dtostrf(state.datosRoom[i].tempMaxNight, 1, 2, floatBuffer);
      sprintf(charBuff, "%s", floatBuffer);
      term.print(charBuff);
      
      x = x + 25;
      term.move(6, x);
    }

    nextActivationTick = nextActivationTick +  PERIOD_TASK_STATE; // Calculate next activation time;
    so.delayUntilTick(nextActivationTick);
  }
}


/* Tarea que simula la temperatura interior de una habitaci??n como si fuera un sensor */
void SimulateTempInt()
{

  typeTempInfo infoSimulateTemp;
  typeTempInfo * rxStructInfoMessage;
  
  float newTempInt;
  int j;

  while (true) {
    
    // Wait until receiving the new infoSimulateTemp from State
    so.waitMBox(mbInfoTemp, (byte**) &rxStructInfoMessage);
    infoSimulateTemp = *rxStructInfoMessage;

    //Formula de la nueva T?? interior
    newTempInt = 0.5 * infoSimulateTemp.actuacion + 0.25 * infoSimulateTemp.tempInt + 0.25 * infoSimulateTemp.tempExt;

    // Write tempIntRoom[] (shared with the task State and KeyDetector)
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
    
    // distinguish whether the event has been a temperature or the time
    switch (flagValue)
    {
      case maskRxTempExtEvent:

        // Update TempExt value (shared with the task State)
        so.waitSem(sShare);

        TempExt = rxTemp;

        so.signalSem(sShare);
        
        break;

      case maskRxTimeEvent:

        // Update MomentDay value (shared with the task State)
        so.waitSem(sShare);

        MomentDay = rxTime;

        so.signalSem(sShare);
        
        break;

      default:
        break;
    }

  }
}


/* Detecta la tecla pulsada y realiza la acci??n correspondiente */
void KeyDetector()
{
  
  typeMessageTemp MessageTemp[NUM_ROOM];
  uint8_t numRoom;
  
  while (1)
  {
    // Wait until any of the bits of the flag fKeyEvent
    // indicated by the bits of maskKeyEvent are set to '1'
    so.waitFlag(fKeyEvent, maskKeyEvent);

    // Clear the flag fKeyEvent to not process the same event twice
    so.clearFlag(fKeyEvent, maskKeyEvent);
    
    //distinguimos entre 3 posibles casos:
    if (key == 0 || key == 1 || key == 2 || key == 3) {
      //mandamos la tecla hacia la tarea InsertTemp, que se corresponde con el  
      //n??mero de la habitaci??n d??nde el usuario quiere cambiar la tempGoal
      
      numRoom = key;
      so.signalMBox(mbNumRoom, (byte*) &numRoom);

    } else if (key == 8) {
      //Caso imprimir T?? exterior: enviar valor key a trav??s del CAN
      //Rellenamos el campo necesario en el array MessageTemp para indicar que 
      //lo que se desea imprimir es la temperatura exterior
      
      MessageTemp[0].typeInfo = 8;

      so.waitSem(sCanCtrl);

      if (CAN.checkPendingTransmission() != CAN_TXPENDING)
        CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp[0]);
      
      so.signalSem(sCanCtrl);

    } else if (key == 7) {
      //Caso imprimir la T?? interior de las 4 habitaciones. Debido a que no cabe 
      //en un ??nico mensaje, vamos a dividirlo en una trama por cada habitaci??n
     
      for (int i = 0; i < NUM_ROOM; i++) {

        MessageTemp[i].typeInfo = 7;
        MessageTemp[i].numRoom = i;

        // Read tempIntRoom[] (shared with the task SimulateTempInt)
        so.waitSem(sTempInt);

        MessageTemp[i].tempInt = tempIntRoom[i];

        so.signalSem(sTempInt);

        so.waitSem(sCanCtrl);
        
        if (CAN.checkPendingTransmission() != CAN_TXPENDING)
          CAN.sendMsgBufNonBlocking(CAN_ID_PRINT_TEMP, CAN_EXTID, sizeof(typeMessageTemp), (INT8U *) &MessageTemp[i]);

        so.signalSem(sCanCtrl);
        
        delay(50); //hacemos un delay para que las 4 tramas puedan llegar a la otra ECU
      }
    }
  }
}

/* tarea encarga de compartir con la tarea State la temperatura deseada en una habitaci??n */
void InsertTemp() {

  uint8_t auxNumRoom;
  uint8_t * rxNumRoomMessage;
  float auxSampledAdc;

  while (true) {

    // Wait until receiving the new numRoom from KeyDetector
    so.waitMBox(mbNumRoom, (byte**) &rxNumRoomMessage);

    auxNumRoom = *rxNumRoomMessage; //posibles valores son 0,1,2,3.

    // Read sampledAdc (shared with ShareAdcValue)
    so.waitSem(sDatosTemp);

    auxSampledAdc = sampledAdc;

    so.signalSem(sDatosTemp);

    // Write Goal (shared with State)
    so.waitSem(sDatosTemp);

    Goal.numRoom = auxNumRoom;
    Goal.tempGoal = auxSampledAdc;

    so.signalSem(sDatosTemp);
  }
}


/* tarea encargada de obtener el valor del ADC, mapearlo y compartirlo con la tarea InsertTemp */
void ShareAdcValue() {

  uint16_t auxAdcValue;
  float auxSampledAdc;

  while (1)
  {
    // Wait until any of the bits of the flag fAdcEvent
    // indicated by the bits of maskAdcEvent are set to '1'
    so.waitFlag(fAdcEvent, maskAdcEvent);

    // Clear the flag fAdcEvent to not process the same event twice
    so.clearFlag(fAdcEvent, maskAdcEvent);

    auxAdcValue = adcValue;
    
    // Map the auxAdcValue in range [-10, 40] (temperature)
    auxSampledAdc = (((float) auxAdcValue) * 0.04887585) - 10;

    // Write sampledAdc (shared with InsertTemp)
    so.waitSem(sDatosTemp);

    sampledAdc = auxSampledAdc;

    so.signalSem(sDatosTemp);
    
  }
}


/* tarea encargada de obtener los l??mites introducidos por el usuario a trav??s del terminal */
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
      //La ejecuci??n va por turnos. Cada turno es para introducir un elemento. El primero es el n??mero
      //de habitaci??n, luego las T?? m??ximas y m??nimas y luego si son l??mites para el d??a o la noche.
      switch (turno)
      {
        case 0: //Obtenemos el n??mero de la habitaci??n
          if (c == '1' || c == '2' || c == '3' || c == '4') {
            
            room = (int)c - 48;
            turno++;

          } else {
            
            turno = 0;
            
          }
          break;

        case 1:
          if (c == ' ') { //El espacio
            
            turno++;
            
          } else {
            
            turno = 0;
          }
          break;

        case 2: //Obtenemos el l??mite de la temeratura m??xima
          if ( c == '.' || c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') {
            if (i == 0) { //cogemos el primer valor: decenas
              
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

        case 4: //Obtenemos el l??mite de la temeratura m??nima
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
            
          } else {
            
            turno = 0;
            
          }
          break;

        case 6: //Usuario indica si los l??mites son para el d??a o para la noche
          if (c == 'd' || c == 'n') {
            
            turno = 0;
            
             // Write arrayLimites[] (shared with State)
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
