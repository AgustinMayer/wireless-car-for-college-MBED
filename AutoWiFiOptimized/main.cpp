
/**
*@file main.cpp
*@author Agustín Alejandro Mayer (agustinmayer13@gmail.com)
*@brief Actividad de regularización y promoción Computación III - Ing. Mecatrónica
*@version 20221102_vA03
 */

#include "mbed.h"
#include "wifi.h"
#include "config.h"

#define RINGBUFFLENGTH      256
#define GENERALINTERVAL     100
#define SECUENCEHB          0x0015
#define MASKHB              0x0F
#define ALIVEAUTOINTERVAL   10000//ms
#define POSID               4
#define POSDATA             5
#define SERIE               1
#define WIFI                0

#define servoCheck          flags.su_bits.bit0
#define servoOnMove         flags.su_bits.bit1
#define HCSROn              flags.su_bits.bit2
#define echoOn              flags.su_bits.bit3
#define blocker             flags.su_bits.bit4
#define isModeOn            flags.su_bits.bit5
#define isButtonPressed     flags.su_bits.bit6
#define door                flags.su_bits.bit7
#define targetDir           flags2.su_bits.bit0
#define located             flags2.su_bits.bit1
#define speedometer         flags2.su_bits.bit2
#define door2               flags2.su_bits.bit3
#define founded             flags2.su_bits.bit4
#define decision            flags2.su_bits.bit5
#define onLine              flags2.su_bits.bit6

#define LMOTOR      motores[0]
#define RMOTOR      motores[1]
#define MAXSERVO    msServo[1]
#define MINSERVO    msServo[0]
#define PPSL        PPS[0]
#define PPSR        PPS[1]
#define PRESS       0
#define UNPRESS     1
#define WAITING     0
#define PLAYING     1
#define PRESSING    2
#define IDLE        0
#define MODE1       1
#define MODE2       2
#define MODE3       3
#define LEFT        0
#define RIGHT       1
#define CENTER      2
#define CASE        3
#define ONTHELINE   0
#define CHECKING    1 
#define BIFURCATION 2
#define FLEFT       3
#define FRIGHT      4
#define CLOSE       5
#define FORWARD     6
#define FIN         7
#define ISINLINE    8

uint32_t DEG_PULSE;
/**
 * @brief Enumeración de la MEF para decodificar el protocolo
 * 
 */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

/**
 * @brief Enumeración de la lista de comandos
 * 
 */
typedef enum{
    ACK=            0x0D,
    GETALIVE=       0xF0,
    STARTCONFIG=    0xEE,
    FIRMWARE=       0xF1,
    LEDS=           0x10,
    PULSADORES=     0x12,
    IR=             0xA0,
    MOTOR=          0xA1,
    SERVO=          0xA2,
    HCSR=           0xA3,
    SPEED=          0xA4,
    CONFIG_SERVO=   0xA5,
    IRWConfig=      0xA6,
    IRBConfig=      0xA7,
    SERVOSTOPPED=   0xF2,
    INFO=           0xF3
}_eID;

/**
*@brief Conversion de tipo de datos
*
 */
typedef union{
    float f32;
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
    int8_t i8[4];
}u_dat;

/**
*@brief Banderas
*
 */
typedef union{
    struct{
        uint8_t bit0:   1;
        uint8_t bit1:   1;
        uint8_t bit2:   1;
        uint8_t bit3:   1;
        uint8_t bit4:   1;
        uint8_t bit5:   1;
        uint8_t bit6:   1;
        uint8_t bit7:   1;
    }su_bits;
    uint8_t byte;
}u_flags;

/**
*@brief Estados del botón
*
 */
typedef enum{
    DOWN,
    UP,
    FALLING,
    RISING
}_eEstados;

/**
*@brief Valores necesarios para cada boton
*
 */
typedef struct{
    uint8_t buttonValue = 0;
    _eEstados buttonState;
}s_Button;

/**
 * @brief Estructura de datos para el puerto serie
 * 
 */
typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart;      //!< Indice para saber en que parte del buffer circular arranca el ID
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[RINGBUFFLENGTH];   //!< Buffer circular de recepción
    uint8_t bufferTx[RINGBUFFLENGTH];   //!< Buffer circular de transmisión
}_sDato ;

/*************************************************************************************************/
/* Prototipo de Funciones */
/**
 * @brief Función que se llama en la interrupción de recepción de datos
 * Cuando se llama la función se leen todos los datos que llagaron.
 */
void onDataRx(void);

/**
 * @brief Decodifica las tramas que se reciben 
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(_sDato *);

/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(_sDato *);

/**
 * @brief  Función Heartbeat
 * Ejecuta las tareas del heartbeat
 */
void heartbeatTask();


/**
 * @brief Rutina para revisar los buffers de comunicación, decodificar y transmitar según sea necesario
 * @param datosCom Puntero a la estructura de datos del buffer
 * @param source flag que indica el módulo a donde transmitir
 */
void comunicationsTask(_sDato *datosCom, uint8_t source);

/**
 * @brief Envía el Alive de manera automática cuando el WIFI esta conectado
 * @param aliveAutoTime variable que controla el tiempo de envio
 */
void aliveAutoTask();

/**
 * @brief envía los datos de conexión al Wifi si estubieran definidos
 */
void autoConnectWifi(void);

/**
*@brief Funcion utilizada para convertir grados a pulsos en us
*
*@param degree: pide un valor en grados dessde -90 a 90
*@return int: retorna el valor de los pulsos
 */
int degToPulse(int8_t degree);

/**
*@brief  Funciones llamadas con la interrupción del sensor HC-SR04
*
 */
void echoRise();
void echoFall();

/**
*@brief Tiempo de espera variable para el movimiento del servo 
*
 */
void SERVO_Func();

/**
*@brief Emite un pulso de 10us en TRIGGER y calcula la distancia obtenida en us
*@details description La variable definida como distancia se almacena en us 
 */
void HCSR_Func();

/**
*@brief Lecutra del BUSIN de los sensores IR
*@details La variable IRByte almacena el valor de cada sensor de izquierda a derecha
*en el orden desde el bit menos significativo hacia el mas significativo
 */
void IR_Func();

/**
*@brief Se lee constantemente esta funcion para asignar el valor de pulso correspondiente a los motores
*
 */
void MOTOR_Func();

/**
*@brief Se llama cada 40ms a esta funcion para evaluar el estado del botón 
*dentro de esta funcion se llama a la maquina de estados del botón
 */
void Button_Func();

/**
*@brief Maquina de estados de los botones
*
*@param stateButton Pide como dato los valores del botón
*@return int Devuelve si el botón subió, bajo o sigue en el mismo estado
 */
int buttonState(s_Button *stateButton);

/**
*@brief Funciones llamadas con las interrupciones de los sensores INFRAROJOS
*@details Se utilizan dos variables, una para sumar los PPS y otra para sumar pulsos a gusto
 */
void velL();
void velR(); 

/**
*@brief Calcula los pulsos por segundo (PPS) y los almacena
*
 */
void SPEED_Func();//Funcion para calcular los PPS

/**
*@brief Modo 1: una vez encontrado el objetivo evalua los grados del servo y 
*mueve el auto la cantidad adecuada de pulsos contados 
 */
void targetFound();//Cuando encuentra el objetivo gira en su dirección

/**
*@brief Se mueve el servo hasta encontrar un objetivo a menos de la distancia maxima
*
 */
void whereTarget();//Si no tiene el obejtivo en frente lo busca

/**
*@brief Conserva los movimientos del auto en una linea
*
 */
void OnTheLine(uint8_t vel);//El auto se encuentra sobre una linea

/**
*@brief Si se encuentra un obstaculo lo rodea hasta volver a encontrarla
*
 */
void whereLine();//El auto perdió la linea

/**
*@brief Si pierde la linea evalúa a su alrededor y toma una deisción
*
 */
void checkingMaze();//Cuando el auto pierde la linea, evalua donde ir

void isInLine();

/**
*@brief Función para resumir el movimiento a seguir mientras no encuentra la linea
*
*@param velL pide como parametros las velocidades de cada motor
*@param velR 
 */
void motorsByIR(int8_t velL, int8_t velR);

/*****************************************************************************************************/

/* Configuración del Microcontrolador */
RawSerial pcCom(PA_9,PA_10); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT
DigitalOut HB(PC_13);
DigitalIn Pulsador(PA_4);
PwmOut Servo(PA_8);
InterruptIn HCSREcho(PB_12);
DigitalOut HCSRTrigger(PB_13);
BusIn IRin(PA_0, PA_1, PA_2);
DigitalOut MIN1(PB_15);
DigitalOut MIN2(PB_14);
DigitalOut MIN3(PB_7);
DigitalOut MIN4(PB_6);
PwmOut ENB(PB_0);
PwmOut ENA(PB_1);
InterruptIn VelL(PB_9);
InterruptIn VelR(PB_8);
PwmOut buzz(PA_11);


_eProtocolo estadoProtocolo;
/**
 * @brief estructura de datos del Wifi para configurar la conexion
 * 
 */
wifiData myWifiData;
 _sDato datosComSerie, datosComWifi;

Timer myTimer;
u_dat decom;
u_flags flags, flags2, checking;
s_Button button;
/**
*@param FIRMVERSION cadena que guarda la versión del codigo
 */
const char FIRMVERSION[] = "20221128_vA03";
/**
*@brief variables del servomotor
*
 */
int8_t deg, degrees, grados, lastDeg, miGrado;
uint8_t lastLine;
uint16_t msServo[2];
int32_t servoTime;
/**
*@brief variables del sensor HCSR
*
 */
int32_t timerHC, delayHC;
uint32_t distancia;
/**
*@brief variables de los IR
*
 */
uint16_t IRs[3];
uint8_t IRIndex, IRByte;
/**
*@brief variables de los motores
*
 */
int32_t motores[2];
/**
*@brief variables de los sensores de horquilla
*
 */
uint32_t PPS[2];
volatile uint8_t pulseL, pulseR, pulseLs=0, pulseRs=0;
uint32_t SpeedTime, lastTimeDebounceL, lastTimeDebounceR;
/**
*@brief variables de lboton
*
 */
int32_t buttonTimer;

/**
*@brief variables generales
*
 */
int32_t  checkMs, timerTarget, errorD, a;
volatile uint8_t rxData[256], RindexW, RindexR;
volatile uint8_t txData[256], TindexW, TindexR;
uint8_t ID, length, cks, mode=0, hbMode = WAITING, stage, buffer[20], mazeState = ONTHELINE, state, mode3VA, mode3VB;
uint32_t movesHB=0, heartBeat=0, mask=0x55555555, generalTimer, aliveAutoTime=0, waiting, difTimer;
/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(datosComWifi.bufferRx,&datosComWifi.indexWriteRx, sizeof(datosComWifi.bufferRx));

/*****************************************************************************************************/
/*********************************  Función Principal ************************************************/
int main(){
    myTimer.start();
    TindexR=0, TindexW=0;
    flags.byte = 0;
    flags2.byte = 0;
    pcCom.baud(115200);
    pcCom.attach(&onDataRx,RawSerial::RxIrq);
    myWifi.initTask();
    autoConnectWifi();
    
    MINSERVO = 500;
    MAXSERVO = 2400;    

    Servo.period_ms(20);
    Servo.pulsewidth_us(MAXSERVO);
    wait_ms(500);
    Servo.pulsewidth_us(MINSERVO);
    wait_ms(500);
    Servo.pulsewidth_us(1450);

    ENB.period_us(25000);
    ENA.period_us(25000);

    SpeedTime = myTimer.read_ms();
    delayHC = myTimer.read_us();
    checkMs = myTimer.read_ms();
    lastTimeDebounceR=myTimer.read_us();
    lastTimeDebounceL=myTimer.read_us();

    VelL.rise(&velL);
    VelR.rise(&velR);
    srand(time(NULL));
    while(true){
        myWifi.taskWifi();
        heartbeatTask();
        comunicationsTask(&datosComSerie, SERIE);
        comunicationsTask(&datosComWifi, WIFI);
        aliveAutoTask();     

        /*************** Evaluacion de los sensores y actuadores************/
        if((myTimer.read_ms() - checkMs) >= 40){
            checkMs = myTimer.read_ms();
            Button_Func();
           
        }
        IR_Func();
        /*if((myTimer.read_ms() - checkMs) >= 20){
            IR_Func();
        }*/
        HCSR_Func();
        SPEED_Func();
        SERVO_Func();
        MOTOR_Func();
        /*******************************************************************/
        /******************* SWITCH DE LOS MODOS ***************************/
        
        switch(mode){
            case IDLE:
                mask=0x55555555;
                break;
            case MODE1:
                switch(hbMode){
                    case WAITING:
                        mask = 1;
                        break;
                    case PLAYING:
                        mask = 0x5F;
                        break;
                    case PRESSING:
                        mask = 0x200401;
                        break;
                }
                if(isModeOn){
                    #define DISTANCIAMAX 1740
                    #define DISTANCIAMIN 58
                    switch(stage){
                    case 1:
                        if(distancia>=638){
                            LMOTOR = ((distancia-638)*8/111)+20;
                            RMOTOR = LMOTOR;
                        }else{
                            if(distancia<=522&&distancia>=DISTANCIAMIN){
                                LMOTOR = ((distancia-522)*-5/29)+20;
                                LMOTOR = LMOTOR*-1;
                                RMOTOR = LMOTOR;
                            }else{
                                LMOTOR = 0;
                                RMOTOR = 0;
                            }
                        }
                        if(distancia>DISTANCIAMAX)
                            stage=2;
                        break;
                    case 2:
                        whereTarget();
                        LMOTOR = 0;
                        RMOTOR = 0;
                        break;
                    case 3:
                        targetFound();
                        break;
                    }
                }
                break;
            case MODE2:
                switch(hbMode){
                    case WAITING:
                        mask = 0x5;
                        door=false;
                        blocker=false;
                        break;
                    case PLAYING:
                        mask = 0x15F;
                        break;
                    case PRESSING:
                        mask = 0x140A05;
                        break;
                }
                if(isModeOn){
                    if(!blocker){
                        OnTheLine(8);
                        if(IRByte==0x07){
                            if(lastLine < 0x04){//este 4 hay que verlo en la función "OnTheLine"
                                LMOTOR = 30;
                                RMOTOR = -30;
                            }else{
                                LMOTOR = -30;
                                RMOTOR = 30;
                            }
                        }
                        if(distancia<870){
                            blocker = true;
                            founded = false;
                            door = false;
                            stage = 1;
                            LMOTOR = 0;
                            RMOTOR = 0;
                        }
                    }else{
                        whereLine();
                    }
                }
                break;
            case MODE3:
                switch(hbMode){
                    case WAITING:
                        mask = 0x15;
                        break;
                    case PLAYING:
                        mask = 0x55F;
                        break;
                    case PRESSING:
                        mask = 0x2A05415;
                        break;
                }
                if(isModeOn){
                    switch(mazeState){
                    case ONTHELINE:
                        OnTheLine(6);
                        if(IRByte==0x07){
                            LMOTOR=0;
                            RMOTOR=0;
                            mazeState=ISINLINE;  
                            generalTimer=myTimer.read_ms(); 
                        }                
                        break;
                    case ISINLINE:
                        #define TIMER 160
                        OnTheLine(6);
                        if(IRByte!=0x07){
                            LMOTOR = 0;
                            RMOTOR = 0;
                            mazeState=ONTHELINE;
                        }
                        difTimer = myTimer.read_ms()-generalTimer;
                        if(lastLine < 0x04){
                            if(difTimer<TIMER*1/4){
                                LMOTOR = 40;
                                RMOTOR = -40;
                            }else{
                                if(difTimer<TIMER*3/4){
                                    LMOTOR = -40;
                                    RMOTOR = 40;
                                }else{
                                    if(difTimer<TIMER){
                                        LMOTOR = 40;
                                        RMOTOR = -40;
                                    }
                                }
                            }
                        }else{
                            if(difTimer<TIMER*1/4){
                                LMOTOR = -40;
                                RMOTOR = 40;
                            }else{
                                if(difTimer<TIMER*3/4){
                                    LMOTOR = 40;
                                    RMOTOR = -40;
                                }else{
                                    if(difTimer<TIMER){
                                        LMOTOR = -40;
                                        RMOTOR = 40;
                                    }
                                }
                            }
                        }
                        if(difTimer>=TIMER){
                            if(IRByte==0x07){
                                state=LEFT; 
                                door=false;
                                checking.byte=0;
                                mazeState=CHECKING;
                            }
                            LMOTOR=0;
                            RMOTOR=0;
                        }
                        break;
                    case CHECKING:
                        checkingMaze();
                        break;
                    case FLEFT:
                        motorsByIR(18,33);
                        break;
                    case FRIGHT:
                        motorsByIR(33,18);
                        break;
                    case CLOSE:
                        motorsByIR(30,-30);
                        break;
                    case FORWARD:
                         motorsByIR(30,30);
                    /*if(IRByte==0x07){
                        errorD=(870-distancia)/58;
                        LMOTOR=errorD+30;
                        RMOTOR=errorD*(-1)+30;
                    }else{
                        LMOTOR = 0;
                        RMOTOR = 0;
                        mazeState=ONTHELINE;
                    }*/
                    break;
                    case FIN:
                        if(door==false){
                            LMOTOR = -100;
                            RMOTOR = 100;
                            generalTimer=myTimer.read_ms();
                            door=true;
                        }
                        if(myTimer.read_ms()-generalTimer>3000){
                            LMOTOR = 0;
                            RMOTOR = 0;
                            door=false;
                            mode=IDLE;
                            hbMode=WAITING;
                        }
                        break;
                    }
                }
            break;
        }
        /*******************************************************************/   
        if(hbMode==WAITING){
            pulseLs=0, pulseRs=0;
            door=false, door2=false;
            located=false, founded=false;
            speedometer=false;
            stage=1;
            onLine=true;
            waiting=myTimer.read_ms();
        }
    }
    return 0;
}
/*****************************************************************************************************/
/****************************************** FUNCIONES ************************************************/
void decodeProtocol(_sDato *datosCom){
    static uint8_t nBytes=0;
    uint8_t indexWriteRxCopy=datosCom->indexWriteRx;

    while (datosCom->indexReadRx!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosCom->indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                datosCom->indexStart=datosCom->indexReadRx;
                nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
                        decodeData(datosCom); 
                    }
                }
               
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
}

void decodeData(_sDato *datosCom){
    wifiData *wifidataPtr;
    uint8_t *ptr; 
    uint8_t auxBuffTx[50], indiceAux=0, cheksum, sizeWifiData, indexBytesToCopy=0, numBytesToCopy=0/*, index=0*/;
    
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0; //this indexStart
    auxBuffTx[indiceAux++]=':';
    auxBuffTx[indiceAux++]='A';
    auxBuffTx[indiceAux++]='M';

    switch (datosCom->bufferRx[datosCom->indexStart+POSID]) {
        case GETALIVE:
            auxBuffTx[indiceAux++]=GETALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x05;         
            break;
        case FIRMWARE:
            auxBuffTx[indiceAux++]=FIRMWARE;
            for(uint8_t i=0; i<strlen(FIRMVERSION); i++){
                auxBuffTx[indiceAux++]=FIRMVERSION[i];
            }
            auxBuffTx[NBYTES]=0x11; 
            break;
        case STARTCONFIG: //Inicia Configuración del wifi 
            auxBuffTx[indiceAux++]=STARTCONFIG;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x05;     
            myWifi.resetWifi();
            sizeWifiData =sizeof(myWifiData);
            indexBytesToCopy=datosCom->indexStart+POSDATA;
            wifidataPtr=&myWifiData;

            if ((RINGBUFFLENGTH - indexBytesToCopy)<sizeWifiData){
                numBytesToCopy=RINGBUFFLENGTH-indexBytesToCopy;
                memcpy(wifidataPtr,&datosCom->bufferRx[indexBytesToCopy], numBytesToCopy);
                indexBytesToCopy+=numBytesToCopy;
                sizeWifiData-=numBytesToCopy;
                ptr= (uint8_t *)wifidataPtr + numBytesToCopy;
                memcpy(ptr,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }else{
                memcpy(&myWifiData,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }
            myWifi.configWifi(&myWifiData);
            break;
        case IR:
            auxBuffTx[indiceAux++]=IR;
            decom.ui16[0] = IRs[0];
            auxBuffTx[indiceAux++]=decom.ui8[0], auxBuffTx[indiceAux++]=decom.ui8[1];
            decom.ui16[0] = IRs[1];
            auxBuffTx[indiceAux++]=decom.ui8[0], auxBuffTx[indiceAux++]=decom.ui8[1];
            decom.ui16[0] = IRs[2];
            auxBuffTx[indiceAux++]=decom.ui8[0], auxBuffTx[indiceAux++]=decom.ui8[1];
            auxBuffTx[NBYTES]=0x0A; 
        break;
        case MOTOR:
            decom.i8[0] = datosCom->bufferRx[datosCom->indexStart+POSID+1], decom.i8[1] = datosCom->bufferRx[datosCom->indexStart+POSID+2];
            decom.i8[2] = datosCom->bufferRx[datosCom->indexStart+POSID+3], decom.i8[3] = datosCom->bufferRx[datosCom->indexStart+POSID+4];
            LMOTOR = decom.i32;
            decom.i8[0] = datosCom->bufferRx[datosCom->indexStart+POSID+5], decom.i8[1] = datosCom->bufferRx[datosCom->indexStart+POSID+6];
            decom.i8[2] = datosCom->bufferRx[datosCom->indexStart+POSID+7], decom.i8[3] = datosCom->bufferRx[datosCom->indexStart+POSID+8];
            RMOTOR = decom.i32;
            auxBuffTx[indiceAux++]=MOTOR;
            auxBuffTx[indiceAux++]=0x0D;
            auxBuffTx[NBYTES]=0x05;
            break;
        case SERVO:
            auxBuffTx[indiceAux++]=SERVO;
            if(!servoOnMove){
                Servo.pulsewidth_us(degToPulse(datosCom->bufferRx[datosCom->indexStart+POSID+1])); 
                servoTime=myTimer.read_ms();
                servoOnMove=true;
            }
            auxBuffTx[indiceAux++]=0x0D;
            auxBuffTx[NBYTES]=0x05; 
            break;
        case HCSR:
            auxBuffTx[indiceAux++]=HCSR;
            decom.ui32=distancia;
            auxBuffTx[indiceAux++] = decom.ui8[0];
            auxBuffTx[indiceAux++] = decom.ui8[1];
            auxBuffTx[indiceAux++] = decom.ui8[2];
            auxBuffTx[indiceAux++] = decom.ui8[3];
            auxBuffTx[NBYTES]=0x08;
            break;
        case SPEED:
            auxBuffTx[indiceAux++]=SPEED;
            decom.ui32=PPSL;
            auxBuffTx[indiceAux++]=decom.ui8[0], auxBuffTx[indiceAux++]=decom.ui8[1];
            auxBuffTx[indiceAux++]=decom.ui8[2], auxBuffTx[indiceAux++]=decom.ui8[3];
            decom.ui32=PPSR;
            auxBuffTx[indiceAux++]=decom.ui8[0], auxBuffTx[indiceAux++]=decom.ui8[1];
            auxBuffTx[indiceAux++]=decom.ui8[2], auxBuffTx[indiceAux++]=decom.ui8[3];
            auxBuffTx[NBYTES]=0x0C;
            break;
        case CONFIG_SERVO:
            if(datosCom->bufferRx[datosCom->indexStart+POSID+1]!=0){
                decom.ui8[0] = datosCom->bufferRx[datosCom->indexStart+POSID+2], decom.ui8[1] = datosCom->bufferRx[datosCom->indexStart+POSID+3];
                MINSERVO = decom.ui16[0];
                decom.ui8[0] = datosCom->bufferRx[datosCom->indexStart+POSID+4], decom.ui8[1] = datosCom->bufferRx[datosCom->indexStart+POSID+5];
                MAXSERVO = decom.ui16[0];
            }
            auxBuffTx[indiceAux++] = CONFIG_SERVO;
            decom.ui16[0] = MINSERVO;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            decom.ui16[0] = MAXSERVO;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            auxBuffTx[NBYTES] = 0x08;
            break;
        case SERVOSTOPPED:
            auxBuffTx[indiceAux++]=SERVO;
            auxBuffTx[indiceAux++]=0x0A;
            auxBuffTx[NBYTES] = 0x05;
            break;
        case INFO:
            auxBuffTx[indiceAux++]=INFO;
            auxBuffTx[indiceAux++]=mode;
            auxBuffTx[indiceAux++]=isModeOn;
            auxBuffTx[indiceAux++]=checking.byte;
            auxBuffTx[NBYTES] = 0x07;
            break;
        case IRWConfig:
            auxBuffTx[indiceAux++] = IRWConfig;
            decom.ui16[0] = 1;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            decom.ui16[0] = 1;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            decom.ui16[0] = 1;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            auxBuffTx[NBYTES] = 0x0A;
        break;
        case IRBConfig:
            auxBuffTx[indiceAux++] = IRBConfig;
            decom.ui16[0] = 0;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            decom.ui16[0] = 0;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            decom.ui16[0] = 0;
            auxBuffTx[indiceAux++] = decom.ui8[0], auxBuffTx[indiceAux++] = decom.ui8[1];
            auxBuffTx[NBYTES] = 0x0A;
        break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x04;
            break;
    }
   cheksum=0;
    for(uint8_t a=0; a<indiceAux; a++){
        cheksum ^= auxBuffTx[a];
        datosCom->bufferTx[datosCom->indexWriteTx++]=auxBuffTx[a];
    }
    datosCom->bufferTx[datosCom->indexWriteTx++]=cheksum;

}

void heartbeatTask(){
    if((myTimer.read_ms()-heartBeat)>100){
        heartBeat=myTimer.read_ms();
        HB.write((~mask) & (1<<movesHB));
        movesHB++;
        if(movesHB>=30)
            movesHB=0;
    } 
}

void comunicationsTask(_sDato *datosCom, uint8_t source){
    if(datosCom->indexReadRx!=datosCom->indexWriteRx ){
            decodeProtocol(datosCom);
    }

    if(datosCom->indexReadTx!=datosCom->indexWriteTx){
        if(source){
            if(pcCom.writeable()){
                pcCom.putc(datosCom->bufferTx[datosCom->indexReadTx++]);
            }
        }
        else{
            myWifi.writeWifiData(&datosCom->bufferTx[datosCom->indexReadTx++],1);  
        } 
    } 
}

void aliveAutoTask(){
    if(myWifi.isWifiReady()){
        if((myTimer.read_ms()-aliveAutoTime)>=ALIVEAUTOINTERVAL){
            aliveAutoTime=myTimer.read_ms();
            datosComWifi.bufferRx[datosComWifi.indexWriteRx+POSID]=GETALIVE;
            datosComWifi.indexStart=datosComWifi.indexWriteRx;
            decodeData(&datosComWifi);
        }
    }else{
        aliveAutoTime=0;
    }
}

void autoConnectWifi(){
    #ifdef AUTOCONNECTWIFI
        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
        myWifi.configWifi(&myWifiData);
    #endif
}

void HCSR_Func(){
    if((myTimer.read_us() - delayHC) > 100000){
        delayHC = myTimer.read_us();
        HCSROn = false;
        if(echoOn){
            echoOn = false;
            //distancia = 0xFFFFFFFF;
        }
    }
    if(!HCSROn){
        HCSRTrigger.write(1);
        if((myTimer.read_us() - delayHC) > 10){
            HCSRTrigger.write(0);
            HCSROn = true;
        }
    }else{
        if(!echoOn){
            HCSREcho.rise(&echoRise);
        }else{
            HCSREcho.fall(&echoFall);
        }
    }
}

void SERVO_Func(){
    static int8_t lastDeg;
    static uint16_t variable;
    if(servoOnMove){
        if(lastDeg<deg)
            variable = (lastDeg - deg)*-2;
        else
            variable = (lastDeg - deg)*2;
        if((myTimer.read_ms()-servoTime)>variable){
            servoOnMove=false;
            datosComWifi.bufferRx[datosComWifi.indexWriteRx+POSID]=SERVOSTOPPED;
            datosComWifi.indexStart=datosComWifi.indexWriteRx;
            decodeData(&datosComWifi);
            datosComSerie.bufferRx[datosComSerie.indexWriteRx+POSID]=SERVOSTOPPED;
            datosComSerie.indexStart=datosComSerie.indexWriteRx;
            decodeData(&datosComSerie);
            lastDeg = deg;
        }
    }
}

int degToPulse(int8_t degree){
    uint32_t pulse = 1450;
    if(degree>=-90 && degree<=90){
        pulse = -1 *degree * (((MAXSERVO-MINSERVO)/180)+1) + ((MAXSERVO-MINSERVO)/2+MINSERVO);
        deg = degree;
    }
    return pulse;
}

int buttonState(s_Button *stateButton){
    switch(stateButton->buttonState){
        case UP:
            if(stateButton->buttonValue == DOWN){
                stateButton->buttonState = FALLING;  
            }
        break;
        case DOWN:
            if(stateButton->buttonValue == UP){
                stateButton->buttonState = RISING;  
            }
        break;
        case RISING:
            if(stateButton->buttonValue == UP){
                stateButton->buttonState = UP;
                /*------------------------------*/
                return UNPRESS;//1
                /*------------------------------*/
            }else{
                stateButton->buttonState = DOWN;
            }
        break;
        case FALLING:
                if(stateButton->buttonValue == DOWN){
                stateButton->buttonState = DOWN;
                /*------------------------------*/
                return PRESS;//0
                /*------------------------------*/  
            }else{
                stateButton->buttonState = UP;
            }
        break;
        default:
            stateButton->buttonState = UP;
    }
    return 2;
}

void IR_Func(){
    for(IRIndex=0; IRIndex<3; IRIndex++){
        if(IRin.read() & (1<<IRIndex))
            IRs[IRIndex] = 0xFFFF;
        else
            IRs[IRIndex] = 0;
    }
    IRByte = IRin.read();
}

void Button_Func(){
    if(Pulsador.read()==1)
        button.buttonValue = UNPRESS;
    else
        button.buttonValue = PRESS;

    switch(buttonState(&button)){
        case PRESS:
            buttonTimer=myTimer.read_ms();
            isButtonPressed=true;
        break;
        case UNPRESS:
            isButtonPressed=false;
            if(!isModeOn){
                if((myTimer.read_ms()-buttonTimer)>100 && (myTimer.read_ms()-buttonTimer)<1000){
                    mode++;
                    if(mode>3)
                        mode=0;
                }else{
                    if((myTimer.read_ms()-buttonTimer)>1000 && (myTimer.read_ms()-buttonTimer)<5000 && mode!=IDLE){
                        hbMode=PLAYING;
                        isModeOn=true;
                    }
                }
                
            }else{
                if((myTimer.read_ms()-buttonTimer)>3000){
                    hbMode=WAITING;
                    isModeOn=false;
                    LMOTOR=0;
                    RMOTOR=0;
                }
            }
            datosComWifi.bufferRx[datosComWifi.indexWriteRx+POSID]=INFO;
            datosComWifi.indexStart=datosComWifi.indexWriteRx;
            decodeData(&datosComWifi);
            datosComSerie.bufferRx[datosComSerie.indexWriteRx+POSID]=INFO;
            datosComSerie.indexStart=datosComSerie.indexWriteRx;
            decodeData(&datosComSerie);
        break;
        default:
        ;
        if(isButtonPressed==true && mode!=IDLE && (myTimer.read_ms() - buttonTimer)>=1000){
            hbMode=PRESSING;
        }else{
            if(!isButtonPressed){
                if(isModeOn){
                    hbMode=PLAYING;
                }else{
                    hbMode=WAITING;
                }
            }
        }
    }
}

void MOTOR_Func(){
    if(LMOTOR<=100 && LMOTOR>=-100){
        if(LMOTOR < 0){
            MIN3.write(1);
            MIN4.write(0);
            ENB.pulsewidth_us((-1*LMOTOR)*220);
        }else{
            MIN3.write(0);
            MIN4.write(1);
            ENB.pulsewidth_us(LMOTOR*220);
        }
        if(LMOTOR == 0){
            MIN3.write(0);
            MIN4.write(0);
        }
    }
    if(RMOTOR<=100 && RMOTOR>=-100){
        if(RMOTOR < 0){  
            MIN1.write(0); 
            MIN2.write(1);
            ENA.pulsewidth_us((-1*RMOTOR*250));
        }else{
            MIN1.write(1);
            MIN2.write(0);
            ENA.pulsewidth_us(RMOTOR*250);
        }
        if(RMOTOR == 0){
            MIN1.write(0);
            MIN2.write(0);
        }
    }
}

void SPEED_Func(){
    if(myTimer.read_ms() - SpeedTime >= 1000){
        PPSL = pulseL;
        PPSR = pulseR;
        SpeedTime = myTimer.read_ms();
        pulseL = 0;
        pulseR = 0;
    }
}

void targetFound(){
    //grados=0;
    if(door==false){
        Servo.pulsewidth_us(degToPulse(0));
        if(degrees<0){
            DEG_PULSE = (-1*degrees)/16;
        }else{
            DEG_PULSE = degrees/16;
        }
        door=true;
    }
    //if(myTimer.read_ms()-waiting>1000){
    if(pulseRs>=DEG_PULSE && pulseLs>=DEG_PULSE){
        degrees = 0;
        DEG_PULSE = 0;
        speedometer=false;
        pulseRs = 0;
        pulseLs = 0;
        LMOTOR = 0;
        RMOTOR = 0;
        if(distancia<=DISTANCIAMAX && distancia>DISTANCIAMIN){
            stage=1;
            door=false;
        }else{
            stage=2;
            door=false;
        }
    }else{
        if(degrees<0){
            RMOTOR = 30;
            LMOTOR = -30;
            targetDir = LEFT;
        }else{
            if(degrees>0){
                RMOTOR = -30;
                LMOTOR = 30; 
                targetDir = RIGHT;
            }
        }
    }
    //}
}

void whereTarget(){
    if(!servoOnMove){
        if(door==false){
            timerTarget = myTimer.read_ms();
            if(targetDir == LEFT){
                grados=-90;
            }else{
                grados=90;
            }
            door=true;
        }
        if(myTimer.read_ms()-timerTarget > 300){
            timerTarget = myTimer.read_ms();
            if(targetDir==LEFT){
                grados += 10;
            }else{
                grados -= 10;
            }
            if(targetDir==LEFT && grados > 90){
                door=false;
                grados=-90;
            }else{
                if(targetDir==RIGHT && grados < -90){
                    door=false;
                    grados=90;
                }
            }
            degrees=grados;
            Servo.pulsewidth_us(degToPulse(grados));
            servoOnMove=true;
        }
        if(distancia<=DISTANCIAMAX && distancia>DISTANCIAMIN){
            speedometer=true;
            pulseRs = 0;
            pulseLs = 0;
            stage=3;
            door=false;
        }
        
    }
}

void OnTheLine(uint8_t vel){
    switch(IRByte){
        //       R C L    
        case 0x00://0 0 0
        break;
        case 0x01://0 0 1  
            LMOTOR = 5*vel;
            RMOTOR = 4*vel;
        break;
        case 0x02://0 1 0
        break;
        case 0x03://0 1 1
            LMOTOR=5*vel;
            RMOTOR=3*vel;
        break;
        case 0x04://1 0 0                                
            LMOTOR=4*vel;
            RMOTOR=5*vel;
        break;
        case 0x05://1 0 1
            LMOTOR = 5*vel;
            RMOTOR = 5*vel;
        break;
        case 0x06://1 1 0
            LMOTOR = 3*vel;
            RMOTOR = 5*vel;
        break;
        case 0x07://1 1 1
        break;
    }
    if(IRByte!=0x07 && IRByte!=5)
        lastLine = IRByte;
}

void whereLine(){
    switch(stage){
    case 1:
        if(door==false){
            LMOTOR = -25;
            RMOTOR = 25;
            door=true;
            speedometer=true;
            pulseLs=0;
            pulseRs=0;
        }
        if(pulseLs>=11 && pulseRs>=11){
            Servo.pulsewidth_us(degToPulse(90));
            LMOTOR = 0;
            RMOTOR = 0;
            speedometer=false;
            pulseLs=0;
            pulseRs=0;
            stage = 2;
        }
        break;
    case 2://avanza hasta dejar de detectar el obstaculo
        if(distancia>2900){
            LMOTOR = 55;
            RMOTOR = 20;
            door=false;
        }else{
            errorD=(1160-distancia)/58;
            LMOTOR=errorD*(-1)+55;
            RMOTOR=errorD+55;
            if(IRByte<0x07){
                generalTimer = myTimer.read_ms();
                Servo.pulsewidth_us(degToPulse(0));
                LMOTOR = 0;
                RMOTOR = 0;
                lastLine=0x06;
                blocker=false;
                door=false;
            }
        }
    }
}

void checkingMaze(){
    #define TCHECK 1500
    switch(state){
    case LEFT:
        if(door==false){
            checking.byte=0;
            Servo.pulsewidth_us(degToPulse(-90));
            generalTimer=myTimer.read_ms();
            door=true;
        }
        if(myTimer.read_ms()-generalTimer > TCHECK){
            if(distancia<1740){
                checking.su_bits.bit0=1;
            }else{
                checking.su_bits.bit0=0;
            }
            door=false;
            state=RIGHT;
        }
        break;
    case RIGHT:
        if(door==false){
            Servo.pulsewidth_us(degToPulse(90));
            generalTimer=myTimer.read_ms();
            door=true;
        }
        if(myTimer.read_ms()-generalTimer > TCHECK){
            if(distancia<1740){
                checking.su_bits.bit1=1;
            }else{
                checking.su_bits.bit1=0;
            }
            door=false;
            state=CENTER;
        }
        break;
    case CENTER:
        if(door==false){
            Servo.pulsewidth_us(degToPulse(0));
            generalTimer=myTimer.read_ms();
            door=true;
        }
        if(myTimer.read_ms()-generalTimer > TCHECK){
            if(distancia<=1740){
                checking.su_bits.bit2=1;
                checking.su_bits.bit3=0;
            }else{
                if(distancia>=2030 && distancia<=3770){
                    checking.su_bits.bit3=1;
                }else{
                    checking.su_bits.bit3=0;
                }
                checking.su_bits.bit2=0;
            }
            door=false;
            state= CASE;
        }
        break;
    case CASE://sección del bite: 0000 Cl Cc R L
        switch(checking.byte){ // la idea de este algoritmo es que con el valor de un byte el auto sepa para donde ir
        case 0x01: //libre al centro y derecha
            decision=rand()%2;
            if(decision){
                mazeState=FRIGHT;
            }else{
                mazeState=FORWARD;
                //Servo.pulsewidth_us(degToPulse(-90));
            }
            break;
        case 0x02://libre al centro e izquierda
            decision = rand() % 2;
            if(decision){
                mazeState=FLEFT;
            }else{
                mazeState=FORWARD;
                //Servo.pulsewidth_us(degToPulse(90));
            }
            break;
        case 0x04: //bifurcación
            decision = rand()%2;
            if(decision){
                mazeState=FLEFT;
            }else{
                mazeState=FRIGHT;
            }
            break;
        case 0x05: //libre derecha
            mazeState=FRIGHT;
            break;
        case 0x06: //libre izquierda
            mazeState=FLEFT;
            break;
        case 0x07: //camino cerrado
            mazeState=CLOSE;
            break;
        case 0x08: //salida
            mazeState=FIN;
            break;
        default:
            mazeState=ONTHELINE;
            break;
        }
        datosComWifi.bufferRx[datosComWifi.indexWriteRx+POSID]=INFO;
        datosComWifi.indexStart=datosComWifi.indexWriteRx;
        decodeData(&datosComWifi);
        datosComSerie.bufferRx[datosComSerie.indexWriteRx+POSID]=INFO;
        datosComSerie.indexStart=datosComSerie.indexWriteRx;
        decodeData(&datosComSerie);
        checking.byte=0;
        break;
    }
}

void isInLine(){
    #define PULSES 2
    switch(state){
    case 1:
        if(door==false){
            LMOTOR=20;
            RMOTOR=-20;
            pulseLs=0;
            pulseRs=0;
            speedometer=true;
            door=true;
        }
        if(pulseLs>=PULSES && pulseRs>=PULSES){
            state=2;
            speedometer=false;
            pulseLs=0;
            pulseRs=0;
            LMOTOR=0;
            RMOTOR=0;
            door=false;
        }
        if(IRByte!=0x07){
            mazeState=ONTHELINE;
            door=false;
            speedometer=false;
            LMOTOR=0;
            RMOTOR=0;
            pulseLs=0;
            pulseRs=0;
        }
        break;   
    case 2:
        if(door==false){
            LMOTOR=-20;
            RMOTOR=20;
            pulseLs=0;
            pulseRs=0;
            speedometer=true;
            door=true;
        }
        if(pulseLs>=PULSES*2 && pulseRs>=PULSES*2){
            state=3;
            speedometer=false;
            pulseLs=0;
            pulseRs=0;
            LMOTOR=0;
            RMOTOR=0;
            door=false;
        }
        if(IRByte!=0x07){
            mazeState=ONTHELINE;
            door=false;
            speedometer=false;
            LMOTOR=0;
            RMOTOR=0;
            pulseLs=0;
            pulseRs=0;
        }
        break; 
    case 3:
        if(door==false){
            LMOTOR=20;
            RMOTOR=-20;
            pulseLs=0;
            pulseRs=0;
            speedometer=true;
            door=true;
        }
        if(pulseLs>=PULSES && pulseRs>=PULSES){
            mazeState=CHECKING;   
            state=LEFT;
            door=false;
            checking.byte=0;
            speedometer=false;
            LMOTOR=0;
            RMOTOR=0;
            pulseLs=0;
            pulseRs=0;
        }
        if(IRByte!=0x07){
            mazeState=ONTHELINE;
            door=false;
            speedometer=false;
            LMOTOR=0;
            RMOTOR=0;
            pulseLs=0;
            pulseRs=0;
        }
        break; 
    }
}

void motorsByIR(int8_t velL, int8_t velR){
    if(IRByte==0x07){
        LMOTOR = velL;
        RMOTOR = velR;
    }else{
        LMOTOR = 0;
        RMOTOR = 0;
        //generalTimer=myTimer.read_ms();
        mazeState=ONTHELINE;
    }
}
/*****************************************************************************************************/
/*************************************** INTERRUPCIONES **********************************************/
void onDataRx(void){
    while (pcCom.readable())
    {
        datosComSerie.bufferRx[datosComSerie.indexWriteRx++]=pcCom.getc();
    }
}

void echoRise(){
    timerHC = myTimer.read_us();
    echoOn = true;
}

void echoFall(){
    echoOn = false;  
    if (myTimer.read_us()>=timerHC)
        distancia = myTimer.read_us() - timerHC;
    else
       distancia = myTimer.read_us()-timerHC+0xFFFFFFFF;
}

void velL(){
    if(VelL.read()==1 && (myTimer.read_us()-lastTimeDebounceL>=1000)){
        lastTimeDebounceL = myTimer.read_us();
        pulseL++;
        if(speedometer){
            pulseLs++;
        }
    }
}

void velR(){
    if(VelR.read()==1 && (myTimer.read_us()-lastTimeDebounceR>=1000)){
        lastTimeDebounceR = myTimer.read_us();
        pulseR++;
        if(speedometer){
            pulseRs++;
        }
    }
}
/*****************************************************************************************************/