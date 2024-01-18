//Authors: Oliver Saari, Janne Tuikka and Roope Paananen
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"


/* Stacks defines */

#define STACKSIZE 2048
Char uartTaskStack[STACKSIZE];
Char sensorTaskStack[STACKSIZE];
Char updateTaskStack[STACKSIZE/2];
Char buzzerTaskStack[STACKSIZE/4];


/* Handles */

static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
// Led pin global variables
static PIN_Handle ledHandle;
static PIN_State ledState;
// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;
// UART pin global variables
//static UART_Handle uart;
//static UART_Params uartParams;
// Button pin global variables
static PIN_Handle button1Handle;
static PIN_State button1State;
static PIN_Handle button2Handle;
static PIN_State button2State;


/* Global configs */

PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};
PIN_Config button1Config[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
   PIN_TERMINATE
};
PIN_Config button2Config[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
   PIN_TERMINATE
};
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};
// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


/* States */

enum state {AFK=1, READ_SENSOR, UPDATE, NEW_MSG};
enum state myState = AFK;
enum musicState {SILENT=1, BEEP, ALERT, MUSIC, ESCAPE};
enum musicState myMusic = SILENT;
enum msgState {NOTHING=1, DATA, MSG1, MSG2, GOTHROUGH};
enum msgState myMsg = NOTHING;


/* Global variables */

char uartBuffer[80];
char msgBuffer[80];

float mpuData [10][3] = {{0.0f}};

int tila = 0;
char lahetys[80];
const char ID[5] = "3047";


/* Music */

int beep[] = {
  2000, 2500, 3000
};
int beep_durations[] = {
  8, 8, 4
};

int alert[] = {
  3000, 2500, 2000, 2250
};
int alert_durations[] = {
  8, 8, 8, 2
};

int music[] = {
  659, 659, 659,
  659, 659, 659,
  659, 784, 523, 587,
  659,
  698, 698, 698, 698,
  698, 659, 659, 659, 659,
  659, 587, 587, 659,
  587, 784
};
int music_durations[] = {
  8, 8, 4,
  8, 8, 4,
  8, 8, 8, 8,
  2,
  8, 8, 8, 8,
  8, 8, 8, 16, 16,
  8, 8, 8, 8,
  4, 4
};

int escapemusic[] = {
                     330, 349, 392, 659, 523, 587, 523, 523, 494, 494, 294,
                     330, 349, 587, 494, 523, 494, 440, 392, 392, 330, 349,
                     392, 523, 587, 659, 587, 523, 440, 587, 659, 698, 659,
                     587, 392, 698, 659, 587, 523, 523/*, 0, 523, 0, 523, 659,
                     523, 587, 0, 587, 587, 587, 0, 587, 698, 587, 659, 0, 659,
                     659, 659, 0, 659, 784, 659, 698, 0, 698, 698, 659, 587, 392,
                     494, 523, 523, 0*/
};

int escapemusic_durations[] = {
                     8, 8, 4, 4, 4, 8, 8, 4, 4, 4, 8,
                     8, 4, 4, 4, 8, 8, 4, 4, 4, 8, 8,
                     4, 8, 8, 4, 8, 8, 4, 8, 8, 4, 8,
                     8, 4, 4, 4, 4, 2, 4/*, 4, 4, 8, 8, 4,
                     4, 4, 8, 8, 2, 4, 8, 8, 4, 4, 4, 8, 8,
                     2, 4, 8, 8, 4, 4, 4, 8, 8, 4, 8, 8, 2,
                     2, 2, 4, 4*/
};


/* Function prototypes */
int readData();
int readMsg();

//This function checks if the message is for us and saves it to msgBuffer.
static void uartFxn(UART_Handle handle, void *rxBuf, size_t len) {
    if(strstr(rxBuf,ID) != NULL) {
        memset(msgBuffer, '\0', sizeof(msgBuffer));
        sprintf(msgBuffer,"%s",rxBuf);
        myState = NEW_MSG;
        myMsg = MSG1;
        //return;
    }
    else {
        myState = NEW_MSG;
        myMsg = GOTHROUGH;
    }
}
//Function checks if button 0 is pressed. When pressed, state will be switched to READ_SENSOR.
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    if (PIN_getInputValue(Board_BUTTON0) != 1){
        pinValue = 1;
        myState = READ_SENSOR;
        //sprintf(data,"id:3047,session:start");
        //UART_write(uart, data, strlen(data)+1);
    }
    else if(myState == READ_SENSOR){
        pinValue = 0;
        myState = UPDATE;
    }
    else{
        pinValue = 0;
        myState = AFK;
    }
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
}

//Checks if button 1 is pressed and starts to play music when it is pressed.
void musicFxn(PIN_Handle handle, PIN_Id pinId) {
    if (PIN_getInputValue(Board_BUTTON1) != 1){
        myMusic = MUSIC;
    } else {
        myMusic = SILENT;
    }
}

//Setups I2C and MPU. When state is READ_SENSOR, it collects acceleration data from MPU and then switches state to UPDATE.
void sensorTaskFxn(UArg arg0, UArg arg1){
    int laskuri = 0;
    float ax, ay, az, gx, gy, gz;
    //char tulostus[40];
    
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (hMpuPin == NULL) {
            System_abort("Pin open failed!");
        }


    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    while(1){
        if (myState == READ_SENSOR){
            if(laskuri <= 9){
                mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
                //sprintf(tulostus,"id:3047, time:%d, ax:%.4f, ay:%.4f, az:%.4f, gx:%.4f, gy:%.4f, gz:%.4f", laskuri, ax, ay, az, gx, gy, gz);
                //UART_write(uart, tulostus, strlen(tulostus)+1);
                mpuData[laskuri][0] = ax;
                mpuData[laskuri][1] = ay;
                mpuData[laskuri][2] = az;
                laskuri++;
            }
            else{
                myState = UPDATE;
            }
        }
        else{
            laskuri = 0;
        }
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

//If state is UPDATE, the function calls readData function to determine the movement and amount. Sets correct states.
void updateTaskFxn(UArg arg0, UArg arg1){

    char muutetaan[4][10] = {"EAT:","PET:","EXERCISE:","ACTIVATE:"};
    //char data[30];
    while (1){
        if (myState == UPDATE){
            //sprintf(data,"id:3047,session:end");
            //UART_write(uart, data, strlen(data)+1);
            int amount = readData();
            //System_printf("%d, %d", tila, amount);
            //System_flush();
            switch(tila){
                case 0:
                    myMsg = NOTHING;
                    break;
                case 1:
                    sprintf(lahetys,"id:3047,%s%d",muutetaan[0], amount);
                    //UART_write(uart, lahetys, strlen(lahetys)+1);
                    myMusic = BEEP;
                    tila = 0;
                    amount = 0;
                    myMsg = DATA;
                    break;
                case 2:
                    sprintf(lahetys,"id:3047,%s%d",muutetaan[1], amount);
                    //UART_write(uart, lahetys, strlen(lahetys)+1);
                    myMusic = BEEP;
                    tila = 0;
                    amount = 0;
                    myMsg = DATA;
                    break;
                case 3:
                    sprintf(lahetys,"id:3047,%s%d",muutetaan[2], amount);
                    //UART_write(uart, lahetys, strlen(lahetys)+1);
                    myMusic = BEEP;
                    tila = 0;
                    amount = 0;
                    myMsg = DATA;
                    break;
                case 4:
                    sprintf(lahetys,"id:3047,%s1;1;1",muutetaan[3]);
                    //UART_write(uart, lahetys, strlen(lahetys)+1);
                    tila = 0;
                    amount = 0;
                    myMsg = DATA;
                    break;
                default:
                    break;
            }
            myState = NEW_MSG;
        }
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
}
//Sets up UART. Saves the message to msgBuffer and sends it to backend.
static void uartTaskFxn(UArg arg0, UArg arg1) {

    char tempStr[70];
    UART_Handle uart;
    UART_Params uartParams;

   // Serial communication initialization
   UART_Params_init(&uartParams);
   uartParams.writeDataMode = UART_DATA_TEXT;
   uartParams.readDataMode = UART_DATA_TEXT;
   uartParams.readEcho = UART_ECHO_ON;
   uartParams.readMode=UART_MODE_CALLBACK;
   uartParams.readCallback  = &uartFxn; // Interrupt handler
   uartParams.baudRate = 9600; // speed 9600bau
   uartParams.dataLength = UART_LEN_8; // 8
   uartParams.parityType = UART_PAR_NONE; // n
   uartParams.stopBits = UART_STOP_ONE; // 1

    // UART initialization
   uart = UART_open(Board_UART, &uartParams);
   if (uart == NULL) {
      System_abort("Error opening the UART");
   }

   UART_read(uart, uartBuffer, 80);

   while(1) {
      if(myState == NEW_MSG) {
          switch(myMsg) {
          case NOTHING:
              break;
          case DATA:
              UART_write(uart, lahetys, strlen(lahetys)+1);
              break;
          case MSG1:
              switch(readMsg()) {
              case 0:
                  sprintf(tempStr,"%s",msgBuffer);
                  memset(msgBuffer, '\0', sizeof(msgBuffer));
                  sprintf(msgBuffer,"id:3047,MSG1:%s",tempStr);
                  break;
              case 1:
                  sprintf(tempStr,"%s",msgBuffer);
                  memset(msgBuffer, '\0', sizeof(msgBuffer));
                  sprintf(msgBuffer,"id:3047,MSG1:%s",tempStr);
                  myMusic = ALERT;
                  break;
              case 2:
                  sprintf(tempStr,"%s",msgBuffer);
                  memset(msgBuffer, '\0', sizeof(msgBuffer));
                  sprintf(msgBuffer,"id:3047,MSG1:%s",tempStr);
                  myMusic = ESCAPE;
                  break;
              }
              UART_write(uart, msgBuffer, strlen(msgBuffer)+1);
              memset(tempStr, '\0', sizeof(tempStr));
              UART_read(uart, uartBuffer, 80);
              break;
          case MSG2:
              break;
          case GOTHROUGH:
              UART_read(uart, uartBuffer, 80);
              break;
          }
          myState = AFK;
      }

      // Sleep 100ms
      Task_sleep(100000 / Clock_tickPeriod);
   }
}

//Controls the sound system using music states.
Void buzzerTaskFxn(UArg arg0, UArg arg1) {

    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
    }

    while(1) {
        if(myMusic == ALERT) {
            int size = sizeof(alert_durations) / sizeof(int);
            buzzerOpen(hBuzzer);
            int note;
            for (note = 0; note < size; note++) {
                //to calculate the note duration, take one second divided by the note type.
                //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                int duration = 1000000 / alert_durations[note];
                buzzerSetFrequency(alert[note]);
                Task_sleep(duration / Clock_tickPeriod);
            }
            buzzerClose();
            myMusic = SILENT;
        }

        if(myMusic == MUSIC) {
            int size = sizeof(music_durations) / sizeof(int);
            buzzerOpen(hBuzzer);
            int note;
            for (note = 0; note < size; note++) {
                int duration = 1500000 / music_durations[note];
                buzzerSetFrequency(music[note]);
                Task_sleep(duration / Clock_tickPeriod);
                int pauseBetweenNotes = duration * 0.30;
                buzzerSetFrequency(0);
                Task_sleep(pauseBetweenNotes / Clock_tickPeriod);
            }
            buzzerClose();
            myMusic = SILENT;
            tila = 4;
            myState = UPDATE;
        }

        if(myMusic == ESCAPE) {
            int size = sizeof(escapemusic_durations) / sizeof(int);
            buzzerOpen(hBuzzer);
            int note;
            for (note= 0; note < size; note++) {
                int duration = 1000000 / escapemusic_durations[note];
                buzzerSetFrequency(escapemusic[note]);
                Task_sleep(duration / Clock_tickPeriod);
                int pauseBetweenNotes = duration * 0.30;
                buzzerSetFrequency(0);
                Task_sleep(pauseBetweenNotes / Clock_tickPeriod);
            }
            buzzerClose();
            myMusic = SILENT;
        }

        if(myMusic == BEEP) {
            int size = sizeof(beep_durations) / sizeof(int);
            buzzerOpen(hBuzzer);
            int note;
            for (note = 0; note < size; note++) {
                //to calculate the note duration, take one second divided by the note type.
                //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
                int duration = 1000000 / beep_durations[note];
                buzzerSetFrequency(beep[note]);
                Task_sleep(duration / Clock_tickPeriod);
            }
            buzzerClose();
            myMusic = SILENT;
        }

        if(myMusic == SILENT && PIN_getInputValue(Board_BUZZER) == 1) {
            buzzerClose();
        }

        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

//Checks what kind of movement is in the received MPU data. Sets the variable "tila" correct and returns amount of movement.
int readData(){
    int pet = 0;
    int exercise = 0;
    int eat = 0;
    float compare_x = mpuData[0][0];
    float compare_y = mpuData[0][1];
    float compare_z = mpuData[0][2];
    mpuData[0][0] = 0; mpuData[0][1] = 0; mpuData[0][2] = 0;
    int i;
    for (i = 1; i <= 9; i++){
        if (compare_x > mpuData[i][0]){
            compare_x = mpuData[i][0];
        }
        else{
            if (mpuData[i][0] > compare_x + 0.5){
                eat++;
                compare_x = mpuData[i][0];
            }
        }
        mpuData[i][0] = 0;
        if (compare_y > mpuData[i][1]){
            compare_y = mpuData[i][1];
        }
        else{
            if (mpuData[i][1] > compare_y + 0.5){
                pet++;
                compare_y = mpuData[i][1];
            }
        }
        mpuData[i][1] = 0;
        if(compare_z > mpuData[i][2]){
            compare_z = mpuData[i][2];
        }
        else{
            if(mpuData[i][2] > compare_z + 1.5){
                exercise++;
                compare_z = mpuData[i][2];
            }
        }
        mpuData[i][2] = 0;
    }
    if(eat != 0 && exercise == 0){
        tila = 1;
        return eat;
    }
    if(pet != 0 && exercise == 0){
        tila = 2;
        return pet;
    }
    if(exercise != 0){
        tila = 3;
        return exercise;
    }
    return 0;
}

//Checks if there is any messages in msgBuffer. If so, it saves the message to msgBuffer
//and returns the correct value for the switch-case in uartFxn():
int readMsg() {
    char *temp = strstr(msgBuffer,"Too late");
    if(temp != NULL) {
        sprintf(msgBuffer,"I will escape!!");
        return 2;
    }
    temp = strstr(msgBuffer,"Severe warning about my wellbeing");
    if(temp != NULL) {
        sprintf(msgBuffer,"Severe warning about my wellbeing");
        return 1;
    }
    temp = strstr(msgBuffer,"Running low on food");
    if(temp != NULL) {
        sprintf(msgBuffer,"Running low on food");
        return 1;
    }
    temp = strstr(msgBuffer,"I could use a scratch");
    if(temp != NULL) {
        sprintf(msgBuffer,"I could use a scratch");
        return 1;
    }
    temp = strstr(msgBuffer,"Calm down");
    if(temp != NULL) {
        sprintf(msgBuffer,"I will die if I eat that!");
        return 0;
    }
    temp = strstr(msgBuffer,"Too fitness");
    if(temp != NULL) {
        sprintf(msgBuffer,"I cannot move anymore!");
        return 0;
    }
    temp = strstr(msgBuffer,"Feels good");
    if(temp != NULL) {
        sprintf(msgBuffer,"My skin will come off!");
        return 0;
    }
    sprintf(msgBuffer,"");
    return 0;
}

//Sets the task handles and parameters etc.
int main(void){

        Task_Handle sensorTask;
        Task_Params sensorTaskParams;
        Task_Handle updateTask;
        Task_Params updateTaskParams;
        Task_Handle buzzerTask;
        Task_Params buzzerTaskParams;
        Task_Handle uartTask;
        Task_Params uartTaskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    button1Handle = PIN_open(&button1State, button1Config);
    if(!button1Handle) {
        System_abort("Error initializing button pins\n");
    }

    button2Handle = PIN_open(&button2State, button2Config);
    if(!button2Handle) {
        System_abort("Error initializing button pins\n");
    }

    /* Open LED pins */
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority = 2;
    sensorTask = Task_create((Task_FuncPtr)sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTask == NULL) {
        System_abort("Sensor task create failed!");
    }

    Task_Params_init(&updateTaskParams);
    updateTaskParams.stackSize = STACKSIZE/2;
    updateTaskParams.stack = &updateTaskStack;
    updateTaskParams.priority = 2;
    updateTask = Task_create((Task_FuncPtr)updateTaskFxn, &updateTaskParams, NULL);
    if (updateTask == NULL) {
        System_abort("Update task create failed!");
    }

    Task_Params_init(&buzzerTaskParams);
    buzzerTaskParams.stackSize = STACKSIZE/4;
    buzzerTaskParams.stack = &buzzerTaskStack;
    buzzerTaskParams.priority = 2;
    buzzerTask = Task_create((Task_FuncPtr)buzzerTaskFxn, &buzzerTaskParams, NULL);
    if (buzzerTask == NULL) {
        System_abort("Buzzer task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority = 2;
    uartTask = Task_create((Task_FuncPtr)uartTaskFxn, &uartTaskParams, NULL);
    if (uartTask == NULL) {
        System_abort("Uart task create failed!");
    }

    if (PIN_registerIntCb(button1Handle, &buttonFxn) != 0) {
        System_abort("Error registering button callback function");
    }
    if (PIN_registerIntCb(button2Handle, &musicFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    /* Start BIOS */
    BIOS_start();

    return (0);
}
