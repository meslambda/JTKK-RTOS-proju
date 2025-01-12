/*
// Authors: Santeri Paavola

Tasks
sensorTaskFxn:  
- Parameters: stackSize = STACKSIZE, stack = &sensorTaskStack, priority of 2.
- Function is to gather data from MPU2950.

uartFxn:
- Parameters: stackSize = STACKSIZE, stack = &sensorTaskStack, priority of 2.
- Function is to communicate with desktop wia UART.

taskFxn
- Parameters: stackSize = STACKSIZE, stack = &taskStack.
- Function is to be able to play music 


Interruptions
- UART interruption: Gets interruption, when device receives morse code from course's Serial Client. Handler is uartTaskFxn. When triggered, receives and saves morse code to global ring buffer.
- Button interruption 1: Triggers when button 1 is pushed. Handler is buttonFxn. When triggered, changes bool to play "music" and changes led value.
- Button interruption 2: Triggers when button 2 is pushed. Handler is buttonFxn2. When triggered, change booleans to play chime and enable/disable sending movement morse to desktop.


Serial communications
- I2C to access MPU2950. Uses only the sensortag and is used to save MPU data to global struct.
- UART to communicate with course's Serial Client. Uses desktop and the sensortag. Is used to send/receive morse characters to and from the desktop.
*/





/* C Standard library */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
#include "sensors/buzzer.h"

#define STACKSIZE 2048
char taskStack[STACKSIZE];
char sensorTaskStack[STACKSIZE];
char uartTaskStack[STACKSIZE];

/* Prototypes */
void detectMoves();
void playSeppo();
void sendMessage(UART_Handle *givenhandle);
static void uartFxn(UART_Handle handle, void *rxBuf, size_t len);
void playNote(uint16_t duration, uint16_t pause, uint16_t freq);
void receiveNotification();
void saveMpuData(I2C_Handle *bus);

// For buzzer
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
PIN_Config cBuzzer[] = {
    Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

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

// machine states
enum state { WAITING=1, DATA_READY, SENDING_MSG };
enum state programState = WAITING;

// Some RTOS vars.
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle buttonHandle2;
static PIN_State buttonState2;
static PIN_Handle ledHandle;
static PIN_State ledState;

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // The configuration table is always terminated with this constant
};

PIN_Config buttonConfig2[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // The configuration table is always terminated with this constant
};

// The constant Board_LED0 corresponds to one of the LEDs
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // The configuration table is always terminated with this constant
};

// global variables for mpu
struct accelerometer_t {
    float ax, ay, az;
};

struct gyroscope_t {
    float gx, gy, gz;
};

struct mpu_sample_t {
    uint32_t timestamp;
    struct accelerometer_t accel;  // ax, ay, az
    struct gyroscope_t gyro;  // gx, gy, gz
} /*mpusamples*/;

char movement[30];

struct mpu_sample_t samples[10];

bool playMusic = false;
bool stopMovement = false;
bool stopSound = false;

uint8_t bufferSize = 10;
char msgBuffer[10];
uint8_t writeIndex = 0, readIndex = 0;

uint8_t uartBuffer[30];





/* Task Functions */
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);

    playMusic = true;
}

void buttonFxn2(PIN_Handle handle, PIN_Id pinId) {

    stopMovement = !stopMovement;
    stopSound = false;
}

void uartTaskFxn(UArg arg0, UArg arg1) {

    // for the task
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    //uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1
    uartParams.readMode      = UART_MODE_CALLBACK;
    uartParams.readCallback  = &uartFxn;

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }

    UART_read(uart, &uartBuffer, 1);

    while (1) {

        // notifies user of incoming morse message
        // doesn't check machine state and notifies user immediately
        receiveNotification();

        if (programState == SENDING_MSG) {

            // sends message to workstation terminal
            sendMessage(&uart);
            programState = WAITING;
        }

        Task_sleep(1000000 / Clock_tickPeriod);
    }
}

static void uartFxn(UART_Handle handle, void *rxBuf, size_t len) {
    // Saves incoming morse to global var.

    if ((writeIndex + 1) % bufferSize == readIndex) {
        return;
    }
    else if (uartBuffer[0] == '-') {
        msgBuffer[writeIndex] = '-';
        writeIndex = (writeIndex + 1) % bufferSize;
    }
    else if (uartBuffer[0] == '.') {
        msgBuffer[writeIndex] = '.';
        writeIndex = (writeIndex + 1) % bufferSize;
    }

   UART_read(handle, rxBuf, 1);
}

void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle      i2c;
    I2C_Params      i2cParams;

    // open i2c
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.custom = (uintptr_t)&i2cMPUCfg;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
       System_abort("Error Initializing I2C\n");
    }

    // mpu on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU ON\n");
    System_flush();

    // wait 100ms before mpu setup
    Task_sleep(100000 / Clock_tickPeriod);

    // mpu setup
    System_printf("MPU Setup\n");
    System_flush();
    mpu9250_setup(&i2c);
    System_printf("MPU Setup OK!\n");
    System_flush();

    while (1) {

        if (programState == WAITING) {

            programState = DATA_READY;
            saveMpuData(&i2c);
        }
        if (programState == DATA_READY && stopMovement) {

            detectMoves();
        }
        else if (!stopMovement && !stopSound) {

            // notes gotten from https://gist.github.com/bboyho/0fcd284f411b1cfc7274336fde6abb45

            playNote(125, 130, 1319);
            playNote(125, 130, 1568);
            playNote(125, 130, 2637);
            playNote(125, 130, 2093);
            playNote(125, 130, 2349);
            playNote(125, 130, 3136);
            stopSound = true;
        }

        Task_sleep(10000 / Clock_tickPeriod);
    }
}

void taskFxn(UArg arg0, UArg arg1) {

  while (1) {

      if (playMusic) {

          playSeppo();
          playMusic = false;
      }
      Task_sleep(100000 / Clock_tickPeriod);
  }

}

int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle task;
    Task_Params taskParams;

    // Init board
    Board_initGeneral();
    Board_initI2C();
    Board_initUART();


    // Button pins
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }
    buttonHandle2 = PIN_open(&buttonState2, buttonConfig2);
    if(!buttonHandle2) {
       System_abort("Error initializing button pins\n");
    }


    // Set the button pin’s interrupt handler to function buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }
    if (PIN_registerIntCb(buttonHandle2, &buttonFxn2) != 0) {
       System_abort("Error registering button callback function");
    }

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    // Buzzer
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
      System_abort("Pin open failed!");
    }


    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task creation failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task creation failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    task = Task_create((Task_FuncPtr)taskFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task creation failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void playNote(uint16_t duration, uint16_t pause, uint16_t freq) {
    // Simplifies buzzer usage for more complex sounds. Takes milliseconds as input.

    buzzerOpen(hBuzzer);
    buzzerSetFrequency(freq);
    Task_sleep(duration * 1000 / Clock_tickPeriod);
    buzzerClose();
    Task_sleep(pause * 1000 / Clock_tickPeriod);
}

void detectMoves(){
    // Detects movements from global acceleration data.

    uint8_t limit = 60;  // change smaller if not detecting.
    uint8_t element;

    for (element = 0; element < 10; element++) {

        if ((abs(samples[element].gyro.gx) >= limit) && (abs(samples[element].gyro.gy) <= limit) && (abs(samples[element].gyro.gz) <= limit)) {

            if (samples[element].gyro.gx < 0) {

                strncpy(movement, ".", 2);
                programState = SENDING_MSG;
                return;
            }
            strncpy(movement, "-.", 3);
            programState = SENDING_MSG;
            return;
        }
        else if ((abs(samples[element].gyro.gx) <= limit) && (abs(samples[element].gyro.gy) >= limit) && (abs(samples[element].gyro.gz) <= limit)) {

            if (samples[element].gyro.gy < 0) {

                strncpy(movement, "..", 3);
                programState = SENDING_MSG;
                return;
            }
            strncpy(movement, "-..", 4);
            programState = SENDING_MSG;
            return;
        }
        else if ((abs(samples[element].gyro.gx) <= limit) && (abs(samples[element].gyro.gy) <= limit) && (abs(samples[element].gyro.gz) >= limit)) {

            if (samples[element].gyro.gy < 0) {

                strncpy(movement, "...", 4);
                programState = SENDING_MSG;
                return;
            }
            strncpy(movement, "-...", 5);
            programState = SENDING_MSG;
            return;
        }
        programState = WAITING;
    }
}

void sendMessage(UART_Handle *givenhandle) {
    // Sends morse code to terminal via uart. Also notifies user of usage through buzzer.

    buzzerOpen(hBuzzer);
    buzzerSetFrequency(300);
    Task_sleep(300000 / Clock_tickPeriod);
    buzzerClose();

    uint8_t element;
    for (element = 0; element < strlen(movement); element++) {

        if (movement[element] == '.') {

            UART_write(*givenhandle, ".\r\n\0", 4);
        }
        else if (movement[element] == '-') {

            UART_write(*givenhandle, "-\r\n\0", 4);
        }
        else {

            UART_write(*givenhandle, " \r\n\0", 4);
        }
    }
    UART_write(*givenhandle, " \r\n\0", 4);
}

void receiveNotification() {
    // Notifies user of incoming morse with buzzer. Utilizes ring buffer.

    if (readIndex != writeIndex) {

        if (msgBuffer[readIndex] == '.') {

            buzzerOpen(hBuzzer);
            buzzerSetFrequency(500);
            Task_sleep(100000 / Clock_tickPeriod);
            buzzerClose();
        }
        else if (msgBuffer[readIndex] == '-') {

            buzzerOpen(hBuzzer);
            buzzerSetFrequency(500);
            Task_sleep(300000 / Clock_tickPeriod);
            buzzerClose();
        }
        readIndex = (readIndex + 1) % bufferSize;
    }
}

void saveMpuData(I2C_Handle *bus) {
    // collects multiple points of data

    uint8_t index;
    for (index = 0; index < 10; index++) {
        mpu9250_get_data(
            bus,
            &samples[index].accel.ax, &samples[index].accel.ay, &samples[index].accel.az,
            &samples[index].gyro.gx,  &samples[index].gyro.gy,  &samples[index].gyro.gz);

        // saves the timestamp
        samples[index].timestamp = Clock_getTicks();

        Task_sleep(100 / Clock_tickPeriod);
    }
}

void playSeppo() {
    // Gigi D´Agostino L´Amour Toujours
    // tempo 140bpm, so wholenote = 1714290 us

    buzzerOpen(hBuzzer);
    buzzerSetFrequency(349);
    Task_sleep(428570 / Clock_tickPeriod);
    buzzerSetFrequency(349);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(587);
    Task_sleep(214290 / Clock_tickPeriod);

    buzzerSetFrequency(523);
    Task_sleep(857140 / Clock_tickPeriod);
    buzzerSetFrequency(523);
    Task_sleep(428570 / Clock_tickPeriod);
    buzzerSetFrequency(523);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(587);
    Task_sleep(214290 / Clock_tickPeriod);

    buzzerSetFrequency(494);
    Task_sleep(857140 / Clock_tickPeriod);
    buzzerSetFrequency(494);
    Task_sleep(428570 / Clock_tickPeriod);
    buzzerSetFrequency(494);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(440);
    Task_sleep(214290 / Clock_tickPeriod);

    buzzerSetFrequency(494);
    Task_sleep(428570 / Clock_tickPeriod);
    buzzerSetFrequency(494);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(440);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(494);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(440);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(494);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(440);
    Task_sleep(214290 / Clock_tickPeriod);
    buzzerSetFrequency(349);
    Task_sleep(428570 / Clock_tickPeriod);
    buzzerClose();
}
