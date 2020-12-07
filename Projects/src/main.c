/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power_manager.h"
#include "vcom.h"

#include "ev_queue.h"
#include "node_msgs.h"

// build one image with this enabled, and one without to create a ping transmitter and ping listener (which acks back to the ping transmitter)
#define SYSTEM_UNDER_TEST

// use to enable/disable hardware watchdog for the SUT
//#define HW_WATCHDOG

// use to control the high voltage psu of the SUT (NB: NOT FULLY FINISHED)
//#define CONTROL_HV_PSU

// use to enable/disable transmitter stepping between min and max power each transmit on the SUT
//#define CYCLE_OUTPUT_POWER

// use to enable/disable frequency hopping
//#define FREQUENCY_HOPPING

//*****************************SHADAB*******************************************
extern int ImPcount;
// set the loop counter used for the test
#define LOOP_COUNT 10000  //(10000 * 10 sec cycle time = 27.7h)

// set the RF freq of test (base freq if frequency hopping is enabled)
#define RF_FREQUENCY                                865300000 // Hz
// set the Spreading Factor of test
#define LORA_SPREADING_FACTOR                       8         // [SF7..SF12]
// set the cycle time, this will affect the duty cycle
#define CYCLE_TIME_MS 20000   // NOTE: MAX WATCHDOG TIME IS AROUND 28 seconds, so will need to disable watchdog if cycle time is greater

// set how long the hv psu is turned on for after a comparator event to turn it on when CONTROL_HV_PSU is enabled
#define HV_PSU_ON_PERIOD_MS 100

// set the Tx output power of test (TX_OUTPUT_POWER_START is used constantly if CYCLE_OUTPUT_POWER is disabled
#define TX_OUTPUT_POWER_START 0
#define TX_OUTPUT_POWER_MAX 15
int8_t outputPower = TX_OUTPUT_POWER_START;

// Duty Cycle Notes
// spread of 10 gives 330ms  50% duty = 800  10% duty = 4000
// spread of  9 gives 185ms  50% duty = 400  10% duty = 2000
// spread of  8 gives 102ms  50% duty = 200  10% duty = 1000
// spread of  7 gives  51ms  50% duty = 100  10% duty =  500

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx (Length in symbols (the hardware adds 4 more symbols))
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#ifdef FREQUENCY_HOPPING
#define FREQ_HOP_ON true
#define HOP_PERIOD 5
#define NUM_FREQ_HOPS 10
uint32_t frequencyHopTable[NUM_FREQ_HOPS] = { // assumes 125kHz bandwidth
		0,
		500000,
		250000,
		375000,
		125000,
		750000,
		1125000,
		625000,
		875000,
		1000000};

#else
#define FREQ_HOP_ON false
#define HOP_PERIOD 0
#endif
// hop count stats, will stay at zero if FREQUENCY_HOPPING not enabled
uint8_t numHops = 0;
uint8_t numHopsTx = 0;
uint8_t numHopsRx = 0;


EventQueue EvQueue;

double expectedTime;


int8_t rssiValue = 0;
int8_t snrValue = 0;
uint16_t rXsize = 0;

// size large enough to hold largest expected packet
#define BUFFER_SIZE 62
uint8_t buffer[BUFFER_SIZE];


uint16_t loopCount = 0;
bool quit = false;

uint32_t seqNum = 1;
uint32_t ackCountToReturn = 0;

uint32_t expectedRxSeqNum = 1;
uint32_t ackToSendBack = 1;


uint32_t rxPingErrorCount = 0;
uint32_t rxAckViaPingErrorCount = 0;

//data extracted from the ping
uint32_t receivedSeqNum;
uint8_t receivedResetReason;
uint32_t receivedBackAckCount;

// debug printf timer variables
double dbgStartPingAt;
double dbgSentPingAt;
double dbgStartAckAt;
double dbgSentAckAt;

uint8_t resetReason=0;

//*****************SHADAB****************************************
//uint8_t volatile ImPcount=0;
//********************END****************************************

/* Private function prototypes -----------------------------------------------*/


/*
 * Timer objects
 */
static TimerEvent_t nodeTimer;
// timer object used by comparator
static TimerEvent_t compTimer;

void OnNodeTimeoutEvent(void);

void OnComptriggeredEvent(void);

void OnCompTimeoutEvent(void);

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

#ifdef FREQUENCY_HOPPING
/*!
 * \brief Function executed on FhssChangeChannel event
 */
void OnFhssChangeChannel(uint8_t currentChannel);
#endif

void TimerSetValueWithComp(TimerEvent_t *obj, uint32_t value);

void sendPing(void);
void sendAck(void);

uint8_t get_system_reset_cause(void);



int main(void) {

    resetReason = get_system_reset_cause();
    HAL_Init();

    SystemClock_Config();

    DBG_Init();

    HW_Init();

    //**************************SHADAB**********************************
    IMC_GPIO_Init();
    //*************************SEND*************************************

#ifdef CONTROL_HV_PSU
    // configure and then start the comparator here for SUT
    InputVoltageCompInit(OnComptriggeredEvent);
    InputVoltageCompStart();

    // initialize a 1 second timer that will be used to end the comp trigger event
    TimerInit(&compTimer, OnCompTimeoutEvent);
    TimerSetValueFloat(&compTimer, HV_PSU_ON_PERIOD_MS);

    // and initialize the output clock signal controlling the hv psu
    highVoltagePSUInitialize();
#endif

    LED_Off(GREEN_LED);
    LED_Off(BLUE_LED);
    LED_Off(RED_LED);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
#ifdef FREQUENCY_HOPPING
    RadioEvents.FhssChangeChannel = OnFhssChangeChannel;
#endif

    Radio.Init(&RadioEvents);

#ifdef FREQUENCY_HOPPING
    Radio.SetChannel(RF_FREQUENCY + frequencyHopTable[0]);
#else
    Radio.SetChannel(RF_FREQUENCY);
#endif

    Radio.SetTxConfig(MODEM_LORA, outputPower, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    true, FREQ_HOP_ON, HOP_PERIOD, LORA_IQ_INVERSION_ON, 3000000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, FREQ_HOP_ON, HOP_PERIOD,
            LORA_IQ_INVERSION_ON, true);


/*    for(int i=0; i<100; i++) {
        PRINTF("%5d %8lu\n\r", i, Radio.TimeOnAir(MODEM_LORA,i));
    }*/

    //The low power mode library has a very odd interface.
    //In order to enable STOP mode, you have to disable OFF mode.
    LPM_SetOffMode(LPM_LIB_Id, LPM_Disable);


    // initialize event message queue
    initQueue(&EvQueue);

    // Initialise timer
    TimerInit(&nodeTimer, OnNodeTimeoutEvent);

#ifdef SYSTEM_UNDER_TEST

    PRINTF("Started Sending on %lu  with Spreading Factor %d for %lu transmissions\n\r", RF_FREQUENCY, LORA_SPREADING_FACTOR, LOOP_COUNT);
    PRINTF("Reset reason: %d\n\r",resetReason);
    PRINTF("Seq Num, Send Ping at, Sent Ping at, Ping Send Time, Output Power, Ack Num, Num Hops Tx, Num Hops Rx, Comment\n\r");
//    PRINTF("Impulse Count: %d\n\r",ImPcount);

#ifdef HW_WATCHDOG
    // enable hardware watchdog
    HW_Watchdog_Enable(CYCLE_TIME_MS+2000);
#endif

    // set a timer to start us
    expectedTime = TimerGetCurrentTimeFloat();
    TimerSetValueWithComp(&nodeTimer, CYCLE_TIME_MS);
    TimerStart(&nodeTimer);

    // enter low power mode until next EVENT_TIMER_FIRED signal
    DISABLE_IRQ();
    LPM_EnterLowPower();
    ENABLE_IRQ();

#else
    PRINTF("Started Listening on %lu with Spreading Factor %d for %lu transmissions\n\r", RF_FREQUENCY, LORA_SPREADING_FACTOR, LOOP_COUNT);
    PRINTF("Reset reason: %d\n\r",resetReason);
    PRINTF("Seq Num, Rx Ping At (ms), Ack Send Time, Rssi, Snr, Reset reason, Ack Num returned, Ping Errors, Ack Errors, Num Hops Rx, Num Hops Tx, Comment\n\r");

    // start listening
    Radio.Rx(0);
#endif

    while (loopCount < LOOP_COUNT && quit == false) {
        // pull the oldest event out of the queue to process
        Queue_Events_t nextEvent = dequeue(&EvQueue);

        // if an event exists
        if (nextEvent != EVENT_NO_EVENT) {

            switch (nextEvent) {

            case EVENT_TIMER_FIRED: {
#ifdef SYSTEM_UNDER_TEST

#ifdef CONTROL_HV_PSU
                // stop the output clock signal to enable the hv psu (keep comparator running as stopping and starting it causes triggers)
                highVoltagePSUEnable(true);
#endif

#ifdef HW_WATCHDOG
                // refresh hardware watchdog
                HW_Watchdog_Refresh();
#endif

#ifdef CYCLE_OUTPUT_POWER
                Radio.SetTxConfig(MODEM_LORA, outputPower, 0, LORA_BANDWIDTH,
                LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, LORA_IQ_INVERSION_ON, 3000000);
#endif

                // set a timer for the next ping
                TimerSetValueWithComp(&nodeTimer, CYCLE_TIME_MS);
                TimerStart(&nodeTimer);

                dbgStartPingAt = TimerGetCurrentTimeFloat();

                //******************PRINT_IMPULSE*********SHADAB******************
                PRINTF("Impulse Count: %d\n\r",ImPcount);

                sendPing();
#else
                // in listener mode the timer is used to clear the LED that might be triggered by the RX event
                LED_Off(GREEN_LED);
                LED_Off(BLUE_LED);
                #endif
                break;
            }

            case EVENT_GOT_TX_COMPLETE: {
#ifdef FREQUENCY_HOPPING
                numHopsTx = numHops;
                numHops = 0;
                // reset to be on first frequency at the end of each tx
                Radio.SetChannel(RF_FREQUENCY + frequencyHopTable[0]);
#endif

#ifdef SYSTEM_UNDER_TEST
                dbgSentPingAt = TimerGetCurrentTimeFloat();
                LED_Off(GREEN_LED);

#ifdef CYCLE_OUTPUT_POWER
                outputPower++;
                outputPower = outputPower > TX_OUTPUT_POWER_MAX ? TX_OUTPUT_POWER_START : outputPower;
#endif

                // start rx for limited time to get the ack
                Radio.Rx(300);

#else
                dbgSentAckAt = TimerGetCurrentTimeFloat();

                if(receivedSeqNum != expectedRxSeqNum ) {
                    if(expectedRxSeqNum != 0) {
                        LED_On(RED_LED); // persistent red led to show fault (sequence number out of line)
                        rxPingErrorCount++;
                    }
                    LED_On(BLUE_LED); // blue led for missed ping packet
                    PRINTF("%8d,%10.2f,%10.2f,%5d,%5d,%5d,%8d,%8d,%8d,%3d,%3d, Possible Missed Ping. Expected %d.\n\r", receivedSeqNum , TimerGetCurrentTimeFloat(), dbgSentAckAt-dbgStartAckAt, rssiValue, snrValue, receivedResetReason, receivedBackAckCount, rxPingErrorCount, rxAckViaPingErrorCount, numHopsRx, numHopsTx, expectedRxSeqNum);
                    expectedRxSeqNum  = receivedSeqNum;
                } else if(receivedBackAckCount != (uint32_t)(ackToSendBack-1)){
                    // case where seqNum correct but SUT must have missed our ack as receivedBackAckCount is not as expected
                    LED_On(RED_LED); // persistent red led to show fault (sequence number out of line)
                    LED_On(BLUE_LED); // blue led for missed ack packet
                    rxAckViaPingErrorCount++;
                    PRINTF("%8d,%10.2f,%10.2f,%5d,%5d,%5d,%8d,%8d,%8d,%3d,%3d, SUT missed our Ack. Expected %d.\n\r", receivedSeqNum , TimerGetCurrentTimeFloat(), dbgSentAckAt-dbgStartAckAt, rssiValue, snrValue, receivedResetReason, receivedBackAckCount, rxPingErrorCount, rxAckViaPingErrorCount, numHopsRx, numHopsTx, (uint32_t)(ackToSendBack-1));
                } else {
                    PRINTF("%8d,%10.2f,%10.2f,%5d,%5d,%5d,%8d,%8d,%8d,%3d,%3d,\n\r", receivedSeqNum, TimerGetCurrentTimeFloat(), dbgSentAckAt-dbgStartAckAt, rssiValue, snrValue, receivedResetReason, receivedBackAckCount, rxPingErrorCount, rxAckViaPingErrorCount, numHopsRx, numHopsTx);
                    LED_On(GREEN_LED); // green led for correct packet
                }

                // start timer to reset the leds
                TimerSetValueFloat(&nodeTimer, 1000);
                TimerStart(&nodeTimer);

                expectedRxSeqNum++;
                loopCount++;
                ackToSendBack++;

                // restart listening continuously
                Radio.Rx(0);
#endif
                break;
            }

            case EVENT_GOT_RX_DATA: {

#ifdef FREQUENCY_HOPPING
                numHopsRx = numHops;
                numHops = 0;
#endif

#ifdef SYSTEM_UNDER_TEST
                // received an ACK
                uint8_t *payload = &buffer[ACK_MSG_PAYLOAD_OFFSET_BYTES];
                // payload should be ack counter, store and send back in next ping
                ackCountToReturn = payload[0] << 24;
                ackCountToReturn += payload[1] << 16;
                ackCountToReturn += payload[2] << 8;
                ackCountToReturn += payload[3];

                // once ping has been sent, and rx received, print stats
                PRINTF("%8d,%10.2f,%10.2f,%10.2f,%2d,%8d,%3d,%3d,\n\r", seqNum, dbgStartPingAt, dbgSentPingAt, dbgSentPingAt-dbgStartPingAt, outputPower, ackCountToReturn, numHopsTx, numHopsRx);
                seqNum++;
                loopCount++;


#ifdef CONTROL_HV_PSU
                highVoltagePSUEnable(false);
#endif
                Radio.Sleep();

                // enter low power mode until next EVENT_TIMER_FIRED signal
                DISABLE_IRQ();
                LPM_EnterLowPower();
                ENABLE_IRQ();

#else
                // stop the continuous RX
                Radio.Standby();

                // extract payload at correct offset
            	uint8_t *payload = &buffer[PING_MSG_PAYLOAD_OFFSET_BYTES];

            	receivedSeqNum = payload[0] << 24;
                receivedSeqNum += payload[1]  << 16;
                receivedSeqNum += payload[2]  << 8;
                receivedSeqNum += payload[3];

                receivedResetReason = payload[4];

                receivedBackAckCount = payload[5] << 24;
                receivedBackAckCount += payload[6]  << 16;
                receivedBackAckCount += payload[7]  << 8;
                receivedBackAckCount += payload[8];

                // send ack back before printing results
                dbgStartAckAt = TimerGetCurrentTimeFloat();
                sendAck();
#endif
                break;
            }

            case EVENT_GOT_RX_ERROR: {
#ifdef SYSTEM_UNDER_TEST
                PRINTF("Rx Error\n\r");
                // fall through to timeout case
#else
                LED_On(RED_LED); // persistent red led to show fault
                PRINTF("Rx Error. Restarting Rx\n\r");
#ifdef FREQUENCY_HOPPING
                numHopsRx = numHops;
                numHops = 0;
#endif
                Radio.Rx(0);
                break;
#endif
            }

            case EVENT_GOT_RX_TIMEOUT: {
                // only in case of SUT will we get a timeout if it does not get an ack back
                // do printf as per old tx complete, but comment that no ack was recieved

#ifdef FREQUENCY_HOPPING
                numHopsRx = numHops;
                numHops = 0;
#endif
                // once ping has been sent and rx tried, print stats
                PRINTF("%8d,%10.2f,%10.2f,%10.2f,%2d,     ---,%3d,%3d, No Ack received this frame\n\r", seqNum, dbgStartPingAt, dbgSentPingAt, dbgSentPingAt-dbgStartPingAt, outputPower, numHopsTx, numHopsRx);
                seqNum++;
                loopCount++;

#ifdef CONTROL_HV_PSU
                highVoltagePSUEnable(false);
#endif
                Radio.Sleep();

                // enter low power mode until next EVENT_TIMER_FIRED signal
                DISABLE_IRQ();
                LPM_EnterLowPower();
                ENABLE_IRQ();

                break;
            }

            case EVENT_COMP_TRIGGERED: {
                // we have just switched on the hv psu due to the comparator logic, just set a timer to turn it off
                TimerStart(&compTimer);
                break;
            }

            case EVENT_COMP_TIMER_FIRED: {
                // start the output clock signal to disable the hv psu
                highVoltagePSUEnable(false);

                // enter low power mode
                DISABLE_IRQ();
                LPM_EnterLowPower();
                ENABLE_IRQ();

                break;
            }

            default: {
                PRINTF("Unexpected event: %d\n\r", nextEvent);
                quit = true;
                break;
            }
            }

        }
    }

    PRINTF("Test Ended\n\r");

    Radio.Sleep();
}


// SUT sends a ping to the receiver
void sendPing(void) {

    uint8_t payload[PING_MSG_PAYLOAD_LENGTH_BYTES];

    // create payload, just set to zero for now
    for(int i=0; i<PING_MSG_PAYLOAD_LENGTH_BYTES; i++) {
        payload[i] = 0;
    }

    payload[0] = (uint8_t)((seqNum >> 24) & 0xFF);
    payload[1] = (uint8_t)((seqNum >> 16) & 0xFF);
    payload[2] = (uint8_t)((seqNum >> 8) & 0xFF);
    payload[3] = (uint8_t)(seqNum & 0xFF);

    // add in reset reason
    payload[4] = resetReason;
    // and return the ack counter we received
    payload[5] = (uint8_t)((ackCountToReturn >> 24) & 0xFF);
    payload[6] = (uint8_t)((ackCountToReturn >> 16) & 0xFF);
    payload[7] = (uint8_t)((ackCountToReturn >> 8) & 0xFF);
    payload[8] = (uint8_t)(ackCountToReturn & 0xFF);

    // form the message into the buffer
    formPingMessageHeader(PING_MSG_HEADER_ROUTE_FORMING, 1, 0, 1, 0, 0, 0, buffer);
    // and add in payload at correct offset
    memcpy(&buffer[PING_MSG_PAYLOAD_OFFSET_BYTES], payload, 50);

    DelayMs(1);  // MAY NEED A DELAY WHEN TRANSITIONING BETWEEN RX AND TX

    LED_On(GREEN_LED);

    Radio.Send(buffer, 62);
}


void sendAck(void) {
    // receiver sends an ack back to the SUT

    uint8_t payload[ACK_MSG_PAYLOAD_LENGTH_BYTES];

    payload[0] = (uint8_t)((ackToSendBack >> 24) & 0xFF);
    payload[1] = (uint8_t)((ackToSendBack >> 16) & 0xFF);
    payload[2] = (uint8_t)((ackToSendBack >> 8) & 0xFF);
    payload[3] = (uint8_t)(ackToSendBack & 0xFF);

    formAckOrNakMessage(ACK_MSG_HEADER_REPLY_ACK, 1, 0, 1, 0, buffer);
    // and add in payload at correct offset
    memcpy(&buffer[ACK_MSG_PAYLOAD_OFFSET_BYTES], payload, 7);

    DelayMs(1);  // MAY NEED A DELAY WHEN TRANSITIONING BETWEEN RX AND TX

    Radio.Send(buffer, 18);

}

void OnTxDone(void) {
    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_GOT_TX_COMPLETE)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnTxTimeout(void) {
    Radio.Sleep();
    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_GOT_TX_TIMEOUT)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {

    // copy out the payload into our buffer, but only what we have space for
    int rxSize = size <= BUFFER_SIZE ? size : BUFFER_SIZE;
    memcpy(buffer, payload, rxSize);

    // store off the current environment properties
    rssiValue = rssi;
    snrValue = snr;
    rXsize = size;

    // and queue appropriate event
    if (!enqueue(&EvQueue, EVENT_GOT_RX_DATA)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnRxTimeout(void) {

    // queue an event
    if (!enqueue(&EvQueue, EVENT_GOT_RX_TIMEOUT)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnRxError(void) {

    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_GOT_RX_ERROR)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

#ifdef FREQUENCY_HOPPING
void OnFhssChangeChannel(uint8_t currentChannel) {

	// time to set the next frequency, look it up in the table, need to react quickly, so don't wait to
	// process this callback in the event loop, deal with it immediately
	Radio.SetChannel(RF_FREQUENCY + frequencyHopTable[(currentChannel) % NUM_FREQ_HOPS]);
	numHops++;
}
#endif

void OnNodeTimeoutEvent(void) {
    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_TIMER_FIRED)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnComptriggeredEvent(void) {
    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_COMP_TRIGGERED)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}

void OnCompTimeoutEvent(void) {
    // queue appropriate event
    if (!enqueue(&EvQueue, EVENT_COMP_TIMER_FIRED)) {
        PRINTF("ERROR - QUEUE FULL\n\r");
    }
}


void TimerSetValueWithComp(TimerEvent_t *obj, uint32_t value) {
    static double previousTimeFloat = 0;
    double currentTimeFloat = TimerGetCurrentTimeFloat();

    // TimerGetCurrentTimeFloat() will wrap at the 32bit tick value, so if rtc at 0.03 this will be after 35 hours
    // however expected time will keep increasing so will no longer be a valid comparison

    // detect wrap of ms timer
    if (currentTimeFloat <  previousTimeFloat) {

        // compensate expectedTime by removing max tick value (converted to ms) from it
        expectedTime -= HW_RTC_Tick2msFloat(0xffffffff);

        PRINTF("RTC WRAP DETECTED = ADJUST EXPECTED TIME TO %f\n\r", expectedTime);
    }

    TimerSetValueFloat(obj, value - (currentTimeFloat - expectedTime));
    expectedTime += value;

    // update history so we can detect wrapping and therefore adjust 'expectedTime'
    previousTimeFloat = currentTimeFloat;

}


uint8_t get_system_reset_cause(void)
{
    uint8_t reset_cause = 0;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        //reset_cause = "LOW_POWER_RESET";
        reset_cause = 1;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        //reset_cause = "WINDOW_WATCHDOG_RESET";
        reset_cause = 2;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        //reset_cause = "INDEPENDENT_WATCHDOG_RESET";
        reset_cause = 3;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        //reset_cause = "SOFTWARE_RESET"; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
        reset_cause = 4;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        //reset_cause = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
        reset_cause = 5;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        //reset_cause = "EXTERNAL_RESET_PIN_RESET";
        reset_cause = 6;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is
    // NOT a POR/PDR reset. See note below.
/*    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
    {
// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock Controller (RCC) header
// files, such as "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h", etc., indicate that the
// brownout flag, `RCC_FLAG_BORRST`, will be set in the event of a "POR/PDR or BOR reset". This means that a
// Power-On Reset (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag. See the
// doxygen just above their definition for the `__HAL_RCC_GET_FLAG()` macro to see this:
// "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout Reset flag will *also* be set in
// the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after* first checking the
// `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is NOT a POR/PDR reset.

        //reset_cause = "BROWNOUT_RESET (BOR)";
        reset_cause = 7;
    }*/
    else
    {
        //reset_cause = "UNKNOWN";
        reset_cause = 8;
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

