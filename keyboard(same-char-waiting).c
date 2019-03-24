/*
 * File:   Keyboard.c
 * Copyright 2013 Tool Labs
 */

#ifndef KEYBOARD_C
#define KEYBOARD_C

//*********************
// Initialization Part
//*********************

// Include files
#include <xc.h>
#include "HardwareProfile.h"
#include "./USB/usb.h"
#include "./USB/usb_function_hid.h"
#include "Keyboard.h"

// Set configuration bits
// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection Bits (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 4x     // PLL Multipler Selection Bit (4x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// Prepare IN/OUT buffer for USB end point
#define IN_DATA_BUFFER_ADDRESS 0x2050
#define OUT_DATA_BUFFER_ADDRESS (IN_DATA_BUFFER_ADDRESS + HID_INT_IN_EP_SIZE)
#define IN_DATA_BUFFER_ADDRESS_TAG @IN_DATA_BUFFER_ADDRESS
#define OUT_DATA_BUFFER_ADDRESS_TAG @OUT_DATA_BUFFER_ADDRESS

// Allocate report in and out buffer
unsigned char   hid_report_in[HID_INT_IN_EP_SIZE] IN_DATA_BUFFER_ADDRESS_TAG;
unsigned char   hid_report_out[HID_INT_OUT_EP_SIZE] OUT_DATA_BUFFER_ADDRESS_TAG;

// Multiple Character
unsigned char   keyCode1[] = {0x09, 0x09, 0x12, 0x12, 0x17, 0x2c, 0x0e, 0x08, 0x1c, 0x05, 0x12, 0x04, 0x15, 0x07}; // Set "foot keyboard"
unsigned char   numberOfChar1 = sizeof(keyCode1);
unsigned char   remainedSendChar;
unsigned short  waitTimeForSameKey;
BOOL            timeWaited;

// Variables to store each switch status
BYTE            old_sw1, old_sw2,old_sw3, old_sw4, old_sw5, old_sw6, old_sw7;

// Handles for usb data transmission status
USB_HANDLE      lastINTransmission;
USB_HANDLE      lastOUTTransmission;


// USB device controll prototype definitions
static void InitializeSystem(void);
void ProcessIO(void);
void Keyboard(void);
void USBCBSendResume(void);
void USBHIDCBSetReportComplete(void);

// Switch status confirmation prototype definitions
BOOL Switch1IsPressed(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
BOOL Switch4IsPressed(void);
BOOL Switch5IsPressed(void);
BOOL Switch6IsPressed(void);
BOOL Switch7IsPressed(void);

// USB callback prototype definitions
void USBCBSuspend(void);
void USBCBWakeFromSuspend(void);
void USBCB_SOF_Handler(void);
void USBCBErrorHandler(void);
void USBCBCheckOtherReq(void);
void USBCBStdSetDscHandler(void);
void USBCBInitEP(void);
void USBCBSendResume(void);



//*********************
//     Coding Part
//*********************
//
// main function
//
int main(void)
{
    // First initialize the system
    InitializeSystem();

    while(1)
    {
        // Call USBDeviceTasks() to respond qurery from host and so on
        USBDeviceTasks();
    				  
        // Call ProcessIO() to process keyboard behavior
        ProcessIO();        
    }
}



//
// InitializeSystem function
//
static void InitializeSystem(void)
{
    // Set all pins to digital mode
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    // Set switch connected pins as input pins and others as output pins
    TRISA  = 0x00;
    TRISB  = 0xC0;
    TRISC  = 0xF0;

    // Set oscillator parameters
    OSCTUNE = 0;
    OSCCON = 0x3C;              // PLL enabled, 3x clock, 16MHz internal oscillator, SCS external
    OSCCONbits.SPLLMULT = 0;    // 4x
    ACTCON = 0x00;              // Clock recovery off, Clock Recovery enabled; SOF packet

    // Turn off the LED
    mLED_1_Off()

    //Initialize all of the push buttons
    old_sw1 = switch1;
    old_sw2 = switch2;
    old_sw3 = switch3;
    old_sw4 = switch4;
    old_sw5 = switch5;
    old_sw6 = switch6;
    old_sw7 = switch7;

    // Initialize the variable holding the handle for the last transmission
    lastINTransmission  = 0;
    lastOUTTransmission = 0;

    // Initialize USB module
    USBDeviceInit();

    // Initialize multiple characters
    remainedSendChar = 0;

    // Initialize wait time and flag for same key repeating
    waitTimeForSameKey = 0;
    timeWaited = FALSE;

    // Turn on the LED
    mLED_1_On()
}


//
// ProcessIO function
//
void ProcessIO(void)
{
    // Call USBCBSendResume when switch5 is pressed
    // Ignore other keys
    if(switch5 == 0)
    {
        USBCBSendResume();
    }

    // Perform keyboard tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)){
        return;
    }

    Keyboard();
}


//
// Keyboard function
//
void Keyboard(void)
{
    // Location for multiple characters
    unsigned char charLocation;

    // If USB transmitting handle is not busy, process the keyboard action
    if(!HIDTxHandleBusy(lastINTransmission))
    {
        // If character remained, send the character to the host without checking switch status
        if(remainedSendChar)
        {
            if(waitTimeForSameKey > 0){
                waitTimeForSameKey--;
            } else {
                // Set the current sending character location of keyCode[]
                charLocation = numberOfChar1 - remainedSendChar;
                // Compare the current character and the previous character
                if(keyCode1[charLocation] != keyCode1[charLocation-1]){
                    // If the current and the previous character is different, send remained chatacter immediately
                    hid_report_in[0] = 0;
                    hid_report_in[1] = 0;
                    hid_report_in[2] = keyCode1[charLocation];
                    hid_report_in[3] = 0;
                    hid_report_in[4] = 0;
                    hid_report_in[5] = 0;
                    hid_report_in[6] = 0;
                    hid_report_in[7] = 0;
                    lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                    remainedSendChar--;
                } else {
                    if(timeWaited) {
                        // If the current and the previous character is different, send remained chatacter immediately
                        hid_report_in[0] = 0;
                        hid_report_in[1] = 0;
                        hid_report_in[2] = keyCode1[charLocation];
                        hid_report_in[3] = 0;
                        hid_report_in[4] = 0;
                        hid_report_in[5] = 0;
                        hid_report_in[6] = 0;
                        hid_report_in[7] = 0;
                        lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                        remainedSendChar--;
                        timeWaited = FALSE;
                    } else {
                        // If the current and the previous character is same, wait for a while
                        waitTimeForSameKey = 60000;
                        timeWaited = TRUE;
                    }
                }
            }
        } else {
            // If no character remained, check switch status and send corresponding charater to the host
            if(Switch1IsPressed())
            {
                // command + w-key
                //hid_report_in[0] = 0x08;
                //hid_report_in[1] = 0;
                //hid_report_in[2] = 0x1a;
                //hid_report_in[3] = 0;
                //hid_report_in[4] = 0;
                //hid_report_in[5] = 0;
                //hid_report_in[6] = 0;
                //hid_report_in[7] = 0;
                //lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);

                // Start sending multiple charaters. Send the first character
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = keyCode1[0];
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
                // and set number of remained characters
                remainedSendChar = numberOfChar1 - 1;
            }
            else if(Switch2IsPressed())
            {
                // space-bar
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x2c;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
            else if(Switch3IsPressed())
            {
                // shift + space-bar
                hid_report_in[0] = 0x02;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x2c;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
            else if(Switch4IsPressed())
            {
                // v-key
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x19;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
            else if(Switch5IsPressed())
            {
                // w-key
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x16;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
            else if(Switch6IsPressed())
            {
                // j-key
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x0d;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
            else if(Switch7IsPressed())
            {
                // k-key
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0x0e;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            } else
            {
                // No keyboard action
                hid_report_in[0] = 0;
                hid_report_in[1] = 0;
                hid_report_in[2] = 0;
                hid_report_in[3] = 0;
                hid_report_in[4] = 0;
                hid_report_in[5] = 0;
                hid_report_in[6] = 0;
                hid_report_in[7] = 0;
                lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
            }
        }
    }

    // If USB receiving handle is not busy, process the data reading action
    if(!HIDRxHandleBusy(lastOUTTransmission))
    {
        if(hid_report_out[0] == 0x01)
        {
            mLED_1_Toggle()
        }
        lastOUTTransmission = HIDRxPacket(HID_EP,(BYTE*)&hid_report_out,HID_INT_OUT_EP_SIZE);
    }
    
    return;		
}


//
// Switch status check function for switch1 through switch5
//
BOOL Switch1IsPressed(void)
{
    if(switch1 != old_sw1)
    {
        old_sw1 = switch1;
        if(switch1 == 0)
            return TRUE;
    }
    return FALSE;
}


BOOL Switch2IsPressed(void)
{
    if(switch2 != old_sw2)
    {
        old_sw2 = switch2;
        if(switch2 == 0)
            return TRUE;
    }
    return FALSE;
}

BOOL Switch3IsPressed(void)
{
    if(switch3 != old_sw3)
    {
        old_sw3 = switch3;
        if(switch3 == 0)
            return TRUE;
    }
    return FALSE;
}

BOOL Switch4IsPressed(void)
{
    if(switch4 != old_sw4)
    {
        old_sw4 = switch4;
        if(switch4 == 0)
            return TRUE;
    }
    return FALSE;
}

BOOL Switch5IsPressed(void)
{
    if(switch5 != old_sw5)
    {
        old_sw5 = switch5;
        if(switch5 == 0)
            return TRUE;
    }
    return FALSE;
}

BOOL Switch6IsPressed(void)
{
    if(switch6 != old_sw6)
    {
        old_sw6 = switch6;
        if(switch6 == 0)
            return TRUE;
    }
    return FALSE;
}

BOOL Switch7IsPressed(void)
{
    if(switch7 != old_sw7)
    {
        old_sw7 = switch7;
        if(switch7 == 0)
            return TRUE;
    }
    return FALSE;
}

//
// USB Callback handler
//
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch( event )
    {
        case EVENT_TRANSFER:
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}


//
// USB Callback functions
//
void USBCBSuspend(void)
{
    // Do nothing
}

void USBCBWakeFromSuspend(void)
{
    // Do nothing
}

void USBCB_SOF_Handler(void)
{
    // Do nothing
}

void USBCBErrorHandler(void)
{
    // Do nothing
}

void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}

void USBCBStdSetDscHandler(void)
{
    // Do nothing
}

void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Arm OUT endpoint so we can receive caps lock, num lock, etc. info from host
    lastOUTTransmission = HIDRxPacket(HID_EP,(BYTE*)&hid_report_out,HID_INT_OUT_EP_SIZE);
}

void USBCBSendResume(void)
{
    static WORD delay_count;

    if(USBGetRemoteWakeupStatus() == TRUE)
    {
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;
            delay_count = 3600U;        

            do
            {
                delay_count--;
            }while(delay_count);
            
            USBResumeControl = 1;
            delay_count = 1800U;

            do
            {
                delay_count--;
            }while(delay_count);

            USBResumeControl = 0;
            USBUnmaskInterrupts();
        }
    }
}


//
// USBHIDCBSetReportHandler
//
void USBHIDCBSetReportHandler(void)
{
    USBEP0Receive((BYTE*)&CtrlTrfData, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);
}

void USBHIDCBSetReportComplete(void)
{
	if(CtrlTrfData[0] == 0x01)
	{
            mLED_1_Toggle()
	}
}	

#endif
