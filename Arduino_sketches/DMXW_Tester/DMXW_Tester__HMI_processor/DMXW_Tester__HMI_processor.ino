/******************************************************************************
 * DMXW Tester (Human-Machine Interface processor)
 * 
 * This sketch implements the user interface porition of the DMXW wireless
 * remote slave tester. It takes inpus from the onboard pushbuttons and
 * potentiometer, provides user feedback via the OLED display, and sends
 * test data to the DMXW Tester output processor. The sketch is taregetted to
 * the Spark MicroView Aruino-compatible board that contains an onboard OLED
 * display.
 * 
 * The Output Processor is responsible for controlling the outputs to the
 * onboard test channels and for sending and receiving DMXW network messages
 * to and from DMXW slave nodes. The Output Processor and Human-Machine
 * Interface Processor communicate via RS-232 based messages.
 *
 * Notes:
 * =====
 *   Special EEPROM memory locations:
 *      Address   Description
 *      -------   ----------------------------------
 *         0      Firmware Version
 *
 * Console Controls:
 * ================
 *   Control              Pin
 *   ----------------     ---
 *   Potentiometer        A5
 *   Pushbuttons
 *     "+"                D3
 *     "-"                D4
 *     "Select"           
 *
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 1.0      20-04-25 J.van Schouwen Initial creation.
 *
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <MicroView.h>
#include <SoftwareSerial.h>
#include "TesterDefs.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>


/*******************************  Constants  *********************************/
#define COPYRIGHT       "(C)2020, A.J. van Schouwen"
#define SW_VERSION_c    "1.0 (2020-04-25)"
#define FW_VERSION_c    1   // Increment (with wraparound) for new F/W;
                            //   clears EEPROM.

#define DEBUG_ON        // Uncomment to turn off debug output to serial port.
#define LOGGING_ON      // Uncomment to turn off packet logging to serial port.

#define EEPROM_FW_ADDR             0
#define EEPROM_DATA_SAVED          1 // 1 = valid data is in EEPROM
#define EEPROM_1ST_OPEN_ADDR      10 // First unreserved EEPROM address

#define SERIAL_BAUD          115200  // For serial debug console
#define IPC_COMMS_BAUD        19200  // For serial IPC with Output processor.

#define HMI_RETRY_PERIOD_MS      50  // Interval between Cmd Tx retries (ms)
#define HMI_MAX_RETRIES           5  // Max # of Tx retries


/*** Digital input pins ***/
// Console buttons
#define PIN_BUTTON_PLUS           3  // "+" pushbutton
#define PIN_BUTTON_MINUS          5  // "-" pushbutton
#define PIN_BUTTON_SELECT         6  // "Select" pushutton
#define NUM_BUTTONS               3

/*** Analog input pins ***/
#define POT_PIN                  A5  // Onboard potentiometer

/*** IPC serial port pins (for SoftwareSerial) ***/
#define HMI_COMMS_RX             A0  // Serial RX port for HMI processor comms
#define HMI_COMMS_TX             A1  // Serial TX port for HMI processor comms


// Macro for defining strings that are stored in flash (program) memory rather
// than in RAM. Arduino defines the non-descript F("string") syntax.
#define FLASH(x) F(x)

#ifdef DEBUG_ON
  #define dbgPrint(x)    Serial.print(x)
  #define dbgPrintln(x)  Serial.println(x)
  #define DEBUGGING  1
#else
  #define dbgPrint(x)
  #define dbgPrintln(x)
  #define DEBUGGING  0
#endif

#ifdef LOGGING_ON
  #define logPrint(x)    Serial.print(x)
  #define logPrintln(x)  Serial.println(x)
#else
  #define logPrint(x)
  #define logPrintln(x)
#endif

/* Button Type Definitions */
#define KEY_NONE        0   // No button is being pressed
#define KEY_SELECT      1   // The "Select" button is being pressed
#define KEY_PLUS        2   // The "+" button is being pressed
#define KEY_MINUS       3   // The "-" button is being pressed

/* Menu Definitions */
#define MAX_SLIDERS     6   // Max # slider widgets that can fit on the
                            // (Arduino compatible) MicroView's OLED display.

#define POT_UPDATE_MS 200   // Update interval (ms) for the channel associated
                            //  with the potentiometer.


/****************************  Type Definitions  ******************************/

typedef struct ButtonStruct
{
  uint8 keyId;
  uint8 digitalPin;
  uint8 state;
} Button_t;



/*****************************  State Variables  ******************************/

// Interprocessor Communications serial port.
SoftwareSerial s_OutComms(HMI_COMMS_RX, HMI_COMMS_TX);
uint8 s_IpcInBuf[MAX_SERIAL_BUF_LEN];
uint8 s_IpcInBufPos = 0;
uint8 s_IpcOutBuf[MAX_SERIAL_BUF_LEN];
uint8 s_IpcOutBufPos = 0;
uint8 s_IpcSeqNum = 0;  // Sequence # for Tx IPC messages.

/** Pushbutton states */
Button_t buttons[NUM_BUTTONS] =
{
  /* keyId          digitalPin      state */
  { KEY_SELECT,  PIN_BUTTON_SELECT,  LOW },
  { KEY_PLUS,    PIN_BUTTON_PLUS,    LOW },
  { KEY_MINUS,   PIN_BUTTON_MINUS,   LOW }
};

/** Channel Selection
 *  For i > 0:
 *    s_DmxwChanSelect[i] = true iff DMXW channel #i is selected for testing.
 *    s_LocalChanSelect[i] = true iff onboard channel #i is selected for
 *                           testing.
 *  The value of s_DmxwChanSelect[0] and s_LocalChanSelect[0] indicates to
 *  which channel the potentiometer is assigned for real-time value control.
 *  If the indicated value is zero, the potentiometer isn't assigned to any
 *  channel.
 */
uint8 s_DmxwChanSelect[NUM_CHAN_DMXW + 1];
uint8 s_LocalChanSelect[NUM_CHANS_ONBOARD + 1];

/** Specifed Channel Values
 *  s_DmxwChanValues[i] = the value assigned to DMXW channel #i.
 *  s_LocalChanValues[i] = the value assigned to onboard channel #i.
 */
uint8 s_DmxwChanValues[NUM_CHAN_DMXW + 1];
uint8 s_LocalChanValues[NUM_CHANS_ONBOARD + 1];

/** LED Pixel String configuration */
uint16 s_PixelStringLen = 0;

TestOutput_t s_TestOutputs  = NO_OUTPUT; // Current test output
TestType_t   s_TestType     = DISABLED;   // Current test type
TestState_t  s_TestState    = STOPPED;    // Current test state
TestState_t  s_TestStateNew = STOPPED;

/** Graphical channel value slider widgets */
MicroViewWidget *valueWidget[MAX_SLIDERS];

bool    s_ResetEeprom     = false;
bool    s_SendConfigData  = false;
bool    s_OverridePot     = false;
bool    s_OutputRebooted  = true;
bool    s_Initialized     = false;

int8 temporary;



//=========================================================================

/******************************************************************************
 * Function: EepromLoad
 * 
 * Load saved test parameter values from EEPROM.
 * 
 * Parameters: (none)
 * Returns:  true iff there was configuration data read from EEPROM
 ******************************************************************************/
int8 EepromLoad()
{
  int addr = EEPROM_1ST_OPEN_ADDR;
  boolean dataValid;

  dataValid = EEPROM.read(EEPROM_DATA_SAVED);
  if (!dataValid)
  {
    return false;
  }
  
  s_TestOutputs = (TestOutput_t)EEPROM.read(addr++);
  s_TestType = (TestType_t)EEPROM.read(addr++);
  s_PixelStringLen  = (uint16)EEPROM.read(addr++) << 8;
  s_PixelStringLen |= EEPROM.read(addr++);
  for (uint8 i = 0; i <= NUM_CHAN_DMXW; i++)
  {
    s_DmxwChanSelect[i] = EEPROM.read(addr++);
    s_DmxwChanValues[i] = EEPROM.read(addr++);
  }
  for (uint8 i = 0; i <= NUM_CHANS_ONBOARD; i++)
  {
    s_LocalChanSelect[i] = EEPROM.read(addr++);
    s_LocalChanValues[i] = EEPROM.read(addr++);
  }
  return true;
}

/******************************************************************************
 * Function: EepromSave
 * 
 * Save test parameter values to EEPROM.
 * 
 * Parameters: (none)
 * Returns:    (none)
 ******************************************************************************/
void EepromSave()
{
  int addr = EEPROM_1ST_OPEN_ADDR;

  EEPROM.write(addr++, s_TestOutputs);
  EEPROM.write(addr++, s_TestType);
  EEPROM.write(addr++, (uint8)(s_PixelStringLen >> 8));
  EEPROM.write(addr++, (uint8)(s_PixelStringLen & 0x0F));
  for (uint8 i = 0; i <= NUM_CHAN_DMXW; i++)
  {
    EEPROM.write(addr++, s_DmxwChanSelect[i]);
    EEPROM.write(addr++, s_DmxwChanValues[i]);
  }
  for (uint8 i = 0; i <= NUM_CHANS_ONBOARD; i++)
  {
    EEPROM.write(addr++, s_LocalChanSelect[i]);
    EEPROM.write(addr++, s_LocalChanValues[i]);
  }

  EEPROM.write(EEPROM_DATA_SAVED, true);
}


/******************************************************************************
 * Function: Crc8
 * 
 * Compute the 8-bit Circular Redundancy Check value for an octet sequence
 * of specified length.
 * 
 * Parameters:
 *   pData:I  - Base address of the octet sequence.
 *   len:I    - Length (in octets) of the octet sequence.
 * Returns:  The computed CRC8 value.
 ******************************************************************************/
uint8 Crc8(const uint8 *pData, uint8 len)
{
  uint8 crc = 0x00;
  uint8 extract;
  uint8 sum;
  uint8 bitCount;

  while (0 != len--)
  {
    extract = *pData++;
    for (bitCount = 8; bitCount != 0; bitCount--)
    {
      sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (0 != sum)
      {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}


/***************************************************************************
 * Function: ScanButton
 * 
 * Determine which button is currently being pressed.
 * 
 * Arguments: (none)
 * Returns:
 *   KEY_NONE,   if no button is being pressed
 *   KEY_SELECT, if the 'Select' button is being pressed
 *   KEY_PLUS,   if the '+' button is being pressed
 *   KEY_MINUS,  if the '-' button is being pressed
 * Note:
 *   Buttons are connected to pins that are configured with internal pull-up
 *   resistors and each button connected between pin and and ground. Thus,
 *   the logic is reversed--when a button is pressed the input value is LOW.
 ***************************************************************************/
uint8 ScanButton(void)
{
  uint8 pressedButton = NUM_BUTTONS;
  uint8 numPressed = 0;
  uint8 keyCode = KEY_NONE;
  
  /* Take an initial reading off all buttons (for debouncing) */
  for (uint8 i = 0; i < NUM_BUTTONS; i++)
  {
    buttons[i].state = !digitalRead(buttons[i].digitalPin);
  }
  
  /* Take second readings and determine if single button is pressed */
  for (uint8 i = 0; i < NUM_BUTTONS; i++)
  {
    if (buttons[i].state == !digitalRead(buttons[i].digitalPin))
    {
      if (buttons[i].state == HIGH)
      {
        numPressed++;
        pressedButton = i;
      }
    }
    else
    {
      buttons[i].state = LOW;
    }
  }
  
  /* Only report a valid button is exactly one button is pressed */
  if (numPressed == 1)
  {
    keyCode = buttons[pressedButton].keyId;
  }
  return keyCode;
}


/***************************************************************************
 * Function: ProcessIpcAck
 * 
 * Parse and process a received acknowledgement in the IPC serial buffer.
 * If the output processor rebooted, flag that fact.
 * 
 * Parameters:
 *   expectedCmdAck:I  - The command to which an ACK is expected.
 * Returns:
 *   IPC Ack code (e.g. TSTACK_OK), if an ACK was received;
 *   -1, otherwise.
 * Input/Output:
 *   s_IpcInBuf:IO
 *   s_IpcInBufPos:IO
 *   s_OutputRebooted:O
 ***************************************************************************/
int8 ProcessIpcAck(uint8 expectedCmdAck)
{
  uint8 idx;
  uint8 cmd;
  int8  rc;
  uint8 isErr;
  
  if (0 == s_IpcInBufPos)
  {
    // There's nothing to process.
    dbgPrintln(FLASH("***ProcessIpcAck() called with empty buffer"));
    return -1;
  }

  idx = 0;
  isErr = false;
  switch (s_IpcInBuf[idx++])
  {
    case TSTCMD_ACK:
      // TSTCMD_ACK(cmdCode, returnCode)
      dbgPrint(millis());
      dbgPrint(FLASH(" ACK: "));
      if (s_IpcInBufPos != 3)
      {
        isErr = true;
        dbgPrintln(FLASH("Corrupted ACK"));
      }
      else
      {
        cmd = s_IpcInBuf[idx++];
        rc = s_IpcInBuf[idx++];
        if (cmd != expectedCmdAck)
        {
          isErr = true;
        }
        dbgPrint(cmd);
        dbgPrint(", ");
        dbgPrintln(rc);
      }
      break;

    default:
      isErr = true;
      dbgPrint(FLASH("***Unexpected IPC message ["));
      dbgPrint(s_IpcInBuf[0]);
      dbgPrintln(FLASH("]"));
  }

  if (isErr)
  {
    return -1;
  }
  else
  {
    if (TSTACK_OUTREBOOT == rc)
    {
      s_OutputRebooted = true;
    }
    return rc;
  }
}


/***************************************************************************
 * Function: HandleIpcRx
 * 
 * Check for, and respond to, messages coming in from the HMI processor
 * over the Interprocessor Communcations serial port.
 * 
 * Parameters:
 *   expectedCmdAck:I  - The command to which an ACK is expected.
 * Returns:
 *   IPC Ack code (e.g. TSTACK_OK), if an ACK was received;
 *   -1, otherwise. 
 * Input/Output:
 *   s_IpcSeqNum:I
 *   s_IpcInBuf:IO
 *   s_IpcInBufPos:IO
 * Note:
 *   This function processes the serial input in stream fashion, processing
 *   as much of the input at-a-time as it can, executing at most one
 *   received command.
 ***************************************************************************/
int8 HandleIpcRx(uint8 expectedCmdAck)
{
  static uint8 state = IPC_SEEK;
  boolean done;
  uint8   inChar;
  uint8   charCount;
  uint8   rxCrc8;
  uint8   calcCrc8;
  uint8   ackSeqNum;

  done = false;
  charCount = s_OutComms.available();

  /* If the serial buffer has overflowed, purge the buffer
   * and abort further processing.
   */
  if (s_OutComms.overflow())
  {
    dbgPrintln(FLASH("***Buffer overflow"));
    for (uint8 i = charCount; i != 0; i++)
    {
      inChar = s_OutComms.read();
    }
    return -1;
  }
  
  while ((charCount != 0) && !done)
  {
    charCount--;
    inChar  = s_OutComms.read();
    switch (state)
    {
      case IPC_SEEK:
        s_IpcInBufPos = 0;
        if (CHAR_STX == inChar)
        {
          state = IPC_SEEK2;
        }
        break;

      case IPC_SEEK2:
        if (CHAR_STX == inChar)
        {
          state = IPC_SEQNUM;
        }
        else
        {
          state = IPC_PURGE;
        }
        break;

      case IPC_SEQNUM:
        ackSeqNum = inChar;
        if (ackSeqNum < s_IpcSeqNum)
        {
          // This is an old ACK. Discard it.
          state = IPC_PURGE;
          dbgPrint(FLASH("Drop: "));
          dbgPrintln(ackSeqNum);
        }
        else
        {
          state = IPC_COLLECT;
        }
        break;

      case IPC_COLLECT:
        if (CHAR_ETX == inChar)
        {
          // Looks like our command is finishing
          state = IPC_CRC;
        }
        else if (CHAR_STX == inChar)
        {
          // We've encountered a non-escape CHAR_STX.
          state = IPC_SEEK2;
        }
        else
        {
          // Keep adding characters
          if (s_IpcInBufPos >= sizeof(s_IpcInBuf))
          {
            // No matter what, we're going to overflow the buffer
            state = IPC_PURGE;
          }
          else
          {
            // The buffer has room for at least one more byte of data.
            if (CHAR_ESC == inChar)
            {
              state = IPC_ESC;
            }
            else
            {
              s_IpcInBuf[s_IpcInBufPos++] = inChar;
            }
          }
        }
        break;

      case IPC_ESC:
        s_IpcInBuf[s_IpcInBufPos++] = inChar;
        state = IPC_COLLECT;
        break;

      case IPC_CRC:
        rxCrc8 = inChar;
        state = IPC_SEEK;
        done = true;
        break;
        
      case IPC_PURGE:
      default:
        /* Purge the buffer until empty or the CHAR_STX character
         * has been found.
         */
        if (CHAR_STX == inChar)
        {
          state = IPC_SEEK2;
        }
    }
  }

  if (done)
  {
    // We have an ACK to be processed
    calcCrc8 = Crc8(s_IpcInBuf, s_IpcInBufPos);
    dbgPrint(FLASH("CRC: Rx="));
    dbgPrint(rxCrc8);
    dbgPrint(FLASH(", Calc="));
    dbgPrintln(calcCrc8);
    if (calcCrc8 == rxCrc8)
    {
      // The CRC8 value check out: Process the ACK.
      return ProcessIpcAck(expectedCmdAck);
    }
    dbgPrintln(FLASH("***Bad ACK CRC8"));
  }
  
  return -1;
}


/***************************************************************************
 * Function: SendIpcCommand
 * 
 * Send a command to the Output Processor via the IPC serial ports.
 * 
 * Parameters:
 *  argBuf:I  - Address of a buffer containing the command arguments.
 *  bufLen:I  - Number of bytes in the buffer.
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error (n is the return code);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_IpcSeqNum:IO
 ***************************************************************************/
int8 SendIpcCommand(uint8 *argBuf, uint8 bufLen)
{
  uint32  retryTime;
  uint8  *src;
  uint8   retryCount;
  uint8   crc8;
  uint8   outChar;
  int8    rc;
  uint8   i;
  boolean acked;

  crc8 = Crc8(argBuf, bufLen);
  retryCount = 0;
  acked = false;
  while ( (retryCount < HMI_MAX_RETRIES) && !acked )
  {
    retryCount++;
    dbgPrint(FLASH("SendIpcCommand. Out: "));
    /***  Send the command  ***/
    
    src = argBuf;
    s_OutComms.write(CHAR_STX);
    dbgPrint(CHAR_STX);
    dbgPrint(" ");
    s_OutComms.write(CHAR_STX);
    dbgPrint(CHAR_STX);
    dbgPrint(" ");
    s_IpcSeqNum++;
    if (s_IpcSeqNum >= CHAR_ESC)
    {
      s_IpcSeqNum = 0;
    }
    s_OutComms.write(s_IpcSeqNum);
    dbgPrint(s_IpcSeqNum);
    dbgPrint(" ");

    /* Transmit the command buffer contents */
    for (i = bufLen; i != 0; i--)
    {
      outChar = *src;
      src++;
      if (outChar >= CHAR_ESC)
      {
        // outChar needs to be escaped
        s_OutComms.write(CHAR_ESC);
        dbgPrint(CHAR_ESC);
        dbgPrint(" ");
      }
      s_OutComms.write((uint8)outChar);
      dbgPrint(outChar);
      dbgPrint(" ");
    }
  
    s_OutComms.write(CHAR_ETX);
    dbgPrint(CHAR_ETX);
    dbgPrint(" ");
    s_OutComms.write(crc8);
    dbgPrintln(crc8);


    /***  Wait for ACK  ***/
    dbgPrint(millis());
    dbgPrint(FLASH(" Try #: "));
    dbgPrintln(retryCount);
    retryTime = millis() + HMI_RETRY_PERIOD_MS;
    while ( (millis() < retryTime) && !acked )
    {
      rc = HandleIpcRx(argBuf[0]);
      if (rc != -1)
      {
        acked = true;
      }
    }
  }

  if (acked)
  {
    return rc;
  }
  else
  {
    return -1;
  }
}


/***************************************************************************
 * Function: IpcSetState
 * 
 * Send an IPC command to the Output Processor to set the testing state.
 * 
 * Parameters:
 *  newState:I  - New testing state.
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 ***************************************************************************/
int8 IpcSetState(TestState_t newState)
{
  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_STATE;
  s_IpcOutBuf[s_IpcOutBufPos++] = newState;
  return SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
}


/***************************************************************************
 * Function: IpcConfigureTest
 * 
 * Send an IPC command to the Output Processor to configure the test
 * type and test outputs.
 * 
 * Parameters: (none)
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 *   s_TestOutputs:I
 *   s_TestType:I
 ***************************************************************************/
int8 IpcConfigureTest(void)
{
  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_TEST;
  s_IpcOutBuf[s_IpcOutBufPos++] = s_TestOutputs;
  s_IpcOutBuf[s_IpcOutBufPos++] = s_TestType;
  return SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
}


/***************************************************************************
 * Function: IpcSelectChannels
 * 
 * Send an IPC command to the Output Processor to select which channels
 * are to be active in a test.
 * 
 * Parameters: (none)
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_DmxwChanSelect:I
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 *   s_LocalChanSelect:I
 *   s_TestOutputs:I
 * Note:
 *   The selected channels are: 
 *   - Those defined in s_DmxwChanSelect[], if the current test outputs
 *     are set to DMXW;
 *   - Those defined in sLocalChanSelect[], if the current test outputs
 *     are set to ONBOARD; or
 *   - All deselected, if the current test outputs are set to NO_OUTPUT.
 ***************************************************************************/
int8 IpcSelectChannels(void)
{
  uint8   *pChanSelect;
  uint8    remainingChans;
  uint8    chanNum;
  uint8    tmpPos;
  uint8    i;
  int8     rc;

  switch (s_TestOutputs)
  {
    case NO_OUTPUT:
      pChanSelect = NULL;
      remainingChans = 0;
      break;

    case DMXW:
      pChanSelect = &s_DmxwChanSelect[1];
      remainingChans = NUM_CHAN_DMXW;
      break;

    case ONBOARD:
      pChanSelect = &s_LocalChanSelect[1];
      remainingChans = NUM_CHANS_ONBOARD;
      break;

    default:
      dbgPrintln(FLASH("***IpcConfigureTest: Invalid test outputs"));
      return -1;
  }

  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_SELECT;
  s_IpcOutBuf[s_IpcOutBufPos++] = SELECT_CLR;
  s_IpcOutBuf[s_IpcOutBufPos++] = 0;
  rc = SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
  if (NO_OUTPUT == s_TestOutputs)
  {
    return rc;
  }

  chanNum = 1;
  while (remainingChans != 0)
  {
    s_IpcOutBufPos = 0;
    s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_SELECT;
    s_IpcOutBuf[s_IpcOutBufPos++] = SELECT_ADD;
    tmpPos = s_IpcOutBufPos++; // Skip the numChans argument for now
    while ( (remainingChans != 0) && (s_IpcOutBufPos < sizeof(s_IpcOutBuf)) )
    {
      if (*pChanSelect)
      {
        s_IpcOutBuf[s_IpcOutBufPos++] = chanNum;
      }
      pChanSelect++;
      chanNum++;
      remainingChans--;
    }
    s_IpcOutBuf[tmpPos] = s_IpcOutBufPos - tmpPos - 1;
    rc = SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
    if (rc != TSTACK_OK)
    {
      return rc;
    }
  }

  return rc;
}


/***************************************************************************
 * Function: IpcSetChannels
 * 
 * Send an IPC command to the Output Processor to set specific channel
 * values.
 * 
 * Parameters: (none)
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_DmxwChanSelect:I
 *   s_DmxwChanValues:I
 *   s_LocalChanSelect:I
 *   s_LocalChanValues:I
 *   s_TestOutputs:I
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 ***************************************************************************/
int8 IpcSetChannels(void)
{
  const uint8 k_MaxChansPerMsg = 8;
  uint8   *pChanSelect;
  uint8   *pChanValues;
  uint8    remainingChans;
  uint8    tmpPos;
  uint8    chanNum;
  uint8    msgChanCount;
  int8     rc;

dbgPrint(FLASH("IpcSetChannels: "));
dbgPrintln(s_TestOutputs);
  switch (s_TestOutputs)
  {
    case NO_OUTPUT:
      pChanSelect = NULL;
      pChanValues = NULL;
      remainingChans = 0;
      break;

    case DMXW:
      pChanSelect = &s_DmxwChanSelect[1];
      pChanValues = &s_DmxwChanValues[0];
      remainingChans = NUM_CHAN_DMXW;
      break;

    case ONBOARD:
      pChanSelect = &s_LocalChanSelect[1];
      pChanValues = &s_LocalChanValues[0];
      remainingChans = NUM_CHANS_ONBOARD;
      break;

    default:
      dbgPrintln(FLASH("***IpcSetChannels: Invalid test outputs"));
      return -1;
  }

  chanNum = 1;
  while (remainingChans != 0)
  {
    s_IpcOutBufPos = 0;
    s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_VALUE;
    tmpPos = s_IpcOutBufPos++; // Skip the numChans argument for now
    msgChanCount = 0;
    while ( (remainingChans != 0) && (msgChanCount < k_MaxChansPerMsg) )
    {
      if (*pChanSelect)
      {
        msgChanCount++;
        s_IpcOutBuf[s_IpcOutBufPos++] = chanNum;
        s_IpcOutBuf[s_IpcOutBufPos++] = pChanValues[chanNum];
      }
      pChanSelect++;
      chanNum++;
      remainingChans--;
    }
    s_IpcOutBuf[tmpPos] = (s_IpcOutBufPos - tmpPos - 1) / (2 * sizeof(uint8));
    rc = SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
    if (rc != TSTACK_OK)
    {
      return rc;
    }
  }

  return rc;
}


/***************************************************************************
 * Function: IpcUpdateChannel
 * 
 * Send an IPC command to the Output Processor to set a single specific
 * channel's value.
 * 
 * Parameters:
 *   chanNum:I  - Channel number of the channel to upate based on the current
 *                test outputs (onboard or DMXW). The channel number is
 *                expected to be within the value range.
 *   value:I    - The channel's value to be updated.
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 ***************************************************************************/
int8 IpcUpdateChannel(uint8 chanNum, uint8 value)
{
  dbgPrint(FLASH("IpcUpdateChannel: "));
  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_VALUE;
  s_IpcOutBuf[s_IpcOutBufPos++] = 1;
  s_IpcOutBuf[s_IpcOutBufPos++] = chanNum;
  s_IpcOutBuf[s_IpcOutBufPos++] = value;
  return SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
}


/***************************************************************************
 * Function: IpcConfigPixelString
 * 
 * Send an IPC command to the Output Processor to configure the LED pixel
 * string connected to onboard digital output channel #5.
 * 
 * Parameters:
 *   len:I  - The number of individually addressible tricolour LED pixels.
 * Returns:
 *   0,     if command was received without error;
 *   n > 0, if command was received with error n (e.g. TSTACK_BAD_PARM);
 *   -1,    if the command timed out.
 * Input/Output:
 *   s_PixelStringLen:I
 *   s_IpcOutBuf:IO
 *   s_IpcOutBufPos:IO
 ***************************************************************************/
int8 IpcConfigPixelString(uint16 len)
{
  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_PIXCFG;
  s_IpcOutBuf[s_IpcOutBufPos++] = (uint8)(len >> 8);
  s_IpcOutBuf[s_IpcOutBufPos++] = (uint8)(len & 0x00FF);
  return SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);
}


/***************************************************************************
 * Function: ScanPixelLenMenu
 * 
 * Scan for Pixel Length Configuration menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_PixelStringLen:IO
 ***************************************************************************/
int8 ScanPixelLenMenu(void)
{
  const uint8 MAX_SELECTION = 2;
  static int8 selection = 0;
  static boolean menuActive = false;
  static uint8 digits[3];
  uint16 tmpValue;
  uint8  inButton;
  int8   rc;

  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
    selection = 0;
    if (s_PixelStringLen > 999)
    {
      s_PixelStringLen = 999;
    }
    tmpValue = s_PixelStringLen;
    digits[0] = tmpValue / 100;
    tmpValue = tmpValue % 100;
    digits[1] = tmpValue / 10;
    digits[2] = tmpValue % 10;
  }

  /* Display the menu and cursor position */

  uView.clear(PAGE);
  uView.setCursor(0, 0);
  uView.println(FLASH("Length of\npixel\nstring:"));
  uView.setCursor(20, 30);
  for (uint8 i = 0; i < 3; i++)
  {
    if (i == selection)
    {
      uView.setColor(BLACK);
    }
    uView.print(digits[i]);
    uView.setColor(WHITE);
  }
  uView.display();

  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      selection++;
      if (selection > MAX_SELECTION)
      {
        s_PixelStringLen = 0;
        for (uint8 i = 0; i < 3; i++)
        {
          s_PixelStringLen = s_PixelStringLen * 10 + digits[i];
        }
        rc = IpcConfigPixelString(s_PixelStringLen);
        if (rc != 0)
        {
          uView.clear(PAGE);
          uView.invert(true);
          uView.setCursor(0, 20);
          uView.print(FLASH("ERROR:"));
          uView.println(rc);
          uView.display();
          delay(3000);
          uView.invert(false);
        }
        menuActive = false;
      }
      break;
    case KEY_PLUS:
      if (9 == digits[selection])
      {
        digits[selection] = 0;
      }
      else
      {
        digits[selection]++;
      }
      break;
    case KEY_MINUS:
      if (0 == digits[selection])
      {
        digits[selection] = 9;
      }
      else
      {
        digits[selection]--;
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  return menuActive;
}


/***************************************************************************
 * Function: ScanStateMenu
 * 
 * Scan for Test State menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_TestState:IO
 *   s_TestStateNew:O
 ***************************************************************************/
int8 ScanStateMenu(void)
{
  static int8 selection = 0;
  static boolean menuActive = false;
  static boolean gotoSubmenu = false;
  const uint8 MAX_SELECTION = 4;
  uint8 inButton;
  int8  rc;

  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
    gotoSubmenu = false;
    selection = (uint8)s_TestState;
  }

  if (gotoSubmenu)
  {
    gotoSubmenu = ScanPixelLenMenu();
    return menuActive;
  }
  
  /* Display the menu and cursor position */
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  if (STOPPED == s_TestState)
  {
    uView.setColor(BLACK);
    uView.println(FLASH(" STOPPED"));
    uView.setColor(WHITE);
  }
  else
  {
    uView.println(FLASH(" STOP"));
  }
  uView.setCursor(0, 10);
  if (PAUSED == s_TestState)
  {
    uView.setColor(BLACK);
    uView.println(FLASH(" PAUSED"));
    uView.setColor(WHITE);
  }
  else
  {
    uView.println(FLASH(" PAUSE"));
  }
  uView.setCursor(0, 20);
  if (RUNNING == s_TestState)
  {
    uView.setColor(BLACK);
    uView.println(FLASH(" RUNNING"));
    uView.setColor(WHITE);
  }
  else
  {
    uView.println(FLASH(" START"));
  }
  uView.setCursor(0, 30);
  uView.println(FLASH(" Pixel len"));
  uView.setCursor(0, 40);
  uView.println(FLASH(" <back>"));
  uView.setCursor(0, 10 * selection);
  uView.print("*");
  uView.display();

  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      menuActive = false;
      break;
    case KEY_PLUS:
      selection++;
      if (selection > MAX_SELECTION)
      {
        selection = 0;
      }
      break;
    case KEY_MINUS:
      selection--;
      if (selection < 0)
      {
        selection = MAX_SELECTION;
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  if (!menuActive)
  {
    switch(selection)
    {
      case 2: // RUN
        if (s_TestState == PAUSED)
        {
          rc = IpcSetState((TestState_t)selection);
          if (0 != rc)
          {
            uView.clear(PAGE);
            uView.invert(true);
            uView.setCursor(0, 20);
            uView.print(FLASH("ERROR:"));
            uView.println(rc);
            uView.display();
            delay(3000);
            uView.invert(false);
          }
        }
        else
        {
          s_TestStateNew = (TestState_t)selection;
          s_SendConfigData = true;
        }
        break;

      case 0: // STOP
      case 1: // PAUSE
        rc = IpcSetState((TestState_t)selection);
        if (0 != rc)
        {
          uView.clear(PAGE);
          uView.invert(true);
          uView.setCursor(0, 20);
          uView.print(FLASH("ERROR:"));
          uView.println(rc);
          uView.display();
          delay(3000);
          uView.invert(false);
        }
        else
        {
          s_TestState    = (TestState_t)selection;
          s_TestStateNew = s_TestState;
        }
        break;
      case 3: // Pixel string length configuration
        gotoSubmenu = true;
        menuActive = true;
        break;
      case 4: // <exit>
        // Nothing to do.
        break;
      default:
        dbgPrint(FLASH("***ScanStateMenu: bad submenu selected - "));
        dbgPrintln(selection);
    }
  }

  return menuActive;
}


/***************************************************************************
 * Function: ScanValuesMenu
 * 
 * Scan for Set Values menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_DmxwChanSelect[]:I
 *   s_LocalChanSelect[]:I
 *   s_OverridePot:IO
 *   s_TestOutputs:I
 *   s_TestState:I
 *   s_DmxwChanValues[]:IO
 *   s_LocalChanValues[]:IO
 * Note:
 *   Be sure to call this menu only if s_TestOutputs is set to a value
 *   other than NO_OUTPUTS.
 ***************************************************************************/
int8 ScanValuesMenu(void)
{
  static uint32   valueUpdateTime = 0;
  static boolean  menuActive = false;
  static int8     selection = 0;
  static uint8    numChans = 0;
  static int8     lastChan = 0;
  static int8     displayCount = 0;
  static uint8    accumCount = 0;
  static uint8   *pChanSelect = NULL;
  static uint8   *pChanValue = NULL;
  static uint8    channelActive = 0;
  static uint8    displaySet[MAX_SLIDERS];
  uint8  inButton;

  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
    selection = 0;
    if (DMXW == s_TestOutputs)
    {
      numChans    = NUM_CHAN_DMXW;
      pChanSelect = s_DmxwChanSelect;
      pChanValue  = s_DmxwChanValues;
    }
    else
    {
      numChans = NUM_CHANS_ONBOARD;
      pChanSelect = s_LocalChanSelect;
      pChanValue  = s_LocalChanValues;
    }

    // Obtain the first set of channels to be adjusted
    displayCount = 0;
    lastChan = 1;
    while ( (displayCount < MAX_SLIDERS) && (lastChan <= numChans) )
    {
      if (pChanSelect[lastChan])
      {
        displaySet[displayCount++] = lastChan;
      }
      lastChan++;
    }
    accumCount = displayCount;
  }

  if ( s_OverridePot && (millis() > valueUpdateTime) )
  {
    int16 chanValue;

    valueUpdateTime = millis() + 2 * POT_UPDATE_MS;
    chanValue = map(analogRead(POT_PIN), 0, 1023, 0, 263) - 4;
    if (chanValue > 255)
    {
      chanValue = 255;
    }
    else
    {
      if (chanValue < 0)
      {
        chanValue = 0;
      }
    }
    pChanValue[channelActive] = chanValue;
              
    if (RUNNING == s_TestState)
    {
      IpcUpdateChannel(channelActive, chanValue);
    }
  }

  /* Display the menu and cursor position */
  uView.clear(PAGE);
  if (0 == accumCount)
  {
    uView.invert(true);
    uView.setCursor(0, 10);
    uView.println(FLASH("No active\nchannels\nselected\nfor test"));
    uView.display();
    delay(3000);
    uView.invert(false);
    menuActive = false;
    return menuActive;
  }
  else
  {
    for (uint8 i = 0; i < displayCount; i++)
    {
      uView.setCursor(0, i * 8 + 1);
      if (selection == i)
      {
        uView.setColor(BLACK);
      }
      uView.print(displaySet[i] / 10);
      uView.print(displaySet[i] % 10);
      uView.setColor(WHITE);
      valueWidget[i]->setValue(pChanValue[displaySet[i]]);
      valueWidget[i]->reDraw();
    }
    uView.display();
  }

  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      s_OverridePot = true;
      channelActive   = displaySet[selection];
      
      // Blink the channel number
      uView.setCursor(0, selection * 8 + 1);
      uView.setColor(WHITE);
      uView.print(channelActive / 10);
      uView.print(channelActive % 10);
      uView.display();
      uView.setColor(BLACK);
      delay(500);
      
      break;
    case KEY_PLUS:
      selection++;
      s_OverridePot = false;
      if (selection == displayCount)
      {
        // Get next set of sliders
        if (lastChan <= numChans)
        {
          selection = 0;
          displayCount = 0;
          while ( (displayCount < MAX_SLIDERS) &&
                  (lastChan <= numChans) )
          {
            if (pChanSelect[lastChan])
            {
              displaySet[displayCount++] = lastChan;
            }
            lastChan++;
          }
          accumCount += displayCount;
        }
        else
        {
          selection--;
        }
      }
      break;
    case KEY_MINUS:
      selection--;
      s_OverridePot = false;
      if (selection < 0)
      {
        if (accumCount <= MAX_SLIDERS)
        {
          menuActive = false;
        }
        else
        {
          // Get previous set of sliders
          selection = 0;
          if (accumCount > MAX_SLIDERS)
          {
            uint8 tmpChan;
          
            // Backup to the start of the current screen
            lastChan--;
            while (displayCount > 0)
            {
              if (pChanSelect[lastChan])
              {
                displayCount--;
              }
              lastChan--;
            }
            tmpChan = lastChan + 1;

            // Now grab the previous set of sliders
            displayCount = MAX_SLIDERS;
            while ( (displayCount > 0) && (lastChan > 0) )
            {
              if (pChanSelect[lastChan])
              {
                displaySet[--displayCount] = lastChan;
              }
              lastChan--;
            }
            lastChan = tmpChan;
            displayCount = MAX_SLIDERS;
            accumCount -= displayCount;
          }
        }
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  return menuActive;
}


/***************************************************************************
 * Function: ScanOutputsMenu
 * 
 * Scan for Test Outputs menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_TestState:I
 *   s_TestOutputs:IO
 ***************************************************************************/
int8 ScanOutputsMenu(void)
{
  static int8 selection = 0;
  static boolean menuActive = false;
  static boolean gotoSubmenu = false;
  const uint8 MAX_SELECTION = 4;
  uint8 inButton;
  int8  rc;

  if (gotoSubmenu)
  {
    if (NO_OUTPUT == s_TestOutputs)
    {
      uView.clear(PAGE);
      uView.invert(true);
      uView.setCursor(0, 10);
      uView.println(FLASH("  Select\n   test\n outputs\n  first"));
      uView.display();
      delay(3000);
      uView.invert(false);
      gotoSubmenu = false;
    }
    else
    {
      gotoSubmenu = ScanValuesMenu();
      menuActive = gotoSubmenu;
    }
    return menuActive;
  }
  
  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
  }
  
  /* Display the menu and cursor position */
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  if (NO_OUTPUT == s_TestOutputs)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" None"));
  uView.setColor(WHITE);
  uView.setCursor(0, 10);
  if (ONBOARD == s_TestOutputs)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" Onboard"));
  uView.setColor(WHITE);
  uView.setCursor(0, 20);
  if (DMXW == s_TestOutputs)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" DMXW"));
  uView.setColor(WHITE);
  uView.setCursor(0, 30);
  uView.println(FLASH(" Values..."));
  uView.setCursor(0, 40);
  uView.println(FLASH(" <back>"));
  uView.setCursor(0, 10 * selection);
  uView.print("*");
  uView.display();

  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      menuActive = false;
      break;
    case KEY_PLUS:
      selection++;
      if (selection > MAX_SELECTION)
      {
        selection = 0;
      }
      break;
    case KEY_MINUS:
      selection--;
      if (selection < 0)
      {
        selection = MAX_SELECTION;
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  if (!menuActive)
  {
    if ( (selection < 3) && (STOPPED != s_TestState) )
    {
      uView.clear(PAGE);
      uView.invert(true);
      uView.setCursor(0, 20);
      uView.println(FLASH("Stop test first"));
      uView.display();
      delay(3000);
      uView.invert(false);
    }
    else
    {
      switch(selection)
      {
        case 0: // None
        case 1: // Onboard
        case 2: // DMXW
          s_TestOutputs = (TestOutput_t)selection;
          break;
        case 3: // Values...
          gotoSubmenu = true;
          menuActive = true;
          break;
        case 4: // <exit>
          // Nothing to do.
          break;
        default:
          dbgPrint(FLASH("***ScanOutputsMenu: bad submenu selected - "));
          dbgPrintln(selection);
      }
    }
  }

  return menuActive;
}


/***************************************************************************
 * Function: ScanTypeMenu
 * 
 * Scan for Test Type menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_TestState:I
 *   s_TestType:IO
 ***************************************************************************/
int8 ScanTypeMenu(void)
{
  static int8 selection = 0;
  static boolean menuActive = false;
  const uint8 MAX_SELECTION = 4;
  uint8 inButton;
  int8  rc;

  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
    selection = (uint8)s_TestType;
  }
  
  /* Display the menu and cursor position */
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  if (DISABLED == s_TestType)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" None"));
  uView.setColor(WHITE);
  uView.setCursor(0, 10);
  if (ONBOARD == s_TestType)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" Manual"));
  uView.setColor(WHITE);
  uView.setCursor(0, 20);
  if (DMXW == s_TestType)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" Sweep"));
  uView.setColor(WHITE);
  uView.setCursor(0, 30);
  if (PIXEL == s_TestType)
  {
    uView.setColor(BLACK);
  }
  uView.println(FLASH(" Pixel"));
  uView.setColor(WHITE);
  uView.setCursor(0, 40);
  uView.println(FLASH(" <back>"));
  uView.setCursor(0, 10 * selection);
  uView.print("*");
  uView.display();

  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      menuActive = false;
      break;
    case KEY_PLUS:
      selection++;
      if (selection > MAX_SELECTION)
      {
        selection = 0;
      }
      break;
    case KEY_MINUS:
      selection--;
      if (selection < 0)
      {
        selection = MAX_SELECTION;
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  if (!menuActive)
  {
    if ( (MAX_SELECTION != selection) && (STOPPED != s_TestState) )
    {
      uView.clear(PAGE);
      uView.invert(true);
      uView.setCursor(0, 20);
      uView.println(FLASH("Stop test first"));
      uView.display();
      delay(3000);
      uView.invert(false);
    }
    else
    {
      switch(selection)
      {
        case 0: // None
        case 1: // Manual
        case 2: // Sweep
        case 3: // Pixel
          s_TestType = (TestType_t)selection;
          break;
        case 4: // <exit>
          // Nothing to do.
          break;
        default:
          dbgPrint(FLASH("***ScanTypeMenu: bad submenu selected - "));
          dbgPrintln(selection);
      }
    }
  }

  return menuActive;
}


/***************************************************************************
 * Function: ScanChannelsMenu
 * 
 * Scan for Channel Selection menu item selections.
 * 
 * Parameters: (none)
 * Returns: true iff the menu needs to remain active.
 * Input/Output:
 *   s_TestOutputs:I
 *   s_DmxwChanSelect[]:IO
 *   s_LocalChanSelectp[]:IO
 ***************************************************************************/
int8 ScanChannelsMenu(void)
{
  static int8 selection = -1;
  static boolean menuActive = false;
  static boolean allSelectToggle = true;
  static uint8 numChans;
  uint8 *pChanSelect;
  uint8 inButton;
  int8 rowLen;
  uint8 currChan;
  uint8 potChan;
  uint8 chanSelectValue;
    
  if (!menuActive)
  {
    // The menu is being re-activated.
    menuActive = true;
    selection = -1;
    allSelectToggle = true;
    if (DMXW == s_TestOutputs)
    {
      numChans = NUM_CHAN_DMXW;
      pChanSelect = s_DmxwChanSelect;
    }
    else
    {
      numChans = NUM_CHANS_ONBOARD;
      pChanSelect = s_LocalChanSelect;
    }
  }

  /* Display the menu and cursor position */
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  if (-1 == selection)
  {
    uView.setColor(BLACK);
  }
  uView.print(FLASH("ALL"));
  uView.setColor(WHITE);
  uView.print(FLASH(" "));
  if (0 == selection)
  {
    uView.setColor(BLACK);
  }
  uView.print(FLASH("<exit>"));
  uView.setColor(WHITE);
  currChan = 0;
  potChan = pChanSelect[0];
  for (uint8 i = 0; i <= numChans; i += 10)
  {
    uView.setCursor(0, ((i / 10) + 1) * 8);
    rowLen = numChans - i;
    if (rowLen > 10)
    {
      rowLen = 10;
    }
    for (uint8 j = 1; j <= rowLen; j++)
    {
      currChan++;
      if (currChan == selection)
      {
        uView.setColor(BLACK);
      }
      if (currChan == potChan)
      {
        uView.print(FLASH("P"));
      }
      else if (pChanSelect[currChan])
      {
        uView.print(FLASH("*"));
      }
      else
      {
        uView.print(FLASH("."));
      }
      uView.setColor(WHITE);
    }
  }
  uView.display();
  
  /* Determine button action, if any */
  inButton = ScanButton();
  switch (inButton)
  {
    case KEY_SELECT:
      if (0 == selection)
      {
        // back
        menuActive = false;
      }
      else if (-1 == selection)
      {
        for (uint8 i = 1; i <= numChans; i++)
        {
          pChanSelect[i] = allSelectToggle;
        }
        allSelectToggle = !allSelectToggle;
      }
      else
      {
        if (potChan == selection)
        {
          pChanSelect[selection] = false;
          pChanSelect[0] = 0;
        }
        else
        {
          chanSelectValue = (uint8)pChanSelect[selection];
          chanSelectValue++;
          if (chanSelectValue > 1)
          {
            chanSelectValue = 1;
            if (potChan != 0)
            {
              pChanSelect[potChan] = true;
            }
            pChanSelect[0] = selection;
          }
          pChanSelect[selection] = (boolean)chanSelectValue;
        }
      }
      break;
    case KEY_PLUS:
      if (selection < 1)
      {
        selection = 1;
      }
      else
      {
        selection += 10;
        if (selection > numChans)
        {
          selection = -1;
        }
      }
      break;
    case KEY_MINUS:
      if (selection > 0)
      {
        if ( ((selection - 1) % 10) == 0)
        {
          selection += 9;
          if (selection > numChans)
          {
            selection = numChans;
          }
        }
        else
        {
          selection--;
        }
      }
      else
      {
        selection = -selection - 1;
      }
      break;
    default:
      /* nothing to do */
      ;
  }
  /* Wait until no button is pressed */
  while (ScanButton() != KEY_NONE) {}

  return menuActive;
}


/***************************************************************************
 * Function: ScanTopMenu
 * 
 * Scan for top level menu item selections.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Input/Output:
 *   s_TestOutputs:I
 *   s_TestState:I
 ***************************************************************************/
void ScanTopMenu(void)
{
  static int8 selection = 0;
  static boolean gotoSubmenu = false;
  const uint8 MAX_SELECTION = 4;
  uint8 inButton;

  if (!gotoSubmenu)
  {
    /* Display the menu and cursor position */
    uView.clear(PAGE);
    uView.setCursor(0, 0);
    uView.print(" [");
    //uView.setColor(BLACK);
    switch (s_TestState)
    {
      case STOPPED:
        uView.print(FLASH("STOPPED"));
        break;
      case PAUSED:
        uView.print(FLASH("PAUSED"));
        break;
      case RUNNING:
        uView.print(FLASH("RUNNING"));
        break;
      default:
        uView.print(FLASH("???????"));
    }
    //uView.setColor(WHITE);
    uView.println("]");
    uView.setCursor(0, 10);
    uView.println(FLASH(" Outputs"));
    uView.setCursor(0, 20);
    uView.println(FLASH(" Test Type"));
    uView.setCursor(0, 30);
    uView.println(FLASH(" Channels"));
    uView.setCursor(0, 40);
    uView.println(FLASH(" Save"));
    uView.setCursor(0, selection * 10);
    uView.print("*");
    uView.display();

    /* Determine button action, if any */
    inButton = ScanButton();
    switch (inButton)
    {
      case KEY_SELECT:
        gotoSubmenu = true;
        break;
      case KEY_PLUS:
        selection++;
        if (selection > MAX_SELECTION)
        {
          selection = 0;
        }
        break;
      case KEY_MINUS:
        selection--;
        if (selection < 0)
        {
          selection = MAX_SELECTION;
        }
        break;
      default:
        /* nothing to do */
        ;
    }

    /* Wait until no button is pressed */
    while (ScanButton() != KEY_NONE) {}
  }
  else
  {
    // Go to the currently selected submenu.
    switch (selection)
    {
      case 0: // State
        gotoSubmenu = ScanStateMenu();
        break;
      case 1: // Outputs
        gotoSubmenu = ScanOutputsMenu();
        break;
      case 2: // Test type
        gotoSubmenu = ScanTypeMenu();
        break;
      case 3: // Channel selection
        if (NO_OUTPUT == s_TestOutputs)
        {
          uView.clear(PAGE);
          uView.invert(true);
          uView.setCursor(0, 10);
          uView.println(FLASH("  Select\n   test\n outputs\n  first"));
          uView.display();
          delay(3000);
          uView.invert(false);
          gotoSubmenu = false;
        }
        else
        {
          gotoSubmenu = ScanChannelsMenu();
        }
        break;
      case 4: // Save configuration data to EEPROM
        gotoSubmenu = false;
        EepromSave();
        uView.clear(PAGE);
        uView.invert(true);
        uView.setCursor(0, 20);
        uView.println(FLASH("Parameters   Saved"));
        uView.display();
        delay(2000);
        uView.invert(false);
        break;
      default:
        dbgPrint(FLASH("***ScanTopMenu: bad submenu selected - "));
        dbgPrintln(selection);
    }
  }
}


/***************************************************************************
 * Function: UpdatePotChannel
 * 
 * While a test is running, check for changes to the potentiometer and
 * update the corresponding channel value.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Inputs/Outputs:
 *   s_DmxwChanSelect[]:I
 *   s_LocalChanSelect[]:I
 *   s_OverridePot:I
 *   s_TestOutputs:I
 *   s_TestState:I
 *   s_TestType:I
 ***************************************************************************/
void UpdatePotChannel(void)
{
  const uint8 kMaxValHistory = 4;
  static uint8 values[kMaxValHistory];
  static uint8 oldPotChanValue;
  static int8  currValIdx = 0;
  static uint8 skipCount = 0;
  static uint32 updateTime = 0;
  int32  potValue;
  uint8  newPotChanValue;
  uint8  potChan;
  
  if (   (RUNNING != s_TestState)
      || (millis() < updateTime)
      || (NO_OUTPUT == s_TestOutputs)
      || s_OverridePot
     )
  {
    return;
  }

  if (CHAN_SWEEP == s_TestType)
  {
    return;
  }

  updateTime = millis() + POT_UPDATE_MS;

  // Read in the current potentiometer value, normalize it to the range
  // 0 ... 255, and calculate the trailing average of the last kMaxValHistory
  // readings. If that value differs from the previously recorded value,
  // update the associated output channel value.
  potValue = analogRead(POT_PIN);
  potValue = map(potValue, 0, 1023, 0, 263) - 4;
  if (potValue > 255)
  {
    potValue = 255;
  }
  else
  {
    if (potValue < 0)
    {
      potValue = 0;
    }
  }
  values[currValIdx] = potValue;
  currValIdx = (currValIdx + 1) % kMaxValHistory;
  potValue = 0;
  for (uint8 i = 0; i < kMaxValHistory; i++)
  {
    potValue += values[currValIdx];
  }
  potValue = potValue / kMaxValHistory; 
  if (potValue == oldPotChanValue)
  {
    return;
  }
  else
  {
    potValue -= oldPotChanValue;
    if (abs(potValue) < 5)
    {
      skipCount++;
      if (skipCount < 4)
      {
        return;
      }
      skipCount = 0;
    }
    else
    {
      skipCount = 0;
    }
    potValue += oldPotChanValue;
  }
  oldPotChanValue = potValue;
  
  switch (s_TestOutputs)
  {
    case ONBOARD:
      potChan = s_LocalChanSelect[0];
      break;

    case DMXW:
      potChan = s_DmxwChanSelect[0];
      break;

    default:
      return;
  }

  if (0 != potChan)
  {
    IpcUpdateChannel(potChan, potValue);
  }
}


/***************************************************************************
 * Function: InitializeSystem
 * 
 * Initialize the HMI processor and Output processor.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Inputs/Outputs:
 *   s_IpcOutBuf:O
 *   s_IpcOutBufPos:O
 *   s_OutputRebooted:O
 *   s_SendConfigData:O
 *   s_TestState:O
 ***************************************************************************/
void InitializeSystem(void)
{
  int8 rc;

  dbgPrintln(FLASH("Init System..."));
  s_TestState = STOPPED;
  s_SendConfigData = false;
  s_IpcOutBufPos = 0;
  s_IpcOutBuf[s_IpcOutBufPos++] = TSTCMD_INIT;
  rc = SendIpcCommand(s_IpcOutBuf, s_IpcOutBufPos);

  // If the the TSTCMD_INIT succeeded, we're ok to configure the output
  // processor. Otherwise, we'll end up getting called again to retry
  // indefinitely.
  if (0 == rc)
  {
    s_OutputRebooted = false;
    s_SendConfigData = true;
  }
}


/***************************************************************************
 * Function: CheckRam
 * 
 * Test how much RAM is left on the MPU, outputing the results out to
 * the serial port.
 * 
 * Parameters:     (none)
 * Returns:        (none)
 * Inputs/Outputs: (none)
 * IMPORTANT: Avoid calling this function during normal operation. This
 *            should be treated as a temporary diagnostic test only. 
 ***************************************************************************/
void CheckRam()
{
  extern int __bss_end;
  extern void *__brkval;
  /* Comment out either of the following to limit the CheckRam output */
  #define OUT_TO_SERIAL
  int freeValue;
  
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
    
  #ifdef OUT_TO_SERIAL
    logPrint(F("Free RAM: "));
    logPrintln(freeValue);
  #endif
}


//=====================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(10);
  
  pinMode(PIN_BUTTON_PLUS,   INPUT_PULLUP);
  pinMode(PIN_BUTTON_MINUS,  INPUT_PULLUP);
  pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);

  // Check if we need to clear out EEPROM and start from scratch
  s_ResetEeprom = (EEPROM.read(EEPROM_FW_ADDR) != FW_VERSION_c);
  if (s_ResetEeprom)
  {
    // Record new F/W version and start with blank data
    EEPROM.write(EEPROM_FW_ADDR, FW_VERSION_c);
  }
  else
  {
    // Read in data stored in EEPROM
    if ( EepromLoad() )
    {
      s_SendConfigData = true;
    }
  }

  // Create MicroView graphical widgets
  for (uint8 i = 0; i < MAX_SLIDERS; i++)
  {
    valueWidget[i] = new MicroViewSlider(13, i*8, 0, 255);
  }

  // Write copyright banner to the consoles
  logPrintln();
  logPrintln();
  for (uint8 i = 1; i <= 80; i++)
    logPrint(FLASH("#"));
  logPrintln();
  logPrintln(FLASH("DMX Tester (Human Machine I/F processor)"));
  logPrint(FLASH("S/W: "));
  logPrint(SW_VERSION_c);
  logPrint(FLASH("\tF/W: "));
  logPrint(FW_VERSION_c);
  logPrint(FLASH("\t"));
  logPrintln(COPYRIGHT);
  
  CheckRam();
  s_OutComms.begin(IPC_COMMS_BAUD);

  // Display tester's banner on the OLED display
  uView.begin();
  uView.contrast(255);
  uView.clear(PAGE);
  uView.invert(true);
  uView.setCursor(0, 10);
  uView.println(FLASH("   DMXW\n  Tester\n"));
  uView.print("v");
  uView.println(SW_VERSION_c);
  uView.display();
  delay(3000);
  uView.clear(PAGE);
  uView.invert(false);
  uView.display();
}

//------------------------------------------------------------------
void loop()
{
  if (s_OutputRebooted)
  {
    if (!s_Initialized)
    {
      s_Initialized = true;
    }
    else
    {
      uView.clear(PAGE);
      uView.invert(true);
      uView.setCursor(0, 0);
      uView.print(FLASH("Recoveringfrom\nSystem\nError"));
      uView.display();
      delay(3000);
      uView.invert(false);
    }
    InitializeSystem();
  }
  
  if (!s_SendConfigData)
  {
    if ( (s_TestStateNew != s_TestState) && (RUNNING == s_TestStateNew) )
    {
      int8 rc;

      dbgPrintln(FLASH("Send run msg"));
      rc = IpcSetState(RUNNING);
      if (rc != 0)
      {
        uView.clear(PAGE);
        uView.invert(true);
        uView.setCursor(0, 20);
        uView.print(FLASH("ERROR:"));
        uView.println(rc);
        uView.display();
        delay(3000);
        uView.invert(false);
        s_TestStateNew = s_TestState;
      }
      else
      {
        s_TestState = s_TestStateNew;
      }
    }

    ///////////////////////////////////////////////
    //  Main control functions
    ///////////////////////////////////////////////
    ScanTopMenu();
    UpdatePotChannel();
  }
  else
  {
    dbgPrintln(FLASH("Sending config"));
    s_SendConfigData = false;
    IpcSetState(STOPPED);
    if (IpcConfigureTest() != 0)
    {
      s_SendConfigData = true;
    }
    if ( !s_SendConfigData && (IpcSelectChannels() != 0) )
    {
      s_SendConfigData = true;
    }
    if ( !s_SendConfigData && (IpcSetChannels() != 0) )
    {
      s_SendConfigData = true;
    }
    if ( !s_SendConfigData && (IpcConfigPixelString(s_PixelStringLen) != 0) )
    {
      s_SendConfigData = true;
    }
    
    if (s_SendConfigData)
    {
      // We failed an attempt to send configuration data: Try again.
      dbgPrintln(FLASH("Config failed."));
      s_SendConfigData  = false;
      uView.clear(PAGE);
      uView.invert(true);
      uView.setCursor(0, 0);
      uView.println(FLASH("Config\nfailed"));
      uView.display();
      delay(3000);
      uView.invert(false);
      uView.clear(PAGE);
      uView.display();
    }
  }
}
