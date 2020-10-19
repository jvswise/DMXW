/******************************************************************************
 * DMXW Tester (Output processor)
 * 
 * This sketch implements the output processor's capabilities as part of a 
 * simple tester for DMX wireless remote slaves as well as for onboard testing
 * of components that can be connected to a remote slave. The sketch is
 * targeted for the Arduino-compatible Moteino boards having an RFM69 radio.
 * The following is a summary of the tester's capabilities:
 * 
 *   Onboard testing:
 *   ---------------
 *    - There are 4 pseudo analogue (PWM) capable channels that emulate a
 *      remote slave's PWM Chan #1 thru #4. These use the same technology as
 *      the slaves use, each incorporating an FQP30N06L N-Channel MOSFET
 *      capable of carrying up to 32A @ 60V. (Note that, like the slaves, the
 *      MOSFETs aren't heat-sinked and, so, can only carry about 5 - 7A of
 *      current. In fact, there is much less cooling capacity in the onboard
 *      channels. So, tests with high current circuits should be performed
 *      on for very short durations in order to avoid buring out a channel's
 *      N-channel MOSFET.)
 *        - These channels must be driven externally thru an external load and
 *          positive voltage source. The external voltage source must also be
 *          connected to the tester's ground (GND) connector.
 *    - There is one digital channel that can operate at 3.3Vdc or 5Vdc logic
 *      levels. This can be used to drive, e.g. a logic driven relay or an RGB 
 *      LED pixel strip. (For pixel strips, use the 5Vdc output.)
 *        - This output is driven internally by the tester.
 *    - These five channels are given pseudo DMXW channel numbers 1 thru 5 
 *      (with the digital channel being DMXW channel 5).
 *    - Activate from 1 to 5 channels for simultaneous testing.
 *        - Each channel can be assigned a value from 0 thru 255 via
 *          menu options.
 *        - Any one of the 4 PWM channels can be associated with the onboard
 *          potentiometer for dynamic control--with the exception noted in
 *          the next bullet.
 *        - The digital channel can be run in either Discrete (on/off) mode
 *          or in Pixel Strip Test mode; the latter mode runs a rainbow
 *          test pattern, the rate of change of which is controlled
 *          dynamically via the onboard potentiometer.
 *
 *  DMXW remote slave testing:
 *  -------------------------
 *    - Activate (and deactivate) up to the maximum number of DMXW channels
 *      (channels 1 thru 48) for simultaneous testing.
 *        -- Each channel can be assigned a value from 0 thru 255 via
 *          menu options.
 *        - One channel can be associated with the onboard potentiometer
 *          for dynamic control.
 *      
 * The DMXW tester incorporates an RFM69 radio operating at 433MHz (compatible
 * with DMXW remote slave nodes).
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
 *   *** Need to import <SoftwareSerial.h> and translate all print statements
 *       from Serial.print...   to mySerial.print... as the 
 *       
 * WARNING:
 * =======
 *   The AdaFruit_NeoPixel library interferes with the SoftwareSerial library.
 *   It appears that if pixel updates are done too frequently (esp. on a long
 *   string), then the SoftwareSerial port will get (and remain) corrupted.
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

#include <SoftwareSerial.h>
#include <RFM69_DMX.h>
#include <SPI.h>
#include "DmxwDefs.h"
#include "TesterDefs.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>



/*****************************  Constants  ***********************************/

#define COPYRIGHT       "(C)2020, A.J. van Schouwen"
#define SW_VERSION_c    "1.0 (2020-04-25)"
#define FW_VERSION_c    1   // Increment (with wraparound) for new F/W;
                            //   clears EEPROM.

//#define DEBUG_ON       // Uncomment to turn off debug output to serial port.
#define LOGGING_ON     // Uncomment to turn off packet logging to serial port.

#define EEPROM_FW_ADDR           0

#define SERIAL_BAUD         115200  // For serial debug console
#define IPC_COMMS_BAUD       19200  // For serial IPC with HMI processor


/*** PWM test channel output pins ***/
#define CHAN1_PWM_PIN            3
#define CHAN2_PWM_PIN            5
#define CHAN3_PWM_PIN            6
#define CHAN4_PWM_PIN            9

/*** Digital test channel output pin ***/
#define CHAN5_PWM_PIN            4

/*** IPC serial port pins (for SoftwareSerial) ***/
#define HMI_COMMS_RX             7  // Serial RX port for HMI processor comms
#define HMI_COMMS_TX             8  // Serial TX port for HMI processor comms

//??JVS
// A 45 ms TX delay equates to a 22 Hz DMXW refresh rate. A DMX-512
// network running the full 512 channels has a refresh rate of 44Hz.
// So we're running at half the slowest refresh rate.
// (Note that a DMX-512 network running fewer channels can refresh
// much more quickly. The minimum frame time is 1196 microsecs (or 836 Hz),
// which can be achieved with at most 24 channel slots.)
#define DMXW_TX_DELAY            100  // milliseconds
//#define DMXW_TX_DELAY            35  // milliseconds
#define DMXW_MAX_BUF_LEN     (MAX_DMXW_CHANS + 3)

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

/** Pixel string definitions
 *  - The actual colour ordering on a given pixel string may be different.
 *    However, it doesn't matter that much as we're only going to run
 *    a rainbow effect on it.
 */
#define LEDS_PER_PIX        3   // 3 LEDs (colours) per pixel.
#define PIX_OFFS_RED        0
#define PIX_OFFS_GREEN      1
#define PIX_OFFS_BLUE       2
#define PIX_UPDATE_LOWER   10   // Pixel strip update period lower bound (ms)

const uint8 kMyNodeId = GATEWAYID; // Tester acts as a DMXW Gateway proxy.
const bool  kPromiscuousMode = true;  // sniff all packets on network iff true

// Mapping from onboard test channel number to output pin #
const int8 kChanToPinMap[] =
{
  /* Channel 0 */   CHAN_UNDEFINED,
  /* Channel 1 */   CHAN1_PWM_PIN,
  /* Channel 2 */   CHAN2_PWM_PIN,
  /* Channel 3 */   CHAN3_PWM_PIN,
  /* Channel 4 */   CHAN4_PWM_PIN,
  /* Channel 5 */   CHAN5_PWM_PIN
};


/*****************************  State Variables  ******************************/

/* Current channel values
 *  - When the test output is DMXW, all valid DMXW channels (and not just those
 *    that are under test) require valid values. Those channels that are not
 *    involved in the test are set to zero. (NOTE that this will likely
 *    conflict with an active DMXW gateway if it happens to be running DMXW
 *    activity.)
 *  - When the test output is ONBOARD, all onboard channels (and not just those
 *    that are under test) require valid values. Those channels that are not
 *    involved in the test are set to zero.
 */
uint8 s_ChanValues[MAX_DMXW_CHANS + 1]; // Channel values for chans 1 - 48.
                                        //  (Entry at index 0 isn't used.)
uint8 s_NumChanValues = 0;              // Current # of valid output channels.
                                        //  (normally either NUM_CHANS_ONBOARD
                                        //  or NUM_CHAN_DMXW)

/** List of channels being tested.
 *  - s_NumTestChannels defines how many entries in s_TestChannels[] are valid
 *    (from index zero up to index (numTestChannels - 1).
 *  - Each valid entry in s_TestChannels[] maps to a channel number for a
 *    channel (onboard or via the DMXW network) that is under test. The array
 *    is kept sorted by increasing channel number.
 */
uint8  s_TestChannels[MAX_DMXW_CHANS];
uint8  s_NumTestChannels;   // Number of valid (active) entries in
                            //  s_TestChannels[].

TestOutput_t s_TestOutputs = NO_OUTPUT; // Current test output
TestType_t   s_TestType    = DISABLED;   // Current test type
TestState_t  s_TestState   = STOPPED;    // Current test state

// Interprocessor Communications serial port.
SoftwareSerial s_HmiComms(HMI_COMMS_RX, HMI_COMMS_TX);
uint8 s_IpcBuf[MAX_SERIAL_BUF_LEN];
uint8 s_IpcBufPos = 0;
uint8 s_IpcSeqNum = 0;

// DMXW packet buffer
uint8 s_DmxwBuf[DMXW_MAX_BUF_LEN];

// LED Pixel String state
Adafruit_NeoPixel *s_pStrip    = NULL; // Ptr to pixel string object
uint8             *s_pPixels   = NULL; // Direct ptr to pixel data.
uint16             s_NumPixels = 0;    // # individually addressable pixels in
                                       //   the onboard-connected pixel string.
uint8              s_FxDelay = 0;      // Pixel string rainbow effect delay
                                       // for speed control:
                                       //    0 = fastest speed
                                       //  255 = slowest

// RFM Radio related
RFM69 s_Radio;
uint8 s_DstNodeId;
uint8 s_SrcNodeId;

bool s_ResetEeprom = false;
bool s_Rebooted = true;  // This remains true until the HMI processor sends
                         // us a TSTCMD_INIT IPC message.



/******************************************************************************
 * Function: EepromLoad
 * 
 * Load saved test parameter values from EEPROM.
 * 
 * Parameters: (none)
 * Returns:    (none)
 ******************************************************************************/
void EepromLoad(void)
{
  int addr;

//??JVS
//  numDmxwChans = EEPROM.read(EEPROM_NUMDMXNODES_ADDR);
//  if (numDmxwChans == 0)
    return;
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
  int addr;

//  EEPROM.write(EEPROM_NUMDMXNODES_ADDR, numDmxwChans);
//??JVS
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

/******************************************************************************
 * Function: PrintDmxwCmd
 * 
 * Print a supported DMXW command in ASCII form to the console.
 * 
 * Parameters:
 *   command:I  - DMXW command code.
 * Returns:    (none)
 ******************************************************************************/
void PrintDmxwCmd(uint8 command)
{
  switch (command)
  {
    case CMD_RUN:    dbgPrint(FLASH("CMD_RUN"));     break;
    case CMD_ECHO:   dbgPrint(FLASH("CMD_ECHO"));    break;
    case CMD_CHAN:   dbgPrint(FLASH("CMD_CHAN"));    break;
    case CMD_OFF:    dbgPrint(FLASH("CMD_OFF"));     break;
    case CMD_UNDEF:  dbgPrint(FLASH("CMD_UNDEF"));   break;
    default:
      dbgPrint(FLASH("<unsupported>["));
      dbgPrint(command);
      dbgPrint("]");
  }
}

/*******************************************************************************
 * Function: SendBuffer
 * 
 * Send a broadcast or unicast message across the DMXW network, and if
 * requested, wait for an ACK.
 * 
 * Parameters:
 *   dst:I         - DMXW destination node ID.
 *   payload:I     - Pointer to the message's payload buffer.
 *   sendSize:I    - Number of bytes in the payload buffer.
 *   requestAck:I  - Set to TRUE iff an ACK is to be requested.
 * Returns:
 *   DMXW acknowledgement code.
 *******************************************************************************/
AckCode_t SendDmxwPacket(int dst, uint8 *payload, int sendSize, bool requestAck)
{
  unsigned long sentTime;

#if 0
  dbgPrint(millis());
  dbgPrint(FLASH(" TX Dst["));
  dbgPrint(dst);
  dbgPrint(FLASH("] Size["));
  dbgPrint(sendSize);
  dbgPrint(FLASH("] Data["));
  PrintDmxwCmd(payload[2]);
  for (uint8 i = 0; i < sendSize; i++)
  {
    dbgPrint(" ");
    dbgPrint(payload[i]);
  }
  dbgPrintln("]");
#endif

  if (dst == BROADCASTID)
    requestAck = false;
  for (byte i = 0; i <= TX_NUM_RETRIES; i++)
  { 
    s_Radio.send(dst, payload, sendSize, requestAck);
    sentTime = millis();
    if (requestAck)
    {
      while ( (millis() - sentTime) < ACK_WAIT_TIME)
      {
        if (s_Radio.ACKReceived(dst))
        {
          //dbgPrint(" ~ms:"); dbgPrintln(millis()-sentTime);
          return s_Radio.DATA[0];
        }
      }
    }
    else
    {
      return ACK_OK;
    }
    //dbgPrint(" RETRY#"); dbgPrintln(i+1);
  }

  return ACK_ETIME;
}

/*******************************************************************************
 * Function: MapToColour
 *   Map LED pixel RGB colour components to a uint32 colour value.
 * Parameters:
 *   r:I  - Amount of red.
 *   g:I  - Amount of green.
 *   b:I  - Amount of blue.
 * Returns: uint32 version of the specified colour corrected for the pixel
 *          strip's LED wiring.
 * Note:
 *   This function uses a more generic mapping mechanism in which it's
 *   assumed that colour ordering varies for different pixel strings. For
 *   our purposes, the pixel[] array isn't really necessary.
 ******************************************************************************/
uint32 MapToColour(uint8 r, uint8 g, uint8 b)
{
  uint8  pixel[3];

  pixel[PIX_OFFS_RED]   = r;
  pixel[PIX_OFFS_GREEN] = g;
  pixel[PIX_OFFS_BLUE]  = b;
  
  return   ((uint32)pixel[0] << 16)
         | ((uint32)pixel[1] <<  8)
         | pixel[2];
}

/*******************************************************************************
 * Function: ColourWheel
 *   Select a colour from a wheel of 256 colours that range from
 *   red to green to blue and back to red again.
 *
 *   pure red (0), thru red/greens to pure green (85), thru green/blues to
 *   pure blue (170), thru blue/reds to pure red (255 -> 0) again.
 * Parameters:
 *   wheelPos:I  - Colour wheel slot:
 *                      0 = red
 *                     42 = yellowish (red/green)
 *                     85 = green
 *                    127 = cyan (green/blue)
 *                    170 = blue
 *                    212 = purple (blue/red)
 * Returns:   WASPCMD_NONE
 * Inputs/Ouputs: (none)
 ******************************************************************************/
uint32 ColourWheel(uint8 wheelPos)
{
  if(wheelPos < 85)
  {
   return MapToColour(255 - wheelPos * 3, wheelPos * 3, 0);
  }
  else if(wheelPos < 170)
  {
   wheelPos -= 85;
   return MapToColour(0, 255 - wheelPos * 3, wheelPos * 3);
  }
  else
  {
   wheelPos -= 170;
   return MapToColour(wheelPos * 3, 0, 255 - wheelPos * 3);
  }
}

/*******************************************************************************
 * Function: RainbowFxIter
 *   Run an iteration of the animated rainbow effect.
 * Parameters:
 *   iterCount:IO  - Address of the state variable for the effect. Upon first
 *                   invocation, the value should be set to 0. On subsequent
 *                   calls, use the output value.
 * Returns:    (none)
 * Inputs/Ouputs:
 *   s_NumPixels:I
 *   s_FxDelay:I
 *   s_pPixels:IO
 *   s_pStrip:I
 ******************************************************************************/
void RainbowFxIter(uint16 *iterCount)
{
  static uint32 resumeTime = 0;
  uint32   colour;
  uint8   *pixels;
  uint16   i = 0;
  uint16   j;

  if ( (NULL == s_pStrip) || (NULL == s_pPixels) )
  {
    logPrintln(FLASH("***ERROR: Pixel strip object wasn't created"));
    return;
  }

  if (millis() < resumeTime)
  {
    return;
  }
  resumeTime = millis() + s_FxDelay;

  j = *iterCount;
  pixels = &s_pPixels[0];
  for (i = 0; i < s_NumPixels; i++)
  {
    /* Rainbow effect */
    //colour = ColourWheel((i+j) & 0xFF);

    /* Rainbow cycle effect (colour spectrum spread evently across pixels */
    colour = ColourWheel(((i * 256 / s_NumPixels) + j) & 0xFF);
    
    pixels[PIX_OFFS_RED]   = (uint8)((colour & 0x00FF0000) >> 16);
    pixels[PIX_OFFS_GREEN] = (uint8)((colour & 0x0000FF00) >>  8);
    pixels[PIX_OFFS_BLUE]  = (uint8)((colour & 0x000000FF) >>  0);
    pixels += LEDS_PER_PIX;
  }
  s_pStrip->show();

  j++;
  if (j >= 256)
  {
    j = 0;
  }
  *iterCount = j;
}

/******************************************************************************
 * Function: OutputChannels
 * 
 * Output current channel values based on the current test outputs.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Inputs/Ouputs:
 *   s_ChanValues:I
 *   s_NumChanValues:I
 *   s_NumPixels:I
 *   s_pPixels:O
 *   s_pStrip:IO
 *   s_TestOutputs:I
 *   s_TestStateI:
 *   s_TestType:I
 ******************************************************************************/
void OutputChannels(void)
{
  static TestState_t lastTestState = STOPPED;
  static uint32      dmxwUpdateTime = 0;
  static uint16      rainbowState = 0;
  AckCode_t ackCode;

  if (lastTestState == s_TestState)
  {
    if (STOPPED == s_TestState)
    {
      // There's nothing to do: we've stopped testing.
      return;
    }
  }
  else
  {
    // If we were running a test and have just stopped, perform the final
    // output to turn off all channels.
    
    if (PAUSED == s_TestState)
    {
      // Since we're currently paused:
      //   - Don't update the outputs
      //   - Just return and forget about the current test state.
      return;
    }
  }
  
  switch (s_TestOutputs)
  {
    case ONBOARD:
      // Onboard channels are updated differently while running a pixel test.
      if (PIXEL == s_TestType)
      {
        if (s_TestState != STOPPED)
        {
          // Update the effect on the LED pixel string that's connected to
          // the onboard digital channel.

          // We the test first starts running, we need to create a pixel
          // string object.
          if (lastTestState != RUNNING)
          {
            s_pStrip = new Adafruit_NeoPixel( s_NumPixels,
                                              CHAN5_PWM_PIN,
                                              NEO_KHZ400 );
            s_pStrip->begin();
            s_pPixels = s_pStrip->getPixels();
            rainbowState = 0;
          }
          
          RainbowFxIter(&rainbowState);
        }
        else
        {
          // Make one last update. Then turn off the pixel string and delete
          // the pixel string object.
          RainbowFxIter(&rainbowState);
          for (uint8 i = 0; i < s_pStrip->numPixels(); i++)
          {
            s_pStrip->setPixelColor(i, 0, 0, 0);
          }
          s_pStrip->show();
          logPrint(FLASH("Deleting pixel strip: "));
          logPrintln(s_pStrip->numPixels());
          delete s_pStrip;
          s_pPixels = NULL;
        }
      }
      else
      {
        // Straightforward update to all onboard channels.
        for (uint8 i = 1; i < s_NumChanValues; i++)
        {
          analogWrite(kChanToPinMap[i], s_ChanValues[i]);
        }
        digitalWrite( kChanToPinMap[s_NumChanValues],
                      s_ChanValues[s_NumChanValues] );
      }
      break;

    case DMXW:
      if ( (millis() >= dmxwUpdateTime) || ( STOPPED == s_TestState) )
      {
        uint8 *pDmxwBuf;
        uint8 *pChanVals;
        
        dmxwUpdateTime = millis() + DMXW_TX_DELAY;
        pDmxwBuf = &s_DmxwBuf[0];
        *(pDmxwBuf++) = kMyNodeId;
        *(pDmxwBuf++) = BROADCASTID;
        *(pDmxwBuf++) = CMD_RUN;
        if (s_TestState != STOPPED)
        {
          // Normal channel update
          pChanVals = &s_ChanValues[1];
          for (uint8 i = MAX_DMXW_CHANS; i != 0; i--)
          {
            *(pDmxwBuf++) = *(pChanVals++);
          }
        }
        else
        {
          // We're on the final update. Turn off all channels.
          memset(pDmxwBuf, 0, MAX_DMXW_CHANS);
        }
        ackCode = SendDmxwPacket( BROADCASTID,
                                  s_DmxwBuf,
                                  DMXW_MAX_BUF_LEN,
                                  false );
        dbgPrintln(FLASH("DMXW sent"));
        s_ChanValues[0] = CMD_UNDEF;
        //if (ackCode != ACK_OK)
        //{
        //  dbgPrint(FLASH("NACK: "));
        //  dbgPrintln(ackCode);
        //}
      }
      break;

    default:
      // Don't refresh any outputs
      return;
  }
  lastTestState = s_TestState;
}

/******************************************************************************
 * Function: UpdateChannels
 * 
 * Update the active test channel values based on the current test type that's
 * being run.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Inputs/Ouputs:
 *   s_ChanValues:I
 *   s_NumTestChannels:I
 *   s_TestChannels:I
 *   s_TestOutputs:I
 *   s_TestState:I
 *   s_TestType:I
 *   s_FxDelay:I
 ******************************************************************************/
void UpdateChannels(void)
{
  const uint32 k_SweepDelay = DMXW_TX_DELAY
  
  ;  // Delay in milliseconds
  const uint8  k_SweepIncr = 8;    // Channel value increments for sweeps
  static TestState_t lastTestState = STOPPED;
  static TestType_t  lastTestType = DISABLED;
  static uint32 sweepUpdTime = 0;
  static int16  currSweepVal   = MIN_CHANNEL_VALUE;
  static uint8  currActiveIdx  = 0; // Current index into s_TestChannels[]
  static uint8  currActiveChan = CHAN_UNDEFINED;
  static int8   currSweepDir   = 1;   // 1 = increasing values; -1 = decreasing

  // First check if we've just stopped a test.
  if (lastTestState != s_TestState)
  {
    lastTestType = DISABLED;
    if (STOPPED == s_TestState)
    {
      // Zero out all channels. The final output update should turn everything
      // off.
      for (uint8 i = 1; i <= MAX_DMXW_CHANS; i++)
      {
        s_ChanValues[i] = 0;
      }
    }
  }
  lastTestState = s_TestState;
  
  if (RUNNING != s_TestState)
  {
    return;
  }

  switch (s_TestType)
  {
    case CHAN_SWEEP:
      if (lastTestType != s_TestType)
      {
        // Initialize variables for sweep testing.
        currSweepDir   = 1;
        currSweepVal   = MIN_CHANNEL_VALUE;
        currActiveIdx  = 0;
        currActiveChan = s_TestChannels[currActiveIdx];
        for (uint8 i = 0; i < s_NumChanValues; i++)
        {
          s_ChanValues[i] = MIN_CHANNEL_VALUE;
        }
      }
      else
      {
        if (millis() >= sweepUpdTime)
        {
          sweepUpdTime = millis() + k_SweepDelay;
          if (1 == currSweepDir)
          {
            // Continue fading up on the current test channel
            if (currSweepVal < MAX_CHANNEL_VALUE)
            {
              currSweepVal += k_SweepIncr;
              if (currSweepVal > MAX_CHANNEL_VALUE)
              {
                currSweepVal = MAX_CHANNEL_VALUE;
              }
            }
            else
            {
              // Time to start fading down
              currSweepDir = -currSweepDir;
              currSweepVal -= k_SweepIncr;
            }
          }
          else
          {
            // Continue fading down on the current test channel
            if (currSweepVal > MIN_CHANNEL_VALUE)
            {
              currSweepVal -= k_SweepIncr;
              if (currSweepVal < MIN_CHANNEL_VALUE)
              {
                currSweepVal = MIN_CHANNEL_VALUE;
              }
            }
            else
            {
              // Time to switch to the next test channel.
              currSweepDir = -currSweepDir; // Fade up on next channel
              currSweepVal += k_SweepIncr;  // Starting fading up
              currActiveIdx++;
              if (currActiveIdx >= s_NumTestChannels)
              {
                // Wraparound back to the first test channel
                currActiveIdx = 0;
              }
              currActiveChan = s_TestChannels[currActiveIdx];
            }
          }
          // Set the new value for the currently active test channel
          s_ChanValues[currActiveChan] = currSweepVal;
        }
      }
      break;

    case PIXEL:
      // The only thing to update is the speed
      if (ONBOARD == s_TestOutputs)
      {
        s_FxDelay = s_ChanValues[PIXEL_DELAY_CHAN];
        if (s_FxDelay < PIX_UPDATE_LOWER)
        {
          s_FxDelay = PIX_UPDATE_LOWER;
        }
      }
      break;

    case MANUAL:
    case DISABLED:
    default:
      // Nothing to be done.
      return;
  }
  lastTestType = s_TestType;
}


/***************************************************************************
 * Function: ModifyTestChans
 * 
 * Add or delete a test channel from the set of channels to be tested
 * 
 * Parameters:
 *  chanNum:I    - The onboard or DMXW channel number of interest.
 *  operation:I  - The modification operation:
 *                   SELECT_ADD - Add the channel, if not already in the test
 *                                set.
 *                   SELECT_DEL - Delete the channel from the set, if it
 *                                is a member.
 * Returns:    (none)
 * Input/Output:
 *   s_NumTestChannels:IO
 *   s_TestChannels:IO
 ***************************************************************************/
void ModifyTestChans(uint8 chanNum, uint8 operation)
{
  boolean found;  // Insertion or deletion point has been found
  uint8 foundChanNum;
  uint8 i, j;

  found = false;
  for (i = 0; i < s_NumTestChannels; i++)
  {
    if (s_TestChannels[i] >= chanNum)
    {
      found = true;
      foundChanNum = s_TestChannels[i];
      break;
    }
  }

  if (found)
  {
    // The channel number at index i is either chanNum or the lowest
    // channel number that's greater than chanNum.
    if (SELECT_DEL == operation)
    {
      if (foundChanNum == chanNum)
      {
        // Shift the remaining channels downward
        for (j = i; j < s_NumTestChannels; j++)
        {
          s_TestChannels[j] = s_TestChannels[j+1];
        }
        s_NumTestChannels--;
      }
    }
    else
    {
      if (foundChanNum != chanNum)
      {
        // Shift the remaining channels upward and insert chanNum
        for (j = s_NumTestChannels; j != i; j--)
        {
          s_TestChannels[j] = s_TestChannels[j-1];
        }
        s_TestChannels[i] = chanNum;
        s_NumTestChannels++;
      }
    }
  }
  else
  {
    // The channel doesn't exist in the list
    if (SELECT_ADD == operation)
    {
      // Add the channel as long as there's still room for it.
      if (s_NumTestChannels <= sizeof(s_TestChannels))
      {
        s_TestChannels[s_NumTestChannels++] = chanNum;
      }
      else
      {
        logPrint(FLASH("***No room to add test channel ["));
        logPrint(chanNum);
        logPrintln(FLASH("]"));
      }
    }
  }
}


/***************************************************************************
 * Function: UpdateChanValue
 * 
 * Set a given test channel to a specified value.
 * 
 * Parameters:
 *  chanNum:I  - The onboard or DMXW channel number of interest.
 *  value:I    - The value to which to set the channel.
 * Returns: true iff chanNum is a member of the current test set. (Refer to
 *          ModifyTestChans().)
 * Input/Output:
 *   s_ChanValues:O
 *   s_NumTestChannels:I
 *   s_TestChannels:I
 ***************************************************************************/
uint8 UpdateChanValue(uint8 chanNum, uint8 value)
{
  uint8 *pTestChans = s_TestChannels;
  for (uint8 i = 0; i < s_NumTestChannels; i++)
  {
    if (*(pTestChans++) == chanNum )
    {
      
      s_ChanValues[chanNum] = value;
      dbgPrint(FLASH("Value: "));
      logPrint(chanNum);
      logPrint(FLASH(" = "));
      logPrintln(value);
      return true;
    }
  }
  dbgPrint(FLASH("Fail Update: "));
  dbgPrint(chanNum);
  dbgPrint(FLASH(" = "));
  dbgPrintln(value);
  return false;
}


/***************************************************************************
 * Function: SendIpcAck
 * 
 * Send an acknowledgement to an IPC command.
 * 
 * Parameters:
 *  ipcCmd:I      - The IPC command to which this is an acknowledgement.
 *  returnCode:I  - The acknowledgement return code value.
 * Returns:      (none)
 * Input/Output:
 *  s_IpcSeqNum:I
 *  s_HmiComms:O
 ***************************************************************************/
void SendIpcAck(uint8 ipcCmd, uint8 returnCode)
{
  uint8 *pBuf;
  uint8 tmpBuf[4];
  uint8 len;
  uint8 crc8;

  len = 0;
  tmpBuf[len++] = TSTCMD_ACK;
  tmpBuf[len++] = ipcCmd;
  if (returnCode >= CHAR_ESC)
  {
    tmpBuf[len++] = CHAR_ESC;
  }
  tmpBuf[len++] = returnCode;
  crc8 = Crc8(tmpBuf, len);
  
  s_HmiComms.write(CHAR_STX);
  s_HmiComms.write(CHAR_STX);
  s_HmiComms.write(s_IpcSeqNum);
  pBuf = tmpBuf;
  for (uint8 i = len; i != 0; i--)
  {
    s_HmiComms.write(*pBuf);
    pBuf++;
  }
  s_HmiComms.write(CHAR_ETX);
  s_HmiComms.write(crc8);
  dbgPrint(millis());
  dbgPrint(FLASH(" "));
  dbgPrint(FLASH("ACK["));
  dbgPrint(ipcCmd);
  dbgPrint(FLASH(", "));
  dbgPrint(returnCode);
  dbgPrint(FLASH(", "));
  dbgPrint(crc8);
  dbgPrintln(FLASH("]"));
}


/***************************************************************************
 * Function: ProcessIpcCommand
 * 
 * Parse and execute the command in the IPC serial buffer.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Input/Output:
 *   s_IpcBuf:IO
 *   s_IpcBufPos:IO
 *   s_Rebooted:IO
 *   s_TestOutputs:IO
 *   s_TestState:IO
 *   s_TestType:IO
 ***************************************************************************/
void ProcessIpcCommand(void)
{
  uint8 idx;
  uint8 tmp1, tmp2, tmp3;
  uint8 rc;
  uint8 cmd;
  
  if (0 == s_IpcBufPos)
  {
    // There's nothing to process.
    dbgPrintln(FLASH("***ProcessIpcCommand() called with empty buffer"));
    return;
  }

  idx = 0;
  rc = TSTACK_OK;
  cmd = s_IpcBuf[idx++];
  dbgPrint(millis());
  dbgPrint(FLASH(" "));
  switch (cmd)
  {
    case TSTCMD_INIT:
      // TSTCMD_INIT()
      dbgPrint(FLASH("INIT: "));
      s_Rebooted = false;
      break;
      
    case TSTCMD_STATE:
      // TSTCMD_STATE(state)
      dbgPrint(FLASH("STATE: "));
      if (s_IpcBufPos != 2)
      {
        rc = TSTACK_BAD_PARM;
      }
      else
      {
        tmp1 = s_IpcBuf[idx++];
        dbgPrintln(tmp1);
        if (tmp1 > RUNNING)
        {
          rc = TSTACK_BAD_PARM;
        }
        else
        {
          s_TestState = (TestState_t)tmp1;
          if (RUNNING == s_TestState)
          {
            s_NumChanValues = ( s_TestOutputs == DMXW ?
                                NUM_CHAN_DMXW : NUM_CHANS_ONBOARD );
          }
        }
      }
      break;

    case TSTCMD_TEST:
      // TSTCMD_TEST(output, type)
      dbgPrint(FLASH("TEST: "));
      if (s_IpcBufPos != 3)
      {
        rc = TSTACK_BAD_PARM;
      }
      else if (s_TestState != STOPPED)
      {
        rc = TSTACK_BAD_STATE;
      }
      else
      {
        tmp1 = s_IpcBuf[idx++];
        tmp2 = s_IpcBuf[idx++];
        dbgPrint(tmp1);
        dbgPrint(FLASH(", "));
        dbgPrintln(tmp2);
        if ( (tmp1 <= DMXW) && (tmp2 <= PIXEL) )
        {
          s_TestOutputs = (TestOutput_t)tmp1;
          s_TestType    = (TestType_t)tmp2;
        }
        else
        {
          rc = TSTACK_BAD_PARM;
        }
      }
      break;

    case TSTCMD_SELECT:
      // TSTCMD_SELECT(selectCmd, numChans, channelList...)
      dbgPrint(FLASH("SELECT: "));
      if (s_TestState != STOPPED)
      {
        rc = TSTACK_BAD_STATE;
      }
      else
      {
        if (s_IpcBufPos < 3)
        {
          rc = TSTACK_BAD_PARM;
        }
        else
        {
          tmp1 = s_IpcBuf[idx++];
          tmp2 = s_IpcBuf[idx++];
          dbgPrint(tmp1);
          dbgPrint(FLASH(", "));
          dbgPrintln(tmp2);
          if (s_IpcBufPos != (tmp2 + 3))
          {
            rc = TSTACK_BAD_PARM;
          }
          else
          {
            switch (tmp1) // tmp1 is selectCmd
            {
              case SELECT_ADD:
              case SELECT_DEL:
                for (uint8 i = 1; i <= tmp2; i++)
                {
                  ModifyTestChans(s_IpcBuf[idx++], tmp1);
                }
                break;
              case SELECT_CLR:
                s_NumTestChannels = 0;
                break;
              default:
                rc = TSTACK_BAD_PARM;
            }
          }
        }
      }
      break;

    case TSTCMD_VALUE:
      // TSTCMD_VALUE(numChans, chanValList...)
      dbgPrintln(FLASH("VALUE:"));
      if (s_IpcBufPos < 4)
      {
        rc = TSTACK_BAD_PARM;
      }
      else
      {
        tmp1 = s_IpcBuf[idx++];
        dbgPrintln(tmp1);
        if ( (0 == tmp1) || (s_IpcBufPos != (2 * tmp1 + 2)) )
        {
          rc = TSTACK_BAD_PARM;
        }
        else
        {
          uint8 *pChanData = &s_IpcBuf[idx];
          
          for (uint8 i = tmp1; i != 0; i--)
          {
            // Read in the next (chan #, value) pair.
            tmp2 = *(pChanData++);
            tmp3 = *(pChanData++);
            if (!UpdateChanValue(tmp2, tmp3))
            {
              rc = TSTACK_BAD_PARM;
              break;
            }
          }
        }
      }
      break;

    case TSTCMD_PIXCFG:
      // TSTCMD_PIXCFG(length)
      dbgPrintln(FLASH("PIXCFG: "));
      if (s_IpcBufPos != 3)
      {
        rc = TSTACK_BAD_PARM;
      }
      else
      {
        tmp1 = s_IpcBuf[idx++];
        tmp2 = s_IpcBuf[idx++];
        s_NumPixels = ((uint16)tmp1 << 8) | (uint16)tmp2;
        dbgPrintln(s_NumPixels);
      }
      break;

    default:
      rc = TSTACK_CORRUPTED;
      logPrint(FLASH("***BAD IPC cmd ["));
      logPrint(s_IpcBuf[0]);
      logPrintln(FLASH("]"));
  }
  if (s_Rebooted)
  {
    rc = TSTACK_OUTREBOOT;
  }
  SendIpcAck(cmd, rc);
}

 
/***************************************************************************
 * Function: HandleIpcRx
 * 
 * Check for, and respond to, messages coming in from the HMI processor
 * over the Interprocessor Communcations serial port.
 * 
 * Parameters: (none)
 * Returns:    (none)
 * Input/Output:
 *   s_IpcBuf:IO
 *   s_IpcBufPos:IO
 *   s_IpcSeqNum:IO
 * Note:
 *   This function processes the serial input in stream fashion, processing
 *   as much of the input at-a-time as it can, executing at most one
 *   received command.
 ***************************************************************************/
void HandleIpcRx(void)
{
  static uint8 state = IPC_SEEK;
  boolean done;
  uint8   inChar;
  uint8   charCount;
  uint8   rxCrc8;
  uint8   calcCrc8;

  done = false;
  charCount = s_HmiComms.available();

  /* If the serial buffer has overflowed, purge the buffer
   * and abort further processing.
   */
  if (s_HmiComms.overflow())
  {
    dbgPrintln(FLASH("IPC serial overflow"));
    for (uint8 i = charCount; i != 0; i++)
    {
      inChar = s_HmiComms.read();
    }
    return;
  }
  
  while ((charCount != 0) && !done)
  {
    charCount--;
    inChar  = s_HmiComms.read();
    dbgPrint("@");
    dbgPrint(state);
    dbgPrint(", ");
    dbgPrintln(inChar);
    switch (state)
    {
      case IPC_SEEK:
        s_IpcBufPos = 0;
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
        s_IpcSeqNum = inChar;
        state = IPC_COLLECT;
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
          if (s_IpcBufPos >= sizeof(s_IpcBuf))
          {
            // No matter what, we're going to overflow the buffer
            dbgPrint(FLASH("***Buffer overflow"));
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
              s_IpcBuf[s_IpcBufPos++] = inChar;
            }
          }
        }
        break;

      case IPC_ESC:
        s_IpcBuf[s_IpcBufPos++] = inChar;
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
    // We now have a command to be processed.
    calcCrc8 = Crc8(s_IpcBuf, s_IpcBufPos);
    dbgPrint(FLASH("CRC: Rx="));
    dbgPrint(rxCrc8);
    dbgPrint(FLASH(", Calc="));
    dbgPrintln(calcCrc8);
    if (calcCrc8 == rxCrc8)
    {
      // The CRC8 value checks out: Execute the command.
      ProcessIpcCommand();
    }
    else
    {
      dbgPrintln(FLASH("***Bad cmd CRC8"));
    }
  }
}

/***************************************************************************
 * Function: CheckRam
 * 
 * Test how much RAM is left on the MPU, outputing the results out to
 * the serial port.
 * 
 * Parameters: (none)
 * Returns:    (none)
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
  Serial.begin(SERIAL_BAUD);  // Open up the console serial ports
  delay(10);
  s_Radio.initialize(FREQUENCY, kMyNodeId, NETWORKID);
  s_Radio.encrypt(ENCRYPTKEY);
  s_Radio.promiscuous(kPromiscuousMode);
  
  pinMode(CHAN1_PWM_PIN, OUTPUT);
  pinMode(CHAN2_PWM_PIN, OUTPUT);
  pinMode(CHAN3_PWM_PIN, OUTPUT);
  pinMode(CHAN4_PWM_PIN, OUTPUT);
  pinMode(CHAN5_PWM_PIN, OUTPUT);

  // Check if we need to clear out EEPROM and start from scratch
  s_ResetEeprom = (EEPROM.read(EEPROM_FW_ADDR) != FW_VERSION_c);
  if (s_ResetEeprom)
  {
    // Record new F/W version and start with blank mappings
    EEPROM.write(EEPROM_FW_ADDR, FW_VERSION_c);
  }
  else
  {
    // Read in mappings stored in EEPROM
    EepromLoad();
  }

  s_NumTestChannels = 0;
  s_NumChanValues = 0;
  for (uint8 i = 0; i <= MAX_DMXW_CHANS; i++)
  {
    s_ChanValues[i] = 0;
  }

  // Write copyright banner to the console
  logPrintln();
  logPrintln();
  for (uint8 i = 1; i <= 80; i++)
    logPrint(FLASH("#"));
  logPrintln();
  logPrint(FLASH("DMXW Tester (Output Controller).  (DMXW node #:"));
  logPrint(kMyNodeId);
  logPrint(FLASH("   Radio frequency: "));
  logPrint(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  logPrintln(FLASH("Mhz)"));
  logPrint(FLASH("S/W: "));
  logPrint(SW_VERSION_c);
  logPrint(FLASH("\tF/W: "));
  logPrint(FW_VERSION_c);
  logPrint(FLASH("\t"));
  logPrintln(COPYRIGHT);
  
  CheckRam();
  s_HmiComms.begin(IPC_COMMS_BAUD);  // Open up HMI serial comms ports

  s_TestState   = STOPPED;
  s_TestType    = DISABLED;
  s_TestOutputs = NO_OUTPUT;
}

//------------------------------------------------------------------
void loop()
{
  HandleIpcRx();
  if (!s_Rebooted)
  {
    UpdateChannels();
    OutputChannels();
  }
}
