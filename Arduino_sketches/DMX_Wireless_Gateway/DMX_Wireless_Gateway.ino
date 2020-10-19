/******************************************************************************
 * DMX_Wireless_Gateway
 *
 * This sketches implements a DMX wireless gateway controller that controls
 * nodes containing RFM69 radio transceivers. A DMX controller has a wired
 * connection to the gateway unit. This gateway can be configured to
 * respond to up to 20 DMX channels and can be configured to communicate
 * with up to 20 wireless RFM69 client nodes. For each configured DMX channel
 * the gateway relays channel values to the appropriate (node, port) pair.
 * Each RFM69 client node has available up to 10 local digital output port, of
 * which up to 6 could support PWM (psuedo analog) output.
 *
 * Each client also has available up to 10 digital input ports and up to 6
 * analog input ports (which are independent of the digital input ports).
 * Input values can be transmitted back to the gateway for monitoring purposes.
 *
 * NOTES on channel, node, and port numbering
 *   - DMX Wireless Gateway numbers DMX channels from 1 - 512. (An internal
 *     channel number of 0 indicates 'undefined/unassigned'.)
 *   - DMX wirless network gateway and nodes are numbered from 1 - 21 (and 255)
 *     with the gateway being node #1. Messages sent to node #255 are broadcasts.
 *   - Port numbers on nodes are numbered 1 - 10 (for digital I/O) and
 *     11 - 16 (for psuedo analog outputs (PWM) and analog inputs).
 *   - terminal emulator (e.g. Linum minicom) settings for console I/O:
 *         - 9600 baud 8 data bits, no parity, 1 stop bit
 *         - enable local echo
 *         - no hardware flow control; software flow control
 *
 * The gateway is also capable of local console control of the 20
 * (node, port) outputs via:
 *    - a single 2-axis analog joystick
 *    - 2 potentiometers
 *    - 4 pushbuttons
 * enabling local control of up to 4 analog (node, port) pairs and
 * 4 digital (node, port) pairs.
 *
 * DMX control and local control are mutually exclusive. There is a switch
 * on the gateway for selecting between DMX and console control.
 *
 * Notes:
 * =====
 *   Special EEPROM memory locations:
 *      Address   Description
 *      -------   ----------------------------------
 *         0      Firmware Version
 *         1      Number of dmxMap[] entries stored.
 *
 *   *** Need to import <SoftwareSerial.h> and translate all print statements
 *       from Serial.print...   to mySerial.print... as the built-in Serial
 *       library is incompatible with the DMXSerial library. This also implies
 *       that console I/O will have to go thru a separate serial port. (Need
 *       to reserve a couple of digital pins for that.)
 *       The DMX_Wireless_Node is not affected because it doesn't use
 *       the DMXSerial library.
 *
 * Console Controls:
 * ================
 *   Control              Pin
 *   ----------------     ---
 *   Joystick x-axis      A0
 *   Joystick y-axis      A1
 *   Potentiometers       A2, A3
 *   Pushbuttons          D3, D4, D5, D6
 *
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 0.1      13-11-09 J.van Schouwen Initial creation.
 * 0.2      13-12-01 J.van Schouwen Initial prototype that is able to
 *                                   distribute data to 20-60 DMXW wireless
 *                                   nodes, plus console controls. It doesn't
 *                                   yet actually handle DMX-512 input.
 * 1.0      14-03-06 J.van Schouwen First release.
 * 1.1      15-11-02 J.van Schouwen Second (minor) release. (No firmware
 *                                   up-version.) Fix bug in command line
 *                                   editing. Fix display of version info.
 *          20-03-30 J.van Schouwen (Release_2 (prototype v1.1) re-released as
 *                                   Release_3 to maintain a complete release
 *                                   lineup.)
 *
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <SoftwareSerial.h>
#include <DMXSerial.h>
#include <RFM69_DMX.h>
#include <SPI.h>
#include "DMXWNet.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define COPYRIGHT       "(C)2015, A.J. van Schouwen"
#define SW_VERSION_c    "1.1 (2015-11-02)"
#define FW_VERSION_c    9   // Increment (with wraparound) for new F/W;
                            //   clears EEPROM.

//#define DEBUG_ON            // Uncomment to turn off debug output to serial port.
#define LOGGING_ON          // Uncomment to turn off packet logging to serial port.

#define PIN_LOCATE      9   // Pin number of digital port connected to
                            // onboard LED (for location purposes)
#define MAX_SERIAL_BUF_LEN  20
#define EEPROM_FW_ADDR             0
#define EEPROM_NUMDMXNODES_ADDR    1
#define EEPROM_FIRST_OPEN_ADDR     2
#define INVALID_MAP_INDEX  (MAX_DMXW_CHANS + 1)

#define SERIAL_BAUD            9600

#define DEL_CHAR               0x7F  // ASCII Del character

#define NUM_BUTTONS            5  // Includes joystick button
#define NUM_POTS               2

/*** Digital input pins ***/
// Console buttons
#define BUTTON1_PIN            6  // Button #1
#define BUTTON2_PIN            9  // Button #2
#define BUTTON3_PIN           14  // Button #3
#define BUTTON4_PIN           15  // Button #4
#define BUTTON5_PIN           16  // Button #5 (joystick button)

#define CONFIG_ENABLED_PIN     4
#define CONSOLE_ENABLED_PIN   17

/*** Digital output pins ***/
#define DMXW_ACTIVITY_LED      3
#define DMX512_ACTIVITY_LED    5

/*** Analog input pins ***/
// Console knobs
#define JOY_X_PIN              5  // Joystick X axis
#define JOY_Y_PIN              4  // Joystick Y axis
#define POT1_PIN               6  // Potentiometer #1
#define POT2_PIN               7  // Potentiometer #2

#define CONFIG_RX              7  // Console Serial RX port
#define CONFIG_TX              8  // Console Serial TX port


// A 45 ms TX delay equates to a 22 Hz DMXW refresh rate. A DMX-512
// network running the full 512 channels has a refresh rate of 44Hz.
// So we're running at half the slowest refresh rate.
// (Note that a DMX-512 network running fewer channels can refresh
// much more quickly. The minimum frame time is 1196 microsecs (or 836 Hz),
// which can be achieved with at most 24 channel slots.)
#define DMXW_TX_DELAY            35  // milliseconds
#define DMX_SUSPEND_DURATION   2000  // milliseconds

#define DMXW_TEST_MODE            1


// Macro for defining strings that are stored in flash (program) memory rather
// than in RAM. Arduino defines the non-descript F("string") syntax.
#define FLASH(x) F(x)

#ifdef DEBUG_ON
  #define dbgPrint(x)    consSerial.print(x)
  #define dbgPrintln(x)  consSerial.println(x)
  #define DEBUGGING  1
#else
  #define dbgPrint(x)
  #define dbgPrintln(x)
  #define DEBUGGING  0
#endif

#ifdef LOGGING_ON
  #define logPrint(x)    consSerial.print(x)
  #define logPrintln(x)  consSerial.println(x)
#else
  #define logPrint(x)
  #define logPrintln(x)
#endif

const Uint8 myNodeId = GATEWAYID; // Must be unique for each node 
// Console I/O serial port.
SoftwareSerial consSerial(CONFIG_RX, CONFIG_TX);
RFM69 radio;
bool promiscuousMode = true;  // sniff all packets on network iff true
Uint8 dstNodeId;
Uint8 srcNodeId;

bool initialized = false;
bool handleInput = false;
bool ackRequested = false;
bool requestAck = true;
bool dataToSend = false;
bool waitForReply = false;
bool deferDmx512 = false;
bool dmx512Suspended = true;
Uint8 quiteMode = 0;  // Set to non-zero for quiet output from "n" serial cmd

Uint8 buffer[MAX_DATA_LEN];
Uint8 bufSize = 0;
Uint8 currReadPos = 0;

char  serialBuffer[MAX_SERIAL_BUF_LEN];
Uint8 serialPos = 0;

bool    resetEeprom = false;
Uint8   command = 0;
Uint8   cmdInProgress = CMD_UNDEF; // Multi-cycle command when not CMD_UNDEF
Int8    iteration = -1;
Uint8   ackBuf[1];
bool    saveNodes = false;

// DMXW Channel Test Mode variables
Uint8   testMode = 0;      // DMXW channel test mode
                           //   0 = no test
                           //   DMXW_TEST_MODE = test)
Uint16  testParam = 0;     // Test parameter (DMX-512 channel #, or
                           //                 0 for all channels)
int     testIdx = -1;      // Index of current DMXW channel being tested
int     testValue = 0;     // Current value for channel under test
Uint16  testSpeed = 0;     // Speed at which test value changes (0 is fastest)
Uint8   testIterations = 0;

unsigned long  rxTime = 0;
unsigned long  ackTime = 0;
unsigned long  suspendStartTime = 0;
unsigned long  dmxwTxTime = 0;
unsigned long  cmdTimeout = 0;

bool dmxTimingStart = false;
unsigned long  transmissionCountdown = 0;
unsigned long  dmxwTimingStart = 0;
long  txCount = 0;
float dmxwFrequency;

DmxwGwMapRecord_t  dmxMap[MAX_DMXW_CHANS];
Uint8  numDmxwChans = 0;
DmxwGwMapRecord_t  tmpMapRecord;

Uint8  nodeList[MAX_DMXW_CHANS];
Uint8  numNodes = 0;

typedef struct buttonData_t {
  Int8    dmxwChan;
  Uint8   pin;
  Uint8   value;
} ButtonData_t;
ButtonData_t  buttonMap[NUM_BUTTONS];

typedef struct potentiometerData_t {
  Int8    dmxwChan;
  Uint8   pin;
  Uint8   value;
} PotentiometerData_t;
PotentiometerData_t  potMap[NUM_POTS];

typedef struct joystickData_t {
  Int8    dmxwChan_x;
  Int8    dmxwChan_y;
  Uint8   pin_x;
  Uint8   pin_y;
  Uint8   x_axis;
  Uint8   y_axis;
} JoystickData_t;
JoystickData_t joystick;


Uint8   node;
bool    configEnabled = false;
bool    oldConfigEnabled = true;
bool    consoleEnabled = false;
bool    dmx512Running = true;
Uint8   dmxwActLedValue = 0;
Int8    dmxwActLedDir = 1;
Uint8   dmx512ActLedValue = 0;
Int8    dmx512ActLedDir = 1;
unsigned long loopCount = 0;


//=========================================================================

void EepromLoad()
{
  int addr;

  numDmxwChans = EEPROM.read(EEPROM_NUMDMXNODES_ADDR);
  if (numDmxwChans == 0)
    return;
    
  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < numDmxwChans; i++)
  {
    dmxMap[i].dmxwChan    = EEPROM.read(addr++);
    dmxMap[i].dmx512Chan  = ((Uint16)(EEPROM.read(addr++))) << 8;
    dmxMap[i].dmx512Chan |= (Uint16)EEPROM.read(addr++);
    dmxMap[i].nodeId      = EEPROM.read(addr++);
    dmxMap[i].port        = EEPROM.read(addr++);
    dmxMap[i].logarithmic = EEPROM.read(addr++);
    //dmxMap[i].value       = EEPROM.read(addr++); // Don't store the value
    dmxMap[i].value       = 0;
  }
  for (Uint8 i = 0; i < NUM_BUTTONS; i++)
    buttonMap[i].dmxwChan = EEPROM.read(addr++);
  for (Uint8 i = 0; i < NUM_POTS; i++)
    potMap[i].dmxwChan    = EEPROM.read(addr++);
  joystick.dmxwChan_x     = EEPROM.read(addr++);
  joystick.dmxwChan_y     = EEPROM.read(addr++);
}

void EepromSave()
{
  int addr;

  EEPROM.write(EEPROM_NUMDMXNODES_ADDR, numDmxwChans);
  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < numDmxwChans; i++)
  {
    EEPROM.write(addr++, dmxMap[i].dmxwChan);
    EEPROM.write(addr++, (byte)(dmxMap[i].dmx512Chan >> 8));
    EEPROM.write(addr++, (byte)(dmxMap[i].dmx512Chan & 0x00ff));
    EEPROM.write(addr++, dmxMap[i].nodeId);
    EEPROM.write(addr++, dmxMap[i].port);
    EEPROM.write(addr++, dmxMap[i].logarithmic);
    //EEPROM.write(addr++, dmxMap[i].value); // Don't store the value
  }
  for (Uint8 i = 0; i < NUM_BUTTONS; i++)
    EEPROM.write(addr++, buttonMap[i].dmxwChan);
  for (Uint8 i = 0; i < NUM_POTS; i++)
    EEPROM.write(addr++, potMap[i].dmxwChan);
  EEPROM.write(addr++, joystick.dmxwChan_x);
  EEPROM.write(addr++, joystick.dmxwChan_y);
}

AckCode_t handleCmdPing()
{
  node = srcNodeId;
  buffer[bufSize++] = CMD_PONG;
  dataToSend = true;
  return ACK_OK;
}

AckCode_t handleCmdPong()
{
  logPrint(FLASH("Pong received from node #"));
  logPrintln(srcNodeId);
  return ACK_OK;
}

AckCode_t handleCmdChan()
{
  char   logTxt[81];
  Int8   dmxwChan     = buffer[currReadPos++];
  Int8   port         = buffer[currReadPos++];
  Int8   outPin       = buffer[currReadPos++];
  Int8   conflictPort = buffer[currReadPos++];
  Uint8  isAnalog     = buffer[currReadPos++];
  Uint8  value        = buffer[currReadPos++];
  Uint8  logarithmic  = buffer[currReadPos++];

  if (port == -1)
    sprintf(logTxt, "DmxwChan:%3d, Node:%2d   *** Not Mapped!",
            dmxwChan, srcNodeId);
  else
    sprintf(logTxt, "DmxwChan:%3d, Node:%2d, Port:%2d, Log?(%1s), "
                    "OutPin:%2d, Value:%3d%s",
            dmxwChan, srcNodeId, port, logarithmic ? "Y" : "N", outPin, value,
            isAnalog ? "(Analog) " : "(Digital)");
  logPrint(logTxt);

  if (port != -1)
  {
    if (conflictPort != -1)
    {
      logPrint(FLASH("  AltPort: "));
      logPrint(conflictPort);
    }
  }
  logPrintln();

  waitForReply = false;
  if (cmdInProgress == CMD_UNDEF)
    logPrintln();
  return ACK_OK;
}

AckCode_t handleCmdUndef()
{
  // Someone failed to set their command code to a valid value.
  dbgPrintln(FLASH("CMD_UNDEF is an invalid command."));
  return ACK_ECMD;
}


// Incoming message from the wireless network of nodes
AckCode_t handleNetRxMessage(Uint8 command)
{
  AckCode_t ret = ACK_ERR;

  switch (command)
  {
    case CMD_PONG:   ret = handleCmdPong();      break;
    
    case CMD_CHAN:   ret = handleCmdChan();      break;
    
    case CMD_PING:   ret = handleCmdPing();      break;
    
    case CMD_RUN:
    case CMD_MAP:
    case CMD_MAPR:
    case CMD_CLRALL:
    case CMD_ECHO:
    case CMD_LOC:
    case CMD_OFF:
    case CMD_PORT:
    case CMD_CTRL:
    case CMD_TEST:
    case CMD_SAVE:   ret = ACK_ECMD;             break;
      
    case CMD_UNDEF:  ret =handleCmdUndef();      break;
    
    default:
      logPrint(FLASH("*** Command not recognized ["));
      logPrint(command);
      logPrintln("]");
      return ACK_ECMD;
  }
  if (ret == ACK_ECMD)
  {
    dbgPrint(FLASH("... Command unsupported for Gateway ..."));
  }
  return ret;
}


void printCommand(Uint8 command)
{
  switch (command)
  {
    case CMD_RUN:    dbgPrint(FLASH("CMD_RUN"));     break;
    case CMD_PING:   dbgPrint(FLASH("CMD_PING"));    break;
    case CMD_PONG:   dbgPrint(FLASH("CMD_PONG"));    break;
    case CMD_MAP:    dbgPrint(FLASH("CMD_MAP"));     break;
    case CMD_MAPR:   dbgPrint(FLASH("CMD_MAPR"));    break;
    case CMD_CLRALL: dbgPrint(FLASH("CMD_CLRALL"));  break;
    case CMD_ECHO:   dbgPrint(FLASH("CMD_ECHO"));    break;
    case CMD_CHAN:   dbgPrint(FLASH("CMD_CHAN"));    break;
    case CMD_LOC:    dbgPrint(FLASH("CMD_LOC"));     break;
    case CMD_OFF:    dbgPrint(FLASH("CMD_OFF"));     break;
    case CMD_PORT:   dbgPrint(FLASH("CMD_PORT"));    break;
    case CMD_CTRL:   dbgPrint(FLASH("CMD_CTRL"));    break;
    case CMD_TEST:   dbgPrint(FLASH("CMD_TEST"));    break;
    case CMD_SAVE:   dbgPrint(FLASH("CMD_SAVE"));    break;
    case CMD_UNDEF:  dbgPrint(FLASH("CMD_UNDEF"));   break;
    default:
      dbgPrint(FLASH("<Bad Cmd>["));
      dbgPrint(command);
      dbgPrint("]");
  }
}


void printAckResult(Uint8 ackResult)
{
  switch (ackResult)
  {
    case ACK_OK:      dbgPrint(FLASH("Ok"));                         break;
    case ACK_ECMD:    logPrint(FLASH("*** Command unsupported"));    break;
    case ACK_ETIME:   logPrint(FLASH("*** TX timed out"));           break;
    case ACK_EDMXW:   logPrint(FLASH("*** DMXW chan access error")); break;
    case ACK_EPORT:   logPrint(FLASH("*** Port access error"));      break;
    case ACK_ERR:     logPrint(FLASH("*** Unknown Error"));          break;
    case ACK_NULL:    logPrint(FLASH("*** Undefined ACK"));          break;
    default:
      logPrint(FLASH("<Unknown ACK code>["));
      logPrint(ackResult);
      logPrint("]");
  }
}


// Send a message to a node on the wireless network
AckCode_t sendBuffer(int dst, Uint8 *payload, int sendSize, bool requestAck)
{
  unsigned long sentTime;
    
  dbgPrint(FLASH("TX Dst["));
  dbgPrint(dst);
  dbgPrint(FLASH("] Size["));
  dbgPrint(sendSize);
  dbgPrint(FLASH("] Data["));
  printCommand(payload[0]);
  for (Uint8 i = 1; i < sendSize; i++)
  {
    dbgPrint(" ");
    dbgPrint(payload[i]);
  }
  dbgPrint("]");

  if (dst == BROADCASTID)
    requestAck = false;
  for (byte i = 0; i <= TX_NUM_RETRIES; i++)
  { 
    radio.send(dst, payload, sendSize, requestAck);
    sentTime = millis();
    if (requestAck)
    {
      while ( (millis() - sentTime) < ACK_WAIT_TIME)
      {
        if (radio.ACKReceived(dst))
        {
          //dbgPrint(" ~ms:"); dbgPrintln(millis()-sentTime);
          return radio.DATA[0];
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


//------------  dmxMap handling functions ------------------------------
// Find the index into dmxMap for a given DMX-512 channel.
Uint8 findDmxMapByDmx512(Uint16 dmx512Chan)
{
  for (Uint8 idx = 0; idx < numDmxwChans; idx++)
  {
    if (dmxMap[idx].dmxwChan == 0)
      continue;
    if (dmxMap[idx].dmx512Chan == dmx512Chan)
      return idx;
  }
  return INVALID_MAP_INDEX;
}

Uint8 findDmxMapByDmxw(Uint8 dmxwChan)
{
  for (Uint8 idx = 0; idx < numDmxwChans; idx++)
  {
    if (dmxMap[idx].dmxwChan == 0)
      continue;
    if (dmxMap[idx].dmxwChan == dmxwChan)
      return idx;
  }
  return INVALID_MAP_INDEX;
}

Uint8 findDmxMapByNodeId(Uint8 nodeId)
{
  for (Uint8 idx = 0; idx < numDmxwChans; idx++)
  {
    if (dmxMap[idx].dmxwChan == 0)
      continue;
    if (dmxMap[idx].nodeId == nodeId)
      return idx;
  }
  return INVALID_MAP_INDEX;
}

bool existsDmxMapPair(Uint8 startIdx, Uint8 nodeId, Uint8 port)
{
  if (startIdx >= numDmxwChans)
    return false;
    
  for (Uint8 idx = startIdx; idx < numDmxwChans; idx++)
  {
    if (dmxMap[idx].dmxwChan == 0)
      continue;
    if ( (dmxMap[idx].nodeId == nodeId) &&
         (dmxMap[idx].port == port) )
      return true;
  }
      
  return false;
}

void writeDmxMapRecord(Uint8 idx,Uint16 dmx512Chan, Uint8 dmxwChan,
                       Uint8 nodeId, Uint8 port, bool logarithmic)
{
  DmxwGwMapRecord_t tmp;
  
  tmp.dmxwChan    = dmxwChan;
  tmp.dmx512Chan  = dmx512Chan;
  tmp.nodeId      = nodeId;
  tmp.port        = port;
  tmp.logarithmic = logarithmic;
  tmp.value       = 0;
  dmxMap[idx] = tmp;
}

// Open an empty space at dmxMap[idx], shifting all entries past it
// upward by one index.
bool shiftUpMapRecords(Uint8 idx)
{
  if (idx == MAX_DMXW_CHANS)
    return false;

  if (idx == numDmxwChans)
  {
    numDmxwChans++;
    return true;
  }

  for (Int8 i = (numDmxwChans - 1); i >= idx; i--)
    dmxMap[i+1] = dmxMap[i];

  numDmxwChans++;
  return true;
}

// Delete the dmxMap[idx] and shift all entries after it down by one index.
bool shiftDownMapRecords(Uint8 idx)
{
  if (idx >= numDmxwChans)
    return false;
  for (Uint8 i = idx; i < (numDmxwChans - 1); i++)
    dmxMap[i] = dmxMap[i+1];
  memset(&dmxMap[numDmxwChans-1], 0, sizeof(DmxwGwMapRecord_t));
  numDmxwChans--;
  return true;
}
  
bool addDmxMap(Uint16 dmx512Chan, Uint8 dmxwChan, Uint8 nodeId, Uint8 port,
               bool logarithmic)
{
  Uint8 idx;
  Uint8 tmpChan;
  DmxwGwMapRecord_t tmp;
  
  if (numDmxwChans >= MAX_DMXW_CHANS)
  {
    logPrintln(FLASH("*** Max DMXW chans exceeded."));
    return false;
  }
  
  // Check value ranges
  if ( (dmx512Chan == 0) || (dmxwChan == 0) || (nodeId == 0) || (port == 0))
    return false;
  if ( (dmx512Chan > MAX_DMX512_CHANS) || (dmxwChan > MAX_DMXW_CHANS) ||
       (nodeId > NODEID_MAX) || (port > MAX_PORTS)  )
    return false;
  
  // Check for duplicate channel numbers or (nodeId, port) pairs
  if (findDmxMapByDmx512(dmx512Chan) != INVALID_MAP_INDEX)
    return false;
  if (findDmxMapByDmxw(dmxwChan) != INVALID_MAP_INDEX)
    return false;
  for (idx = 0; idx < numDmxwChans; idx++)
  {
    if (existsDmxMapPair(idx, nodeId, port))
      return false;
  }
  

  for (idx = 0; idx < MAX_DMXW_CHANS; idx++)
  {
    tmpChan = dmxMap[idx].dmxwChan;

    // Insert new record if the nodeId for entry i is 0 (unused) or
    // is greater than dmxwChan. (Entries are sorted on nodeId.)
    if ( (tmpChan == 0) || (tmpChan > dmxwChan))
    {
      shiftUpMapRecords(idx);
      writeDmxMapRecord(idx, dmx512Chan, dmxwChan, nodeId, port, logarithmic);
      bufSize = 0;
      buffer[bufSize++] = CMD_MAP;
      buffer[bufSize++] = dmxwChan;
      buffer[bufSize++] = port;
      buffer[bufSize++] = logarithmic;
      node = nodeId;
      dataToSend = true;
      return true;
    }
  }
  logPrintln(FLASH("*** <unexpected error> ***"));
  return false;
}

bool delDmxMapByDmx512Chan(Uint16 dmx512Chan)
{
  Uint16 tmpChan;
  
  for (Uint8 idx = 0; idx < numDmxwChans; idx++)
  {
    tmpChan = dmxMap[idx].dmx512Chan;
    if (tmpChan == 0)
    {
      shiftDownMapRecords(idx);
      logPrintln(FLASH("*** <warning - empty entry in middle of dmxMap[]"));
    }
    if (tmpChan == dmx512Chan)
    {
      bufSize = 0;
      buffer[bufSize++] = CMD_MAPR;
      buffer[bufSize++] = dmxMap[idx].dmxwChan;
      node              = dmxMap[idx].nodeId;
      dataToSend = true;
      shiftDownMapRecords(idx);
      return true;
    }
  }
  return false;
}

bool delDmxMapByDmxwChan(Uint8 dmxwChan)
{
  Uint8 tmpChan;
  Uint8 idx;
  
  for (idx = 0; idx < numDmxwChans; idx++)
  {
    tmpChan = dmxMap[idx].dmxwChan;
    if (tmpChan == 0)
    {
      shiftDownMapRecords(idx);
      logPrintln(FLASH("*** <warning - empty entry in middle of dmxMap[]"));
    }
    if (tmpChan == dmxwChan)
    {
      shiftDownMapRecords(idx);
      return true;
    }
  }
  return false;
}


void datafillDmxwRunPacket(void)
{
  #define DATA_START  1 // buffer position of 1st DMXW channel data entry
  Uint8 i;
  Uint8 idx;
  int tmpValue;
  int tmpChan;
  
  if (numDmxwChans == 0)
    return;
    
  if (testMode == DMXW_TEST_MODE)
  {
    if (testValue == 0)
      for (i = 0; i < numDmxwChans; i++)
        dmxMap[i].value = 0;
    dmxMap[testIdx].value = testValue;
  }
  else
  {
    // For each defined DMXW channel extract the current value for the
    // associated DMX-512 channel.
    for (i = 0; i < numDmxwChans; i++)
    {
      tmpChan = dmxMap[i].dmx512Chan;
      if ((tmpChan >= 1) && (tmpChan <= 512))
      {
        dmxMap[i].value = DMXSerial.read(tmpChan);
        /*logPrint(FLASH("DMX #"));
        logPrint(tmpChan);
        logPrint(FLASH("  value="));
        logPrintln(dmxMap[i].value);
        */
      }
      else
      {
        logPrint(FLASH("Found bad DMX chan #"));
        logPrintln(tmpChan);
      }
    }
  }
    
  bufSize = 0;
  memset(buffer, 0, sizeof(buffer));
  buffer[bufSize++] = CMD_RUN;
  
  // Datafill all DMXW channel values into the CMD_RUN packet.
  for (i = 0; i < numDmxwChans; i ++)
  {
    buffer[DATA_START + dmxMap[i].dmxwChan -1] = dmxMap[i].value;
  }
  
  if (testMode != DMXW_TEST_MODE)
  {
    if (consoleEnabled)
    {
      // Overwrite console control values if they're enabled
      for (Uint8 i = 0; i < NUM_BUTTONS; i++)
        if (buttonMap[i].dmxwChan > 0)
        {
          tmpValue = digitalRead(buttonMap[i].pin);
          // Buttons are configured with internal pull-up resistors. So
          // we need to negate the logic.
          if (i != (NUM_BUTTONS - 1))
            buttonMap[i].value = (tmpValue == 0 ? 0 : 255);
          else
            // Reverse the logic for the joystick button: it has
            // a built-in 3.3k resistor to Vcc.
            buttonMap[i].value = (tmpValue == 0 ? 255 : 0);
          tmpChan = findDmxMapByDmxw(buttonMap[i].dmxwChan);
          if (tmpChan != INVALID_MAP_INDEX)
            dmxMap[tmpChan].value = buttonMap[i].value;
          buffer[DATA_START + buttonMap[i].dmxwChan - 1] = buttonMap[i].value;
        }
      for (Uint8 i = 0; i < NUM_POTS; i++)
        if (potMap[i].dmxwChan > 0)
        {
          tmpValue = analogRead(potMap[i].pin);
          tmpValue = map(tmpValue, 0, 1023, 255, 0);
          potMap[i].value = constrain(tmpValue, 0, 255);
          tmpChan = findDmxMapByDmxw(potMap[i].dmxwChan);
          if (tmpChan != INVALID_MAP_INDEX)
            dmxMap[tmpChan].value = potMap[i].value;
          buffer[DATA_START + potMap[i].dmxwChan - 1] = potMap[i].value;
        }
      if (joystick.dmxwChan_x > 0)
      {
        tmpValue = analogRead(joystick.pin_x);
        tmpValue = map(tmpValue, 0, 1023, 255, 0);
        joystick.x_axis = constrain(tmpValue, 0, 255);
        tmpChan = findDmxMapByDmxw(joystick.dmxwChan_x);
        if (tmpChan != INVALID_MAP_INDEX)
          dmxMap[tmpChan].value = joystick.x_axis;
        buffer[DATA_START + joystick.dmxwChan_x - 1] = joystick.x_axis;
      }
      if (joystick.dmxwChan_y > 0)
      {
        tmpValue = analogRead(joystick.pin_y);
        tmpValue = map(tmpValue, 0, 1023, 255, 0);
        joystick.y_axis = constrain(tmpValue, 0, 255);
        tmpChan = findDmxMapByDmxw(joystick.dmxwChan_y);
        if (tmpChan != INVALID_MAP_INDEX)
          dmxMap[tmpChan].value = joystick.y_axis;
        buffer[DATA_START + joystick.dmxwChan_y - 1] = joystick.y_axis;
      }
    }
  }
  
  bufSize = DATA_START + MAX_DMXW_CHANS;
  node = BROADCASTID;
  dataToSend = true;
}



// Map a serialBuffer entry from a numeric character to an integer value.
Uint8 buildNodeList()
{
  Uint8 numNodes = 0;
  Int8  addIdx = 0;
  
  for (Uint8 i = 0; i < numDmxwChans; i++)
  {
    node = dmxMap[i].nodeId;
    for (Uint8 j = 0; j < numNodes; j++)
    {
      if (nodeList[j] == node)
      {
        addIdx = -1;
        break;
      }
      addIdx++;
    }
    if (addIdx >= 0)
    {
      nodeList[numNodes++] = node;
    }
  }
  return numNodes;
}


int serialParseInt(void)
{
  Uint8 tmp;
  char *token = NULL;
  
  tmp = strspn(&serialBuffer[serialPos], " ,#\n");
  token = strtok(&serialBuffer[serialPos], " ,#\n");
  serialPos += tmp;
  if (token != NULL)
  {
    serialPos += strlen(token) + 1;
    return atoi(token);
  }
  dbgPrintln(FLASH("Couldn't parse integer"));
  return 0;
}


void showSerialHelp()
{
  logPrintln();
  logPrintln(FLASH("Serial port commands are enabled. Commands are:"));
  logPrintln(FLASH("  (spaces may replaces commas)"));
  logPrint(FLASH("<x> in {1...512};  <n> in {1...20}; <d> in {1...48};  "));
  logPrintln(FLASH("<p> in {1...16};  <v> in {0...255}"));
  logPrintln(FLASH("  c[b|j|p] <i>,<d>     - Map console button i, joystick, or "
                                              "potentiometer i to DMXW "));
  logPrintln(FLASH("                         channel d (d=0) to delete. ["
                                              "For joystick, i=0 is x-axis, "));
  logPrintln(FLASH("                          i=1 is y-axis.]"));
  if (!dmx512Running)
  {
    logPrintln(FLASH("  f <n>                - Turn ofF all ports at node n, "
                                                 "or at all nodes (n = 255)"));
  }
  logPrintln(FLASH("  free                 - Display free RAM"));
  logPrintln(FLASH("  h                    - Print this help text"));
  if (!dmx512Running)
  {
    logPrintln(FLASH("  l <n>                - Locate node n"));
    logPrintln(FLASH("  m <x>,<d>,<n>,<p>,<l> - Map DMX-512 chan x to DMXW "
                                               "chan d, which is assigned to "));
    logPrintln(FLASH("                           node n port p (l=1 means "
                                                "scale logarithmically; 0 "
                                                "otherwise)"));
    logPrintln(FLASH("  n [<v>]              - Show all DMXW channel mapping "
                                                 "detail for all known "
                                                 "channels."));
    logPrintln(FLASH("                           (quiet mode if v present & not 0)"));
    logPrintln(FLASH("  p <n>                - Ping node n / all nodes (n = 255)"));
    logPrintln(FLASH("  r <x>                - Remove map for DMX-512 chan x "));
  }
  logPrintln(FLASH("  s                    - Show DMX channel mappings and DMXW "
                                               "channel values"));
  logPrintln(FLASH("  t <t>                - time DMXW transmissions for "
                                               "<t> seconds"));
  if (dmx512Running)
  {
    logPrintln(FLASH("  test <e>,<d>,<s>     - test DMXW channel d / all known "
                                                 "channels (d = 0)."));
    logPrintln(FLASH("                           [e=1, enable; e=0, disable test] "
                                                 "with speed s (0 - 9000)"));
  }
  if (!dmx512Running)
  {
    logPrintln(FLASH("  z <n>,<d>,<v>        - Send ctrl command to node n, "
                                                 "indicating that the port"));
    logPrintln(FLASH("                         assigned to DMXW channel d "
                                              "should take value v."));
  }
  logPrintln(FLASH("  ------------------------------------------------------"));
  logPrintln(FLASH("  run  | stop          - Run/stop DMX-512 distribution thru "
                                             "DMXW network."));
  if (!dmx512Running)
  {
    logPrintln(FLASH("  save                 - Save to EEPROM, DMX-512/DMXW "
                                                 "mappings at gateway, and "));
    logPrintln(FLASH("                            DMXW/Port mappings at "
                                                 "all nodes."));
    logPrintln(FLASH("  copy <n>             - Copy channel mappings for node "
                                                 "n back to node n"));
    logPrintln(FLASH("  xxx <n>              - Clear all DMXW mappings at node n, "
                                                 "or at all nodes (n = 255)."));
    logPrintln(FLASH("  xxxs <n>             - Same as xxx, but save cleared data "
                                                 "to EEPROM as well."));
  }
  else
  {
    logPrintln(FLASH("\nFor more commands, type 'stop', then 'h'"));
  }
  logPrintln(FLASH("  ------------------------------------------------------"));
  logPrint(FLASH("DMXW channels:  Total avail - "));
  logPrint(MAX_DMXW_CHANS);
  logPrint(FLASH("\tMapped - "));
  logPrint(numDmxwChans);
  logPrint(FLASH("\tNum unmapped - "));
  logPrintln(MAX_DMXW_CHANS - numDmxwChans);
  logPrint(FLASH("DMX-512 incoming is "));
  if (!initialized)
    logPrint(FLASH("<undetermined>"));
  else if (dmx512Suspended)
    logPrint(FLASH("***inactive***"));
  else
    logPrint(FLASH("active"));
  logPrint(FLASH("\tDMXW is "));
  logPrintln(dmx512Running ? "running" : "***stopped***");
}


void getSerialCommand()
{  
  // Handle commands from the serial port
  // - Serial port command have the format (without spaces):
  //       <cmd> [<arg> [, <arg>]*] #
  //   - where:
  //       <cmd> in {'c',...}
  //       <arg> are integers
  //       # is the char '#' (octothorpe/hash)
  Uint8  port = 0;
  Uint8  val = 0;
  Uint8  dmxwChan = 0;
  Uint16 dmx512Chan = 0;
  Uint8  idx = 0;
  Uint8  logarithmic = 0;
  bool   cmdToProcess = false;
  bool   cmdInvalid = false;
  bool   blockWhileRunning = true;
  DmxwGwMapRecord_t *tmp;
  bool   serialDone = false;
  Uint8  serialBufSize = 0;


  if (configEnabled != oldConfigEnabled)
  {
    oldConfigEnabled = configEnabled;
    if (configEnabled)
    {
      // Clear whatever garbage may be sitting on the serial port.
      while (consSerial.available() > 0)
        consSerial.read();
    }
  }

  if (consSerial.available() > 0)
  {
    serialBuffer[serialPos] = consSerial.read();
    if ( (serialBuffer[serialPos] == '\b') ||
         (serialBuffer[serialPos] == DEL_CHAR) )
    {
      if (serialPos > 0)
        serialPos--;
      serialBuffer[serialPos] = '\0';
      return;
    }
    else if (serialBuffer[serialPos] == '\r')
    {
      serialBuffer[serialPos] = '\0';
      serialBufSize = serialPos;
      cmdToProcess = true;
      while (consSerial.available() > 0)
        (void)consSerial.read();
    }
    else if (serialPos >= (MAX_SERIAL_BUF_LEN - 1))
    {
      serialPos = 0;
      return;
    }
    else
      serialPos++;
  }
  else
    return;
    
  if (cmdToProcess)
  {
    cmdToProcess = false;
    serialPos = 0;
    logPrint(FLASH("\n\rCommand: ["));
    logPrint(serialBuffer);
    logPrint(FLASH("]   DMX-512 distribution (DMXW) is "));
    logPrintln(dmx512Running ? "running" : "stopped");
    cmdInProgress = CMD_UNDEF;
    waitForReply = false;
    bufSize = 0;
    
    // Block commands that are incompatible with running DMX-512 broadcasts
    if (dmx512Running)
    {
      blockWhileRunning = true;
      switch (serialBuffer[serialPos])
      {
        case 'c':
          // Copy command is blocked. Others aren't.
          if (strstr(serialBuffer, "copy") == null)
            blockWhileRunning = false;
          break;
          
        case 'h':
        case 't':
          // Do nothing. Let command handling pass thru
          blockWhileRunning = false;
          break;
          
        case 's':
          if ( (strstr(serialBuffer, "stop") != null) ||
               (serialBufSize == 1) )
            blockWhileRunning = false;
          break;
          // Definitely not the 'show' or 'stop' command. So fall thru.
          
        case 'f':
          if (strstr(serialBuffer, "free") != null)
            blockWhileRunning = false;
          break;
          // Not the "free" command. So fall thru.
          
        default:
          break;
      }
      if (blockWhileRunning)
      {
        logPrintln(FLASH("*** Can't process this command. "
                         "Stop DMX-512 first ***"));
        serialPos = 0;
        return;
      }
    }
    switch (serialBuffer[serialPos++])
    {

      case 'c':
        if (strstr(serialBuffer, "copy") != NULL)
        {
          // copy <n>
          serialPos += 3;
          node = serialParseInt();
          if (findDmxMapByNodeId(node) == INVALID_MAP_INDEX)
          {
            logPrint(FLASH("*** Node #"));
            logPrint(node);
            logPrintln(FLASH(" doesn't have any channel mappings."));
            break;
          }
          else
          {
            for (int i = 0; i < numDmxwChans; i++)
            {
              if (dmxMap[i].nodeId == node)
              {
                bufSize = 0;
                buffer[bufSize++] = myNodeId;
                buffer[bufSize++] = node;
                buffer[bufSize++] = CMD_MAP;
                buffer[bufSize++] = dmxMap[i].dmxwChan;
                buffer[bufSize++] = dmxMap[i].port;
                buffer[bufSize++] = dmxMap[i].logarithmic;
                logPrint(FLASH("Copying DMXW chan #"));
                logPrint(dmxMap[i].dmxwChan);
                logPrint(FLASH(" to node #"));
                logPrintln(node);
                (void)sendBuffer(node, buffer, bufSize, false);
                delay(50);
              }
            }
          }
          //JVS??
        }
        else
        {
          // c[b|j|p] <i>, <d>
          // Map button i, joystick, or potentiometer i to DMXW channel d (d=1
          // to delete). (For joystick, i=0 is x-axis, i=1 is y-axis.)
          switch (serialBuffer[serialPos++])
          {
            case 'b':
              idx = serialParseInt();
              dmxwChan = serialParseInt();
              if ( (idx > 0) && (idx <= NUM_BUTTONS) )
              {
                if ( (dmxwChan >= 0) && (dmxwChan <= MAX_DMXW_CHANS) )
                  buttonMap[idx-1].dmxwChan = dmxwChan;
                else
                  logPrintln(FLASH("*** Invalid DMXW channel number"));
              }
              else
                logPrintln(FLASH("*** Invalid button number"));
              break;
           
            case 'j':
              idx = serialParseInt();
              dmxwChan = serialParseInt();
              if ( (idx == 0) || (idx == 1) )
              {
                if ( (dmxwChan >= 0) && (dmxwChan <= MAX_DMXW_CHANS) )
                  if (idx == 0)
                    joystick.dmxwChan_x = dmxwChan;
                  else
                    joystick.dmxwChan_y = dmxwChan;
                else
                  logPrintln(FLASH("*** Invalid DMXW channel number"));
              }
              else
                logPrintln(FLASH("*** Invalid joystick axis specified"));
              break;

            case 'p':
              idx = serialParseInt();
              dmxwChan = serialParseInt();
              if ( (idx >= 0) && (idx <= NUM_POTS) )
              {
                if ( (dmxwChan >= 0) && (dmxwChan <= MAX_DMXW_CHANS) )
                  potMap[idx-1].dmxwChan = dmxwChan;
                else
                  logPrintln(FLASH("*** Invalid DMXW channel number"));
              }
              else
                logPrintln(FLASH("*** Invalid potentiometer number"));
              break;

            default:
              logPrintln(FLASH("*** Invalid console mapping command."));
          }
        }
        break;

      case 'f':
        if (strstr(serialBuffer, "free") != null)
        {
          CheckRam();
        }
        else
        {
          // f <n>
          // Turn off all ports at node n, or all all node if n = 255
          node = serialParseInt();
          if ( (node != BROADCASTID) && (node > NODEID_MAX))
          {
            logPrint(FLASH("*** Invalid node Id #"));
            logPrintln(node);
          }
          else
          {
            buffer[bufSize++] = CMD_OFF;
            buffer[bufSize++] = node;
            dataToSend = true;
            logPrint(FLASH("'OFF' command sent to "));
            if (node == BROADCASTID)
            {
              logPrintln(FLASH("ALL nodes"));
            }
            else
              logPrint(FLASH("node #"));
              logPrintln(node);
          }
        }
        break;
        
      case 'h':
        // h
        // Display serial command help text.
        showSerialHelp();
        break;
        
      case 'l':
        // l <n>
        // Locate node n
        node = serialParseInt();
        buffer[bufSize++] = CMD_LOC;
        dataToSend = true;

        break;
        
      case 'm':
        // m <x>, <d>, <n>, <p>, <l>
        // Map DMX-512 channel x to DMXW channel d which is, in turn, to
        // be assigned to node n, port p, (whose values are to be scaled
        // logarithmically when l=1)
        dmx512Chan  = serialParseInt();
        dmxwChan    = serialParseInt();
        node        = serialParseInt();
        port        = serialParseInt();
        logarithmic = serialParseInt();
        if (!addDmxMap(dmx512Chan, dmxwChan, node, port, logarithmic))
        {
          logPrintln(FLASH("*** Unable to add Channel Map. Check "
                               "arguments and table"));
        }
        else
        {
          idx = findDmxMapByDmx512(dmx512Chan);
          if (idx == INVALID_MAP_INDEX)
            logPrint(FLASH("*** Oops. Map entry not added."));
          else
          {
            logPrint(FLASH("DMX Mapper table entry #"));
            logPrint(idx);
            logPrintln(FLASH(" added."));
          }
        }
        break;
        
      case 'n':
        // n [<v>]
        // Show all DMXW channel mapping details for all nodes.
        // (Quiet mode if v present and not 0.)
        quiteMode = serialParseInt();
        cmdInProgress = CMD_ECHO;
        logPrintln(FLASH("Requesting Port Mapping info from nodes..."));
        break;
        
      case 'p':
        // p <n>
        // Ping node n
        node = serialParseInt();
        buffer[bufSize++] = CMD_PING;
        dataToSend = true;
        requestAck = false;
        deferDmx512 = true;
        break;
        
      case 'r':
        if (strstr(serialBuffer, "run") != NULL)
        {
          // run
          // Run DMX-512 distribution throughout the DMXW network.
          dmx512Running = true;
          logPrintln(FLASH("DMX-512 is now running"));
        }
        else
        {
          // r <x>
          // Remove mapping entry for DMX-512 channel x.
          dmx512Chan = serialParseInt();
          if (delDmxMapByDmx512Chan(dmx512Chan))
          {
            logPrint(FLASH("Mapping deleted for DMX-512 channel "));
            logPrintln(dmx512Chan);
          }
          else
          {
            logPrint(FLASH("*** failed to delete mapping for DMX-512 "
                               "channel "));
            logPrintln(dmx512Chan);
          }
        }
        break;
      
      case 's':
        if (strstr(serialBuffer, "stop") != NULL)
        {
          // stop
          // Stop DMX-512 distribution throughout the DMXW network.
          dmx512Running = false;
          logPrintln(FLASH("DMX-512 is STOPPED"));
        }
        else if (strstr(serialBuffer, "save") != NULL)
        {
          // save
          // Save to EEPROM, DMX-512/DMXW mappings at gateway, and
          // DMXW/Port mappings at all nodes.
          Int8 addIdx = 0;
          
          EepromSave();
          serialPos += 3;
          numNodes = buildNodeList();
          
          if (numNodes > 0)
          {
            cmdInProgress = CMD_SAVE;
            logPrintln(FLASH("Config data saved at gateway. "
                             "SAVE at nodes in progress..."));
          }
          else
          {
            logPrintln(FLASH("Warning - no nodes mapped. Config data saved "
                             "at gateway only."));
          }
        }
        else
        {
          // s
          // Show DMX channel mappings
          logPrintln(FLASH("\nConsole control to DMXW Channel Mappings"));
          logPrintln(FLASH(  "========================================"));
          logPrintln(FLASH("DMXW\tConsole\tIn Pin\tValue"));
          logPrintln(FLASH("----\t-------\t------\t-----"));
          for (Uint8 i = 0; i < NUM_BUTTONS; i++)
          {
            if (buttonMap[i].dmxwChan <= 0)
              logPrint(FLASH("  -"));
            else
              logPrint(buttonMap[i].dmxwChan);
            logPrint("\tB");
            logPrint(i+1); logPrint("\t");
            logPrint(buttonMap[i].pin); logPrint("\t");
            logPrintln(buttonMap[i].value);
          }
          
          if (joystick.dmxwChan_x <= 0)
            logPrint(FLASH("  -"));
          else
            logPrint(joystick.dmxwChan_x);
          logPrint("\tJ-x\tA");
          logPrint(joystick.pin_x); logPrint("\t");
          logPrintln(joystick.x_axis);
          if (joystick.dmxwChan_y <= 0)
            logPrint(FLASH("  -"));
          else
            logPrint(joystick.dmxwChan_y);
          logPrint("\tJ-y\tA");
          logPrint(joystick.pin_y); logPrint("\t");
          logPrintln(joystick.y_axis);

          for (Uint8 i = 0; i < NUM_POTS; i++)
          {
            if (potMap[i].dmxwChan <= 0)
              logPrint(FLASH("  -"));
            else
              logPrint(potMap[i].dmxwChan);
            logPrint("\tP");
            logPrint(i+1); logPrint("\tA");
            logPrint(potMap[i].pin); logPrint("\t");
            logPrintln(potMap[i].value);
          }
          
          
          logPrintln(FLASH("\nDMX Channel Map for Gateway"));
          logPrintln(FLASH(  "==========================="));
          logPrint(FLASH("# entries: ")); logPrintln(numDmxwChans);
          if (numDmxwChans)
          {
            logPrintln(FLASH("Idx\tDMX-512\tDMXW\tNode\tPort\tLog?\tValue"
                             "\tConsole"));
            logPrintln(FLASH("---\t-------\t----\t----\t----\t----\t-----"
                             "\t-------"));
            for (Uint8 idx = 0; idx < MAX_DMXW_CHANS; idx++)
            {
              tmp = &dmxMap[idx];
              dmx512Chan  = tmp->dmx512Chan;
              dmxwChan    = tmp->dmxwChan;
              node        = tmp->nodeId;
              port        = tmp->port;
              logarithmic = tmp->logarithmic;
              val         = tmp->value;
              if (dmxwChan)
              {
                logPrint(idx); logPrint("\t");
                logPrint(dmx512Chan); logPrint("\t");
                logPrint(dmxwChan); logPrint("\t");
                logPrint(node); logPrint("\t");
                logPrint(port); logPrint("\t");
                logPrint(logarithmic); logPrint("\t");
                logPrint(val); logPrint("\t");
                for (Uint8 i = 0; i < NUM_BUTTONS; i++)
                  if (buttonMap[i].dmxwChan == dmxwChan)
                  {
                    logPrint("B"); logPrint(i+1);
                  }
                for (Uint8 i = 0; i < NUM_POTS; i++)
                  if (potMap[i].dmxwChan == dmxwChan)
                  {
                    logPrint("P"); logPrint(i+1);
                  }
                if (joystick.dmxwChan_x == dmxwChan)
                  logPrint("J-x");
                if (joystick.dmxwChan_y == dmxwChan)
                  logPrint("J-y");
                logPrintln();
              }
            }
            logPrintln(FLASH("----------------------------------------------"
                             "---------------"));
          }
          logPrintln();
        }
        break;

      case 't':
        if (strstr(serialBuffer, "test") != NULL)
        {
          // test <e> <d>
          if (!dmx512Running)
          {
            logPrintln(FLASH("*** DMX-512 distribution must be running in "
                             "order to run a test"));
            break;
          }
          serialPos += 3;
          testMode = serialParseInt();
          logPrintln(testMode);
          testParam = 0;
          if (testMode > 0)
          {
            testMode = DMXW_TEST_MODE;
            testParam = serialParseInt();
            if (testParam != 0)
            {
              testIdx = findDmxMapByDmxw(testParam);
              if (testIdx ==  INVALID_MAP_INDEX)
              {
                logPrint(FLASH("*** Invalid DMXW channel to test: "));
                logPrintln(testParam);
                testMode = 0;
                break;
              }
            }
            testIdx = -1;
            testSpeed = serialParseInt();
            if (testSpeed > 9000)
            {
              testSpeed = 9000;
              logPrint(FLASH(">> test speed set to slowest value, "));
              logPrintln(testSpeed);
            }
          }
        }
        else
        {
          // t <t>
          // Time DMXW transmissions for t seconds.
          if (dmx512Running)
          {
            transmissionCountdown = serialParseInt();
            transmissionCountdown = transmissionCountdown * 1000;
            dmxTimingStart = true;
          }
          else
            logPrintln(FLASH("*** Can't time DMXW transmissions. DMX-512 isn't "
                             "running. Type 'run' to start."));
        }
        break;
        
      case 'x':
        if (strstr(serialBuffer, "xxx") != null)
        {
          // xxx <n>     OR
          // xxxs <n>
          // Clear all DMXW mappings at node n or at all nodes (n = 255).
          // If "xxxs", save cleared data to EEPROM as well.
          Uint8 idx;
          
          serialPos += 2;
          if (strstr(serialBuffer, "xxxs") != null)
          {
            saveNodes = true;
            serialPos++;
          }
          node = serialParseInt();
          cmdInProgress = CMD_CLRALL;

          if (node == BROADCASTID)
          {
            numNodes = buildNodeList();
          }
          else
          {
            numNodes = 0;
            nodeList[numNodes++] = node;
          }
        }
        else
          cmdInvalid = true;
        break;

      case 'z':
        // z <n>, <p>, <v>
        // Send Ctrl(p, v) to node n
        node = serialParseInt();
        port = serialParseInt();
        val  = serialParseInt();
        buffer[bufSize++] = CMD_CTRL;
        buffer[bufSize++] = port;
        buffer[bufSize++] = val;
        dataToSend = true;
        break;
        
      default:
        logPrint(FLASH("Invalid command '"));
        logPrint(serialBuffer[0]);
        logPrintln("'");
    }
    
    serialPos = 0;
    
    if (cmdInvalid)
      logPrintln(FLASH("Invalid command."));
  }
}


void dumpBuffer()
{
  char hex[3];
  
  logPrint(FLASH("\n\rDUMP:  Src["));
  logPrint(srcNodeId);
  logPrint(FLASH("/"));
  logPrint(radio.DATA[0]);
  logPrint(FLASH("]  Dst["));
  logPrint(dstNodeId);
  logPrint(FLASH("/"));
  logPrint(radio.DATA[1]);
  logPrint(FLASH("] Size ["));
  logPrint(bufSize);
  logPrint(FLASH("]\n\rBuffer: ["));
  for (int i = 0; i < bufSize; i++)
  {
    sprintf(hex, "%.2x", buffer[i]);
    logPrint(hex);
    logPrint(" ");
  }
  logPrintln(FLASH("]."));
}


void UpdateActivityLed(bool active, Uint8 pin,
                       Uint8 &ledValue, Int8 &dirIncrement)
{
  if (active)
  {
    if ( (loopCount %20) == 0)
    {
      ledValue += dirIncrement;
      if ((ledValue == 0) | (ledValue == 255))
        dirIncrement = -dirIncrement;
    }
  }
  else
  {
    ledValue = 0;
    dirIncrement = 1;
  }
  analogWrite(pin, ledValue);
}


void CheckConfigEnabled()
{
  configEnabled = digitalRead(CONFIG_ENABLED_PIN);
  if (configEnabled != oldConfigEnabled)
  {
    if (configEnabled)
      showSerialHelp();
    else
    {
      logPrintln(FLASH("==> Configuration serial port disabled: "
                       "check Config Enable switch"));
      oldConfigEnabled = false;
    }
  }
}


/***************************************************************************
 * Test how much RAM is left on the MPU, printing the results out to
 * the serial port.
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
  consSerial.begin(SERIAL_BAUD);
  delay(10);
  DMXSerial.init(DMXReceiver);
  radio.initialize(FREQUENCY, myNodeId, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  currReadPos = 0;

  // Initialize empty mappings
  tmpMapRecord.dmxwChan   = 0;
  tmpMapRecord.dmx512Chan = 0;
  tmpMapRecord.nodeId     = 0;
  tmpMapRecord.port       = 0;
  tmpMapRecord.value      = 0;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    dmxMap[i] = tmpMapRecord;
  }

  for (Uint8 i = 0; i < NUM_BUTTONS; i++)
  {
    buttonMap[i].dmxwChan  = 0;
    switch (i)
    {
      case 0: buttonMap[i].pin = BUTTON1_PIN;  break;
      case 1: buttonMap[i].pin = BUTTON2_PIN;  break;
      case 2: buttonMap[i].pin = BUTTON3_PIN;  break;
      case 3: buttonMap[i].pin = BUTTON4_PIN;  break;
      case 4: buttonMap[i].pin = BUTTON5_PIN;  break;
      default:
        logPrintln(FLASH("*** Button map error"));
    }
    buttonMap[i].value     = LOW;
    pinMode(buttonMap[i].pin, INPUT_PULLUP);
  }
  for (Uint8 i = 0; i < NUM_POTS; i++)
  {
    potMap[i].dmxwChan   = 0;
    switch (i)
    {
      case 0: potMap[i].pin = POT1_PIN;  break;
      case 1: potMap[i].pin = POT2_PIN;  break;
      default:
        logPrintln(FLASH("*** Pot map error"));
    }
    potMap[i].value      = 0;
  }
  joystick.dmxwChan_x    = 0;
  joystick.dmxwChan_y    = 0;
  joystick.pin_x         = JOY_X_PIN;
  joystick.pin_y         = JOY_Y_PIN;
  joystick.x_axis        = 0;
  joystick.y_axis        = 0;
  
  pinMode(DMXW_ACTIVITY_LED,   OUTPUT);
  pinMode(DMX512_ACTIVITY_LED, OUTPUT);
  pinMode(CONFIG_ENABLED_PIN,  INPUT);
  pinMode(CONSOLE_ENABLED_PIN, INPUT);

  // Check if we need to clear out EEPROM and start from scratch
  resetEeprom = (EEPROM.read(EEPROM_FW_ADDR) != FW_VERSION_c);
  if (resetEeprom)
  {
    // Record new F/W version and start with blank mappings
    EEPROM.write(EEPROM_FW_ADDR, FW_VERSION_c);
    EEPROM.write(EEPROM_NUMDMXNODES_ADDR, 0);  // numDmxwChans
  }
  else
  {
    // Read in mappings stored in EEPROM
    EepromLoad();
  }
  for (Uint8 i = 0; i < MAX_DATA_LEN; i++)
    buffer[i] = 0;

  for (Uint8 i = 1; i <= 5; i++)
    logPrintln();
  for (Uint8 i = 1; i <= 80; i++)
    logPrint(FLASH("#"));
  logPrintln();
  logPrint(FLASH("DMX Wireless Network...\t\tNode #"));
  logPrint(myNodeId);
  logPrint(FLASH("   (Gateway)   Radio frequency: "));
  logPrint(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  logPrintln(FLASH("Mhz"));
  logPrint(FLASH("S/W: "));
  logPrint(SW_VERSION_c);
  logPrint(FLASH("\tF/W: "));
  logPrint(FW_VERSION_c);
  logPrint(FLASH("\t"));
  logPrint(COPYRIGHT);
  logPrint(FLASH("\t"));
  CheckRam();
  
  CheckConfigEnabled();
  oldConfigEnabled = configEnabled;
  if (configEnabled)
    showSerialHelp();
}

//------------------------------------------------------------------
void loop()
{
  handleInput = false;
  bufSize = 0;
  dataToSend = false;
  requestAck = true;
  ackBuf[0] = ACK_ERR;
  dmx512Suspended = (DMXSerial.noDataSince() > 1000);

  loopCount++;
  consoleEnabled = digitalRead(CONSOLE_ENABLED_PIN);
  CheckConfigEnabled();

  // Handle incoming messages.
  if (!dmx512Running || deferDmx512)
  {
    // IMPORTANT:
    // We don't check for incoming messages when we're sending a lot
    // of DMX data as it appears that receiving immediately after sending
    // (or perhaps the problem is sending immediately after receiving)
    // results in transmitted packets getting corrupted quite frequently.
    if (radio.receiveDone())
    {
      // Determine if the message is aimed at us.
      dstNodeId = radio.TARGETID;
      srcNodeId = radio.SENDERID;
      if ( (dstNodeId == myNodeId) || (dstNodeId == BROADCASTID) )
      {
        if ( (radio.DATA[0] == srcNodeId) && (radio.DATA[1] == dstNodeId))
        {
          handleInput = true;
          ackRequested = radio.ACK_REQUESTED;
          bufSize = radio.DATALEN - 2;
          memcpy(buffer, (const void *)&radio.DATA[2], bufSize);
        }
        else
        {
          logPrintln(FLASH("*** RX corrupt pkt; dropped."));
          dumpBuffer();
        }
      }
      else
      {
        dbgPrintln(FLASH(" ... ignored (not intended for me)"));
      }
      dbgPrint(FLASH("RX  Src["));
      dbgPrint(radio.SENDERID);
      dbgPrint(FLASH("]  Dst["));
      dbgPrint(radio.TARGETID);
      dbgPrint(FLASH("]  Size["));
      dbgPrint(radio.DATALEN);
      dbgPrint(FLASH("]  "));

      /*logPrint(FLASH(" DATA["));
      for (Uint8 i = 0; i < radio.DATALEN; i++)
      {
        logPrint(radio.DATA[i]); logPrint(" ");
      }
      logPrint("]  ");
      */
    
  
      if (handleInput)
      {
        rxTime = millis();
        currReadPos = 0;
        command = buffer[currReadPos++];
        dbgPrint(FLASH("Cmd["));
        printCommand(command);
        dbgPrint("] ");
        ackBuf[0] = (Uint8) handleNetRxMessage(command);
        dbgPrint(FLASH("  Result["));
        printAckResult(ackBuf[0]);
        dbgPrintln("]");
        if (ackRequested)
        {
          radio.sendACK(ackBuf, 1);
          dbgPrint(FLASH("ACK sent["));
          dbgPrint(millis() - rxTime);
          dbgPrintln("]");
        }
      }
    }
  }
  
  // Handle DMXW channel test mode
  if (testMode == DMXW_TEST_MODE)
  {
    if (testIdx == -1)
    {
      logPrintln();
      logPrintln(FLASH("########## DMXW Channel Test Mode ##########"));
      if (testParam == 0)
        testIdx = -1;
      else
        testIdx = findDmxMapByDmxw(testParam);
      testValue = 255;
      testIterations = 0;
    }
    testValue++;
    if (testValue > 255)
    {
      testValue = 0;
      if (testParam == 0)
      {
        testIdx++;
        if (testIdx >= numDmxwChans)
        {
          testIdx = 0;
          testIterations++;
        }
      }
      else
        testIterations++;
      if (testIterations > 10)
      {
        testMode = 0;
        logPrint(FLASH("Test stopped at "));
        logPrint(testIterations - 1);
        logPrintln(FLASH(" iterations."));
      }
      else{
        logPrint(FLASH("-> testing DMXW channel #"));
        logPrint(dmxMap[testIdx].dmxwChan);
        logPrint(FLASH(" (DMX-512 channel #"));
        logPrint(dmxMap[testIdx].dmx512Chan);
        logPrintln(FLASH("). [Type 'test 0' to stop]"));
      }
    }
    delayMicroseconds(testSpeed + 1000); // wait a little bit
  }
  
  // Handle multi-cycle commands:
  // Only execute the next transmit of a multi-message command if there isn't
  // already data that needs to be sent.
  if ( (cmdInProgress != CMD_UNDEF)  && !dataToSend )
  {
    if ( (iteration < numDmxwChans) && (numDmxwChans > 0) )
    {
      if (iteration == -1)
      {
        iteration = 0;
        deferDmx512 = true;
      }
    }
    switch (cmdInProgress)
    {
      case CMD_ECHO:
        if ( (iteration < numDmxwChans) && (numDmxwChans > 0) )
        {
          if (waitForReply)
          {
            // No reply from node. Move on to the next iteration
            if (millis() >= cmdTimeout)
            {
              waitForReply = false;
              logPrintln(FLASH("\t\t...timeout. Next channel"));
            }
          }
          
          if (!waitForReply)
          {
            waitForReply = true;
            requestAck = false;
            bufSize = 0;
            buffer[bufSize++] = CMD_ECHO;
            buffer[bufSize++] = dmxMap[iteration].dmxwChan;
            //buffer[bufSize++] = dmxMap[iteration].port;
            node = dmxMap[iteration].nodeId;
            if (!quiteMode)
            {
              logPrint(FLASH("   Query "));
              logPrint(node);
              logPrint(FLASH(", "));
              logPrint(dmxMap[iteration].dmxwChan);
              logPrintln(FLASH("..."));
            }
            dataToSend = true;
            cmdTimeout = millis() + 100;
            iteration++;
          }
        }
        else
        {
          cmdInProgress = CMD_UNDEF;
          iteration = -1;
          quiteMode = 0;
          logPrintln(FLASH("'Show mappings' requests completed."));
        }
        break;
        
      case CMD_CLRALL:
        if ( (iteration < numNodes) && (numNodes > 0) )
        {
          Int8 idx;
          
          bufSize = 0;
          buffer[bufSize++] = CMD_CLRALL;
          node = nodeList[iteration];
          dataToSend = true;
          iteration++;
          logPrint(FLASH("Clearing data at node#"));
          logPrintln(node);
          while ( (idx = findDmxMapByNodeId(node)) != INVALID_MAP_INDEX)
            delDmxMapByDmxwChan(dmxMap[idx].dmxwChan);
        }
        else
        {
          logPrintln(FLASH("'ClearAll' done."));
          iteration = -1;
          if (saveNodes)
          {
            cmdInProgress = CMD_SAVE;
          }
          else
          {
            cmdInProgress = CMD_UNDEF;
            numNodes = 0;
          }
        }

        break;
        
      case CMD_SAVE:
        if ( (iteration < numNodes) && (numNodes > 0) )
        {
          bufSize = 0;
          buffer[bufSize++] = CMD_SAVE;
          node = nodeList[iteration];
          dataToSend = true;
          iteration++;
          logPrint(FLASH("Saving data at node#"));
          logPrintln(node);
        }
        else
        {
          if (saveNodes)
          {
            saveNodes = false;
            EepromSave();
          }
          cmdInProgress = CMD_UNDEF;
          iteration = -1;
          numNodes = 0;
          logPrintln(FLASH("Configuration 'save' completed."));
        }
        break;
        
      default:
        cmdInProgress = CMD_UNDEF;
        waitForReply = false;
        deferDmx512 = false;
        iteration = -1;
    }
  }
  
  if (configEnabled)
    getSerialCommand();
  
  
  if (deferDmx512)
  {
    deferDmx512 = false;
    suspendStartTime = millis();
    logPrint(FLASH("\n\r--------- DMX512 suspended for "));
    logPrint(DMX_SUSPEND_DURATION/1000);
    logPrintln(FLASH(" seconds --------"));
  }
  else if (suspendStartTime > 0)
  {
    if ((millis() - suspendStartTime) >= DMX_SUSPEND_DURATION)
    {
      suspendStartTime = 0;
      dmxwTxTime = millis() + DMXW_TX_DELAY;
    }
  }
  
  if (dmx512Running && !dataToSend && !cmdInProgress && (suspendStartTime == 0))
  {
    if (millis() >= dmxwTxTime)
    {
      dmxwTxTime = millis() + DMXW_TX_DELAY;
      datafillDmxwRunPacket();
      txCount++;
    }
  }
  
  if (dataToSend)
  {
    dataToSend = false;
    if ((bufSize + 2) <= MAX_DATA_LEN)
    {
      ackTime = millis();
      memmove(&buffer[2], buffer, bufSize);
      bufSize += 2;
      buffer[0] = myNodeId;
      buffer[1] = node;
      ackBuf[0] = sendBuffer(node, buffer, bufSize, requestAck);
      dbgPrint(FLASH(" RxAck["));
      dbgPrint(millis() - ackTime);
      dbgPrint("]");
      if ( (ackBuf[0] != ACK_OK) && (cmdInProgress != CMD_UNDEF) )
      {
        logPrint(FLASH("\nNode ["));
        logPrint(node);
        logPrint("] ");
        if ( ackBuf[0] == ACK_ETIME )
          waitForReply = false;
      }
      if (DEBUGGING)
      {
        dbgPrint(FLASH("  Result["));
        printAckResult(ackBuf[0]);
        dbgPrintln("]");
      }
      else if (ackBuf[0] != ACK_OK)
      {
        logPrint(FLASH("  Result["));
        printAckResult(ackBuf[0]);
        logPrintln("]");      
      }
    }
  }
  
  if (transmissionCountdown > 0)
  {
    if (dmxTimingStart)
    {
      dmxTimingStart = false;
      dmxwTimingStart = millis();
      transmissionCountdown = transmissionCountdown + dmxwTimingStart;
      txCount = 0;
    }
  }
  if (transmissionCountdown > 0)
  {
    if (transmissionCountdown < millis())
    {
      dmxwFrequency = (float)txCount /
                      (float)(transmissionCountdown - dmxwTimingStart);
      transmissionCountdown = 0;
      dmxwFrequency *= 1000.0;
      logPrint(FLASH("====>  DMX-512 distribution rate: "));
      logPrint(dmxwFrequency);
      logPrintln(FLASH("Hz  <====="));
    }
  }
  
  initialized = true;
  
  UpdateActivityLed(dmx512Running, DMXW_ACTIVITY_LED,
                    dmxwActLedValue, dmxwActLedDir);
  UpdateActivityLed(!dmx512Suspended, DMX512_ACTIVITY_LED,
                    dmx512ActLedValue, dmx512ActLedDir);
}
