/******************************************************************************
 * DMX_Wireless_Node
 *
 * This sketches implements a DMX wireless node that interacts with the
 * DMS wireless gateway.
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
 *     11 - 16 (for psuedo analog outputs (PWM) and analog inputs). As a general
 *     rule, the pin assignments for each Moteino node are (but they
 *     can be modified if deemed absolutely necessary):
 *         Port #  Pin # (Moteino)
 *         ------  ---------------
 *           1     2
 *           2     3*           (* = overlapping digital and PWM output)
 *           3     4
 *           4     5*
 *           5     6*
 *           6     7
 *           7     8
 *           8     9*           (Note pin 9 has default onboard locator LED)
 *           9     12
 *           10    13
 *           11    Out 3* (PWM) & In A7
 *           12    Out 5* (PWM) & In A6
 *           13    Out 6* (PWM) & In A5
 *           14    Out 9* (PWM) & In A4
 *           15    10
 *           16    11
 *
 * Notes:
 * =====
 *   Special EEPROM memory locations:
 *      Address   Description
 *      -------   ----------------------------------
 *         0      Firmware Version
 *         1      Node ID (recorded in myNodeId)
 *         2      Mapping data validity (1=valid; 0=invalid)
 *   Arduino Serial Monitor settings for console I/O:
 *     - 9600 baud
 *     - "Carriage return" as line ending
 *
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 0.1      13-11-10 J.van Schouwen Initial creation.
 * 0.2      13-12-01 J.van Schouwen Complete working prototype of the DMXW
 *                                   node.
 * 0.21     14-08-10 J.van Schouwen Updated port mappings in the comments
 *                                   above.
 * 1.0      20-03-13 J.van Schouwen Updated to RFM69 library v1.0.0 and added
 *                                   more version info to startup console ouput.
 *                                   (Prototype v1.0 released as Release_3)
 *
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <RFM69.h>
#include <SPI.h>
#include "DMXWNet.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define COPYRIGHT     "(C)2020, A.J. van Schouwen"
#define SW_VERSION_c  "1.0 (2020-03-13)"
#define FW_VERSION_c  6   // Increment (with wraparound) for new F/W;
                          //   clears EEPROM.

//#define DEBUG_ON         // Uncomment to turn off debug output to serial port.
#define LOGGING_ON       // Uncomment to turn off packet logging to serial port.
#define SERIAL_CMDS_ENABLED // Enables command line at serial port

#define PIN_LOCATE    9    // Pin number of digital port connected to
                           // onboard LED (for location purposes)
#define MAX_SERIAL_BUF_LEN  20
#define EEPROM_FW_ADDR             0
#define EEPROM_NODEID_ADDR         1
#define EEPROM_VALIDITY_ADDR       2
#define EEPROM_FIRST_OPEN_ADDR     3

// Macro for defining strings that are stored in flash (program) memory rather
// than in RAM. Arduino defines the non-descript F("string") syntax.
#define FLASH(x) F(x)

#ifdef DEBUG_ON
  #define dbgPrint(x)    Serial.print(x)
  #define dbgPrintln(x)  Serial.println(x)
#else
  #define dbgPrint(x)
  #define dbgPrintln(x)
#endif

#ifdef LOGGING_ON
  #define logPrint(x)    Serial.print(x)
  #define logPrintln(x)  Serial.println(x)
#else
  #define logPrint(x)
  #define logPrintln(x)
#endif


// Node ID.
// Must be unique for each node. Gateway is always 1.
// Range: 2 - 20  (0 = 'undefined')
Uint8   myNodeId = 0;  // Must be unique for each node 

RFM69   radio;
bool    promiscuousMode = true;  // sniff all packets on network iff true
Uint8   dstNodeId;
Uint8   srcNodeId;

bool    resetEeprom = false;
bool    handleInput = false;
bool    ackRequested = false;
bool    dataToSend = false;
bool    requestAck = true;
bool    saveMappings = false;
Uint8   currReadPos = 0;
Uint8   command = 0;
Uint8   buffer[RF69_MAX_DATA_LEN];
Uint8   ackBuf[1];
Uint8   bufSize = 0;
char    serialBuffer[MAX_SERIAL_BUF_LEN];
Uint8   serialBufSize = 0;
Uint8   serialPos = 0;
long    rxTime = 0;
long    ackTime = 0;
bool    nodeIdValid = false;
unsigned long rxCount = 0;
Uint8   resetCount;
Uint16  badAddr = 0;

// Port to I/O pin mapping
NodePortMapRecord_t portMap[MAX_PORTS] =
{
  //{ inPin, outPin, conflictPort, isAnalog }
    {   2,     2,         -1,       false },  // Port 1
    {   3,     3,         11,       false },  // Port 2
    {   4,     4,         -1,       false },  // Port 3
    {   5,     5,         12,       false },  // Port 4
    {   6,     6,         13,       false },  // Port 5
    {   7,     7,         -1,       false },  // Port 6
    {   8,     8,         -1,       false },  // Port 7
    {   9,     9,         14,       false },  // Port 8
    {  12,    12,         -1,       false },  // Port 9
    {  13,    13,         -1,       false },  // Port 10
    {   0,     3,          2,       true  },  // Port 11
    {   1,     5,          4,       true  },  // Port 12
    {   2,     6,          5,       true  },  // Port 13
    {   3,     9,          8,       true  },  // Port 14
    {   4,    10,         -1,       true  },  // Port 15
    {   5,    11,         -1,       true  },  // Port 16
};


// List of defined ports indexed by DMXW channel # (-1 = not used)
DmxwNodeMapRecord_t nodeMap[MAX_DMXW_CHANS];

Uint8 node;

Int8 blinkState = 0;  // 0 = disabled; 1 = on; -1 = off
long blinkTime = 0;
long lastTime = 0;



void EepromLoad()
{
  int addr;
  bool dataIsValid;

  dataIsValid = EEPROM.read(EEPROM_VALIDITY_ADDR);
  if (!dataIsValid)
    return;
    
  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    nodeMap[i].port          = EEPROM.read(addr++);
    nodeMap[i].isOutput      = EEPROM.read(addr++);
    if (nodeMap[i].isOutput)
      pinMode(portMap[nodeMap[i].port - 1].outPin, OUTPUT);
    else
      pinMode(portMap[nodeMap[i].port - 1].inPin, INPUT);
    nodeMap[i].isLogarithmic = EEPROM.read(addr++);
    //nodeMap[i].value       = EEPROM.read(addr++); // Don't store the value
    nodeMap[i].value         = 0;
  }
}

void EepromSave()
{
  const Uint8 dataIsValid = 1;
  int addr;

  EEPROM.write(EEPROM_VALIDITY_ADDR, dataIsValid);
  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    EEPROM.write(addr++, nodeMap[i].port);
    EEPROM.write(addr++, nodeMap[i].isOutput);
    EEPROM.write(addr++, nodeMap[i].isLogarithmic);
    //EEPROM.write(addr++, nodeMap[i].value); // Don't store the value
  }
}


bool isPortMapValid(Uint8 port, bool isOutput)
{
  Int8 conflictPort;
  
  if ( (port == 0) || (port > MAX_PORTS) )
  {
    logPrint(FLASH("*** Port # out of range - "));
    logPrintln(port);
    return false;
  }

  // Check for a conflict
  conflictPort = portMap[port - 1].conflictPort;
  if ( (conflictPort != -1) && isOutput )
  {
    // We have a potential conflict ... but only if both ports are
    // output ports.
    for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
      if (nodeMap[i].port == conflictPort)
        if (nodeMap[i].isOutput)
        {
          logPrint(FLASH("*** DMXW Channel "));
          logPrint(i+1);
          logPrint(FLASH(" is mapped to conflicting output port "));
          logPrintln(conflictPort);
          return false;
        }
  }
  
  // Check for a duplicated port
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    if (nodeMap[i].port == port)
    {
      logPrint(FLASH("*** DMXW Channel "));
      logPrint(i+1);
      logPrint(FLASH(" already uses port #"));
      logPrintln(port);
      return false;
    }

  return true;
}


bool addNodeMap(Uint8 dmxwChan, Uint8 port, bool isOutput, bool logarithmic)
{
  DmxwNodeMapRecord_t *tmpMap;
  
  if ( (dmxwChan == 0) || (dmxwChan > MAX_DMXW_CHANS))
  {
    logPrint(FLASH("*** DMXW Chan# out of range - "));
    logPrintln(dmxwChan);
    return false;
  }
  dmxwChan--;
  if (!isPortMapValid(port, isOutput))
    return false;
    
  if (nodeMap[dmxwChan].port != -1)
  {
    logPrint(FLASH("*** DMXW Chan already mapped to port ["));
    logPrint(dmxwChan);
    logPrint(FLASH(" / "));
    logPrint(port);
    logPrintln(FLASH("]"));
    return false;
  }
  
  tmpMap = &nodeMap[dmxwChan];
  tmpMap->port          = port;
  tmpMap->isOutput      = isOutput;
  if (portMap[port-1].isAnalog)
    tmpMap->isLogarithmic = logarithmic;
  else
    tmpMap->isLogarithmic = 0;
  tmpMap->value         = 0;
  
  return true;
}

bool delNodeMap(Uint8 dmxwChan)
{
  if ( (dmxwChan == 0) || (dmxwChan > MAX_DMXW_CHANS))
  {
    logPrint(FLASH("*** DMXW Chan# out of range - "));
    logPrintln(dmxwChan);
    return false;
  }
  dmxwChan--;
  
  if (nodeMap[dmxwChan].port == -1)
    return false;
  nodeMap[dmxwChan].port = -1;
  return true;
}


Int8 getPortMapIdxByDmxwChan(int dmxwChan)
{
  Uint8 port;
  
  if ( (dmxwChan < 1) || (dmxwChan > MAX_DMXW_CHANS))
    return -1;
  port = nodeMap[dmxwChan - 1].port;
  if (port == -1)
    return -1;
  return (port - 1);
}


// Returns an adjusted PWM value (0..255) that provides a provides a
// perceived linear LED brightness relationship relative to the
// linearity assumed by lighting consoles generating DMX values in
// the range 0..255.
Uint8 linearLedValue(Uint8 dmx512Val)
{
  // coefficients
  double percentage = (double)dmx512Val/(double)255.0;
  double a = 9.7758463166360387E-01;
  double b = 5.5498961535023345E-00;

  return floor((a * exp(b*percentage) + 0.5)) - 1;
}


AckCode_t handleCmdRun()
{
  DmxwNodeMapRecord_t *currNodeMap = NULL;
  Uint8 value;
  Int8 port;
  Int8 pin;

  if (bufSize != (MAX_DMXW_CHANS + 1))
  {
    logPrintln(FLASH("CMD_RUN: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }
  
  #ifdef DEBUG_ON
    logPrint("RUN: [");
    for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    {
      logPrint(buffer[currReadPos + i]);
      logPrint(" ");
    }
    logPrintln();
  #endif

  for (Uint8 dmxwChan = 1; dmxwChan <= MAX_DMXW_CHANS; dmxwChan++)
  {
    //Serial.print("DMXW:"); Serial.print(dmxwChan);
    currNodeMap = &nodeMap[dmxwChan - 1]; // Chan map for DMXW chan, dmxwChan
    port = currNodeMap->port;  // Port assigned to dmxwChan
    //Serial.print(", Port:");  Serial.print(port);
    if (port > 0)
    {
      pin = portMap[port - 1].outPin;
      //Serial.print(", Pin:"); Serial.print(pin);
      if ( (pin == PIN_LOCATE) && blinkState )
      {
        logPrint(FLASH("\n*** Warning: skipping pin "));
        logPrint(PIN_LOCATE);
        logPrintln(FLASH(" DMXW update. Pin currently used as 'Locator'"));
        return ACK_OK;
      }
      if (pin != -1)
      {
        // We can write to the pin associated with the port
        value = buffer[dmxwChan];
        currReadPos++;  // Not really necessary, but good to keep it updated.
        //Serial.print(", Value:"); Serial.print(value);
        pinMode(pin, OUTPUT);
        if (portMap[port - 1].isAnalog)
        {
          if (currNodeMap->isLogarithmic)
            currNodeMap->value = linearLedValue(value);
          else
            currNodeMap->value = value;
          analogWrite(pin, currNodeMap->value);
        }
        else
        {
          currNodeMap->value = (value != 0);
          digitalWrite(pin, currNodeMap->value);
        }
      }
    }
    //Serial.println();
  }
  return ACK_OK;
}

AckCode_t handleCmdPing()
{
  node = srcNodeId;
  bufSize = 0;
  buffer[bufSize++] = CMD_PONG;
  dataToSend = true;
  requestAck = false;
  delay(50 * (myNodeId - 1));
  return ACK_OK;
}

AckCode_t handleCmdPong()
{
  // Not supported by node. Node doesn't send CMD_PING.
  dbgPrintln(FLASH("*** Error: Pong received."));
  return ACK_ECMD;
}

AckCode_t handleCmdMap()
{
  Int8  dmxwChan    = buffer[currReadPos++];
  Int8  port        = buffer[currReadPos++];
  Uint8 logarithmic = buffer[currReadPos++];
  
  if (bufSize != currReadPos)
  {
    logPrintln(FLASH("CMD_MAP: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }
  
  if (addNodeMap(dmxwChan, port, true, logarithmic))
  {
    pinMode(portMap[port - 1].outPin, OUTPUT);
    return ACK_OK;
  }

  return ACK_EPORT; 
}

AckCode_t handleCmdMapR()
{
  Int8 dmxwChan = buffer[currReadPos++];
  
  if (bufSize != currReadPos)
  {
    logPrintln(FLASH("CMD_MAPR: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }
  
  if (delNodeMap(dmxwChan))
    return ACK_OK;
    
  return ACK_EDMXW;
}

AckCode_t handleCmdClrAll()
{
  DmxwNodeMapRecord_t tmpRec;
  
  tmpRec.port     = -1;
  tmpRec.isOutput = true;
  tmpRec.value    = 0;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    nodeMap[i] = tmpRec;
  return ACK_OK;
}

AckCode_t handleCmdEcho()
{
  Int8 dmxwChan = buffer[currReadPos++];
  Int8 portIdx;

  if (bufSize != currReadPos)
  {
    logPrintln(FLASH("CMD_ECHO: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }

  portIdx = getPortMapIdxByDmxwChan(dmxwChan);
  if (portIdx == -1)
    return ACK_EPORT;
  bufSize = 0;
  buffer[bufSize++] = CMD_CHAN;
  buffer[bufSize++] = dmxwChan;
  buffer[bufSize++] = portIdx + 1;
  buffer[bufSize++] = portMap[portIdx].outPin;
  buffer[bufSize++] = portMap[portIdx].conflictPort;
  buffer[bufSize++] = portMap[portIdx].isAnalog;
  buffer[bufSize++] = nodeMap[dmxwChan - 1].value;
  buffer[bufSize++] = nodeMap[dmxwChan - 1].isLogarithmic;
  node = BROADCASTID;
  dataToSend = true;
  return ACK_OK;
}

AckCode_t handleCmdChan()
{
  //Not supported by node.
  return ACK_ECMD;
}

AckCode_t handleCmdLoc()
{
  digitalWrite(PIN_LOCATE, HIGH);
  blinkState = 1;
  blinkTime = millis();
  return ACK_OK;
}

AckCode_t handleCmdOff()
{
  blinkState = 0;
  for (Uint8 i = 0; i < MAX_PORTS; i++)
  {
    if (portMap[i].isAnalog)
      analogWrite(portMap[i].outPin, 0);
    else
      digitalWrite(portMap[i].outPin, LOW);
  }
  return ACK_OK;
}

AckCode_t handleCmdPort()
{
  Uint8 port   = buffer[currReadPos++];
  Uint8 value  = buffer[currReadPos++];
  Int8  pin;

  if (bufSize != currReadPos)
  {
    logPrintln(FLASH("CMD_PORT: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }

  if ( (port > 0) && (port <= MAX_PORTS) )
  {
    pin = portMap[port - 1].outPin;
    if ( (pin == PIN_LOCATE) && blinkState )
      blinkState = -1;
      
    // We can write to the pin associated with the port
    if (portMap[port - 1].isAnalog)
      analogWrite(pin, value);
    else
      digitalWrite(pin, value);
  }
  else
    return ACK_EPORT;

  return ACK_OK;
}

AckCode_t handleCmdCtrl()
{
  Uint8 dmxwChan = buffer[currReadPos++];
  Uint8 value    = buffer[currReadPos++];
  Int8  port;
  Int8  pin;
  DmxwNodeMapRecord_t *currNodeMap = NULL;

  if (bufSize != currReadPos)
  {
    logPrintln(FLASH("CMD_CTRL: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }

  if ((dmxwChan == 0) || (dmxwChan > MAX_DMXW_CHANS))
  {
    logPrint(FLASH("CMD_CTRL: Bad DMXW Chan #"));
    logPrintln(dmxwChan);
    return ACK_EDMXW;
  }
  
  currNodeMap = &nodeMap[dmxwChan - 1]; // Chan map for DMXW chan, dmxwChan
  port = currNodeMap->port;  // Port assigned to dmxwChan
  if (port > 0)
  {
    pin = portMap[port - 1].outPin;
    if ( (pin == PIN_LOCATE) && blinkState )
      blinkState = -1;
      
    if (pin != -1)
    {
      // We can write to the pin associated with the port
      currNodeMap->value = value;
      if (portMap[port - 1].isAnalog)
        analogWrite(pin, value);
      else
        digitalWrite(pin, value);
    }
  }
  else
    return ACK_EPORT;
    
  return ACK_OK;
}

AckCode_t handleCmdTest()
{
  logPrintln(FLASH("Test command received. Nothing to do."));
  return ACK_OK;
}

AckCode_t handleCmdSave()
{
  if (bufSize != 1)
  {
    logPrintln(FLASH("CMD_SAVE: Packet dropped--corrupted"));
    return ACK_NULL;  // Buffer is corrupted. Drop the DMXW packet.
  }

  saveMappings = true;
  logPrintln(FLASH("CMD_SAVE: Node mapping saved to EEPROM."));
  return ACK_OK;
}

AckCode_t handleCmdUndef()
{
  // Someone failed to set their command code to a valid value.
  logPrintln(FLASH("CMD_UNDEF is an invalid command."));
  return ACK_ECMD;
}


// Incoming message from the wireless network of nodes
AckCode_t handleNetRxMessage(Uint8 command)
{
  AckCode_t ret = ACK_ERR;

  switch (command)
  {
    case CMD_RUN:    ret = handleCmdRun();       break;
    case CMD_PING:   ret = handleCmdPing();      break;
    case CMD_PONG:   ret = handleCmdPong();      break;
    case CMD_MAP:    ret = handleCmdMap();       break;
    case CMD_MAPR:   ret = handleCmdMapR();      break;
    case CMD_CLRALL: ret = handleCmdClrAll();    break;
    case CMD_ECHO:   ret = handleCmdEcho();      break;
    case CMD_CHAN:   ret = handleCmdChan();      break;
    case CMD_LOC:    ret = handleCmdLoc();       break;
    case CMD_OFF:    ret = handleCmdOff();       break;
    case CMD_PORT:   ret = handleCmdPort();      break;
    case CMD_CTRL:   ret = handleCmdCtrl();      break;
    case CMD_TEST:   ret = handleCmdTest();      break;
    case CMD_SAVE:   ret = handleCmdSave();      break;
    case CMD_UNDEF:  ret = handleCmdUndef();     break;
    default:
      logPrint(FLASH("*** Command not recognized ["));
      logPrint(command);
      logPrint("]");
      return ACK_ECMD;
  }
  if (ret == ACK_ECMD)
  {
    logPrint(FLASH("... Command unsupported for Node ..."));
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
//JVS??
if (command != CMD_RUN)
{
  logPrint(FLASH("RX cmd["));
  logPrint(command);
  logPrint(FLASH("]  ["));
  for (int i = 0; i < bufSize; i++)
  {
    logPrint(buffer[i]);
    logPrint(FLASH("  "));
  }
  logPrintln(FLASH("]"));
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
  long sentTime;

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


#ifdef SERIAL_CMDS_ENABLED

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
  logPrintln(); logPrintln();
  logPrintln(FLASH("Serial port commands are enabled. Commands are:"));
  logPrintln(FLASH("  (spaces may replaces commas)"));
  logPrintln(FLASH("<x> in {1,...,512};  <d>,<n> in {1,...,20};"));
  logPrintln(FLASH("<p> in {1,...,16};   <v> in {0,...,255}"));
  logPrintln(FLASH("  d <d>, <v>        - Simulate receipt of value v for "
                                             "for DMXW channel #d."));
  logPrintln(FLASH("  free              - display free RAM"));
  logPrintln(FLASH("  h                 - Print this help text"));
  logPrintln(FLASH("  n <d>, <p>, <l>   - Map DMXW chan d to port p, with "
                                         "values to be scaled "));
  logPrintln(FLASH("                       logarithmically if l=1"));
  logPrintln(FLASH("  p                 - Display the Port Mapping."));
  logPrintln(FLASH("  r <d>             - Remove map for DMXW chan d."));
  logPrintln(FLASH("  s                 - Show DMXW channel mapping."));
  logPrintln(FLASH("  nodeid <n>        - Set Node Id to n, n in "
                                             "{2, 3, ..., 20} unique"));
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
  char   tabChar = '\t';
  Uint8  port = 0;
  Uint8  val = 0;
  Uint8  dmxwChan = 0;
  Uint8  logarithmic = 0;
  Uint8  idx = 0;
  bool   cmdToProcess = false;
  bool   cmdInvalid = false;
  DmxwNodeMapRecord_t *tmp;
  bool   serialDone = false;

/*
  if (Serial.available() > 0)
  {
    Serial.setTimeout(30000);
    if (Serial.readBytesUntil('#', serialBuffer, MAX_SERIAL_BUF_LEN) == 0)
      return;
*/
  if (Serial.available() > 0)
  {
    serialBuffer[serialPos] = Serial.read();
    if ( (serialBuffer[serialPos] == '\b') && (serialPos > 0) )
    {
      serialPos--;
    }
    else if (serialBuffer[serialPos] == '\r')
    {
      serialBuffer[serialPos] = '\0';
      serialBufSize = serialPos;
      cmdToProcess = true;
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
    
    if (!nodeIdValid)
    {
      logPrintln(FLASH("\n\nYou must power cycle the node first!"));
      serialPos = 0;
      return;
    }
    
    serialPos = 0;
    logPrint(FLASH("Command: ["));
    logPrint(serialBuffer);
    logPrintln("]");
    bufSize = 0;
    switch (serialBuffer[serialPos++])
    {
      case 'd':
        // d <d>, <v>
        // Simulate receipt of a DMXW CMD_RUN packet in which channel d
        // has value v.
        dmxwChan = serialParseInt();
        val      = serialParseInt();
        if ( (dmxwChan < 1) || (dmxwChan > MAX_DMXW_CHANS) )
        {
          logPrintln(FLASH("*** Chan # out of range"));
          break;
        }
        nodeMap[dmxwChan - 1].value = val;
        memset(buffer, 0, sizeof(buffer));
        command = CMD_RUN;
        buffer[bufSize++] = command;
        for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
          buffer[bufSize++] = nodeMap[i].value;
        logPrint(FLASH("Buffer: Size["));
        logPrint(bufSize);
        logPrint(FLASH("]  ["));
        for (Uint8 i = 0; i < bufSize; i++)
        {
          logPrint(buffer[i]);
          logPrint(" ");
        }
        logPrintln("]");
        handleNetRxMessage(command);
        logPrintln(FLASH("CMD_RUN command processed"));
        break;
        
      case 'f':
        if (strstr(serialBuffer, "free") != null)
          CheckRam();
        break;
         
      case 'h':
        // h
        // Display serial command help text.
        showSerialHelp();
        break;
        
      case 'n':
        if (strstr(serialBuffer, "nodeid") != null)
        {
          // nodeid <n> 
          // Set Node Id to n, n in {2, 3, ..., 20} unique.
          serialPos += 5;
          myNodeId = serialParseInt();
          // Node ID must be in the range {2, 3, ..., NODEID_MAX}
          if ( (myNodeId > 1) && (myNodeId <= NODEID_MAX) )
          {
            EEPROM.write(EEPROM_NODEID_ADDR, myNodeId);
            EEPROM.write(EEPROM_VALIDITY_ADDR, 0); // ID change invalidates data
            nodeIdValid = false;
            logPrint(FLASH("Node ID #"));
            logPrint(myNodeId);
            logPrintln(FLASH(" saved to EEPROM.\nPOWER CYCLE THE NODE."));
          }
          else
          {
            logPrintln(FLASH("*** Node ID value out of range."));
          }
        }
        else
        {
          // n <d>, <p>
          // Map DMXW chan d to port p.
          dmxwChan    = serialParseInt();
          port        = serialParseInt();
          logarithmic = serialParseInt();
          if (addNodeMap(dmxwChan, port, true, logarithmic))
          {
            pinMode(portMap[port - 1].outPin, OUTPUT);
            logPrint(FLASH("Channel "));
            logPrint(dmxwChan);
            logPrint(FLASH(" is mapped to port "));
            logPrintln(port);
          }
          else
          {
            logPrint(FLASH("*** Couldn't add mapping"));
          }
        }
        break;
        
      case 'p':
        // p
        // Display the Port Mapping
        logPrintln();
        logPrintln(FLASH("DMXW"));
        logPrintln(FLASH("Port #\tIn Pin\tOut Pin\tConflict  Analog?"));
        logPrintln(FLASH("------\t------\t-------\t--------  -------"));
        for (Uint8 i = 0; i < MAX_PORTS; i++)
        {
          logPrint(i + 1); logPrint(tabChar);
          logPrint(portMap[i].inPin); logPrint(tabChar);
          logPrint(portMap[i].outPin); logPrint(tabChar);
          if (portMap[i].conflictPort != -1)
            logPrint(portMap[i].conflictPort);
          logPrint(tabChar); logPrint("  ");
          logPrintln(portMap[i].isAnalog ? "Y" : "N");
        }
        logPrintln(FLASH("----------------------------------------"));
        logPrintln();
        break;
        
      case 'r':
        // r <d>
        // Remove port mapping for DMXW chan d.
        dmxwChan = serialParseInt();
        if (delNodeMap(dmxwChan))
          logPrintln(FLASH("Mapping removed"));
        else
          logPrintln(FLASH("Unable to remove mapping."));
        break;
        
      case 's':
        // s
        // Show the DMXW channel mapping
        logPrintln();
        logPrint(FLASH("DMX Channel Mapping for Node #"));
        logPrintln(myNodeId);
        logPrintln(FLASH("DMXW Chan\tPort\tOut Pin\tAnalog?\tLog?\tValue"));
        logPrintln(FLASH("---------\t----\t-------\t-------\t----\t-----"));
        for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
        {
          if (nodeMap[i].port != -1)
          {
            port = nodeMap[i].port - 1;
            logPrint(i + 1); logPrint(tabChar); logPrint(tabChar);
            logPrint(port + 1); logPrint(tabChar);
            logPrint(portMap[port].outPin); logPrint(tabChar);
            logPrint(portMap[port].isAnalog ? "Y" : "N"); logPrint(tabChar);
            logPrint(nodeMap[i].isLogarithmic ? "Y" : "N"); logPrint(tabChar);
            logPrintln(nodeMap[i].value);
          }
        }
        logPrintln(FLASH("-----------------------------------------------------"));
        logPrintln();
        break;

      default:
        logPrint(FLASH("Invalid command '"));
        logPrint(serialBuffer[0]);
        logPrintln("'");
    }
    
    serialPos = 0;
    
    if (cmdInvalid)
      logPrintln(FLASH("Invalid command."));
    serialBufSize = 0;
    memset(serialBuffer, 0, sizeof(serialBuffer));
//CheckRam();
  }
}
#endif

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
    Serial.print(F("Free RAM: "));
    Serial.println(freeValue);
  #endif
}


void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(10);
  myNodeId = EEPROM.read(EEPROM_NODEID_ADDR);
  nodeIdValid = true;
  radio.initialize(FREQUENCY, myNodeId, NETWORKID);
  #ifdef IS_RFM69HW
    radio.setHighPower();
  #endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  currReadPos = 0;
  for (Uint8 i = 0; i < RF69_MAX_DATA_LEN; i++)
    buffer[i] = 0;
  
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    nodeMap[i].port = -1;
  }

  resetCount = EEPROM.read(1023) + 1;
  
  // Check if we need to clear out EEPROM and start from scratch
  resetEeprom = (EEPROM.read(EEPROM_FW_ADDR) != FW_VERSION_c);
  if (resetEeprom)
  {
    // Record new F/W version and start with blank mappings
    EEPROM.write(EEPROM_FW_ADDR, FW_VERSION_c);
    EEPROM.write(EEPROM_VALIDITY_ADDR, 0);  // dataIsValid = false;
    resetCount = 0;
  }
  else
  {
    // Read in mappings stored in EEPROM
    EepromLoad();
  }
  EEPROM.write(1023, resetCount);

  Serial.print(FLASH("\nDMX Wireless Network...\t\tNode #"));
  Serial.print(myNodeId);
  Serial.print(FLASH("   (Slave)   Radio frequency: "));
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(FLASH("Mhz"));
  Serial.print(COPYRIGHT);
  Serial.print(FLASH("\t#Resets: "));
  Serial.print(resetCount);
  Serial.println(FLASH("\t\t"));
  Serial.print(FLASH("Node S/W: "));
  Serial.print(SW_VERSION_c);
  Serial.print(FLASH("\tF/W: "));
  Serial.print(FW_VERSION_c);
  Serial.println();
  CheckRam();
  
  #ifdef SERIAL_CMDS_ENABLED
    showSerialHelp();
  #endif
  
  pinMode(9, OUTPUT);  
}


void loop()
{
  handleInput = false;
  bufSize = 0;
  dataToSend = false;
  requestAck = true;
  ackBuf[0] = ACK_ERR;
  
  // Handle incoming messages.
  if (radio.receiveDone())
  {
    // Determine if the message is aimed at us.
    dstNodeId = radio.TARGETID;
    srcNodeId = radio.SENDERID;
    // Handle the RX msg if it's intended for me
    if ( ( (dstNodeId == myNodeId) && (dstNodeId > NODEID_UNDEF) ) ||
         ( (dstNodeId == BROADCASTID) && (srcNodeId == GATEWAYID) ) )
    {
      if ( (radio.DATA[0] == srcNodeId) && (radio.DATA[1] == dstNodeId))
      {
        bufSize = radio.DATALEN - 2;
        if (bufSize <= (MAX_DMXW_CHANS + 1))
        {
          handleInput = true;
          ackRequested = radio.ACK_REQUESTED;
          memcpy(buffer, (const void *)&radio.DATA[2], bufSize);
        }
        else
        {
          logPrintln(FLASH("*** RX invalid pkt length; dropped"));
        }
      }
      else
      {
//JVS??
        bufSize = radio.DATALEN - 2;
        memcpy(buffer, (const void *)&radio.DATA[2], bufSize);
//
        logPrint(FLASH("*** RX corrupt pkt; dropped. ["));
        for (int i = 0; i < bufSize; i++)
        {
          logPrint(buffer[i]);
          logPrint(FLASH("  "));
        }
        logPrintln(FLASH("]"));
      }
    }
    else
    {
      badAddr++;
      dbgPrint(FLASH(" ... ignored (pkt not for me). Src:"));
      dbgPrint(srcNodeId);
      dbgPrint(FLASH(" Dst:"));
      dbgPrint(dstNodeId);
      dbgPrintln(FLASH(")"));
//JVS??  vvv
/*        bufSize = radio.DATALEN - 2;
        memcpy(buffer, (const void *)&radio.DATA[2], 30);
*/
      logPrintln(FLASH(" ... ignored (pkt not for me)"));
/*
      logPrint(srcNodeId);
      logPrint(FLASH(" Dst:"));
      logPrint(dstNodeId);
      logPrint(FLASH("  #Bad:"));
      logPrint(badAddr);
      logPrint(FLASH(")  ["));
for (int i = 0; i < 30; i++)
{
  logPrint(buffer[i]);
  logPrint("  ");
}
logPrintln("]");
*/
//JVS??  ^^^
    }
    
    dbgPrint(FLASH("RX  Src["));
    dbgPrint(radio.SENDERID);
    dbgPrint(FLASH("]  Dst["));
    dbgPrint(radio.TARGETID);
    dbgPrint(FLASH("]  Size["));
    dbgPrint(radio.DATALEN);
    dbgPrint(FLASH("]  "));
    
    /*Serial.print(FLASH(" DATA["));
    for (Uint8 i = 0; i < radio.DATALEN; i++)
    {
      Serial.print(radio.DATA[i]); Serial.print(" ");
    }
    Serial.print("]  ");
    */
  
    if (handleInput)
    {
      rxTime = millis();
      currReadPos = 0;
      command = buffer[currReadPos++];

      dbgPrint(FLASH("Cmd["));
      printCommand(command);
      dbgPrint("] ");
      rxCount++;
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
  
  #ifdef SERIAL_CMDS_ENABLED
    getSerialCommand();
  #endif

  if (saveMappings)
  {
    saveMappings = false;
    EepromSave();
  }
  
  if (dataToSend)
  {
//JVS??
    if ((bufSize + 2) <= RF69_MAX_DATA_LEN)
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
      dbgPrint(FLASH("  Result["));
      printAckResult(ackBuf[0]);
      dbgPrintln("]");
    }
  }
  
//JVS??
  if (handleInput && (rxCount % 1000) == 0)
  {
    logPrint(FLASH("rxCount:"));
    logPrint(rxCount);
    logPrint(FLASH("\t"));
    CheckRam();
  }

  lastTime = millis();
  if (blinkState)
  {
    if ((lastTime - blinkTime) > 500)
    {
      blinkTime = lastTime;
      blinkState = -blinkState;
      if (blinkState == 1)
        digitalWrite(PIN_LOCATE, HIGH);
      else
        digitalWrite(PIN_LOCATE, LOW);
    }
  }
  delayMicroseconds(2000);
}
