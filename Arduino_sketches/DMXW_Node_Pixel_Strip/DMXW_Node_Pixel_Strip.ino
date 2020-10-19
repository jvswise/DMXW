/******************************************************************************
 * DMXW_Node_Address_LED_Strip
 *
 * DMXW remote node for adressable LED strip.
 *
 * This sketches implements a DMX wireless remote node that interacts with the
 * DMS wireless gateway.
 *
 * All ports specify a lighting effect and effect parameters rather than
 * directly controlling individual output pins. Therefore, all ports are
 * assigned (by default) to output pin 4 which is used to control the
 * addressable LED strip. (The output pin can be changed to any pin in
 * the ranges 3-9, and 14-21.)
 *
 *         Port #  Pin # (Moteino)
 *         ------  ---------------
 *           1     4
 *           2     4
 *           3     4
 *           ...   ...
 *           16    4
 *
 * Port Definitions:
 * ----------------
 *
 *   Port 1  - Delay (in milliseconds; 0=LEDs off; 255=hold current settings)
 *   Port 2  - Effect:
 *               (Decade bands are used in order to ease manual config via
 *                potentiometers (e.g. on the DMXW Gateway))
 *               1: 10 - 19 = Colour Wipe
 *               2: 20 - 29 = Rainbow
 *               3: 30 - 39 = Rainbow Cycle
 *               4: 40 - 49 = Theatre Chase
 *               5: 50 - 59 = Theatre Chase Rainbow
 *               6: 60 - 69 = Water and Embers
 *               7: 70 - 79 = Twinkle
 *   Port 3   - Argument #1
 *   Port 4   - Argument #2
 *   Port 5   - Argument #3
 *   Port 6   - Argument #4
 *   Port 7   - Argument #5
 *   Port 7   - Argument #6
 *   (All other ports are undefined. DMX512 values assigned to those ports are
 *   ignored.)
 *
 * Effect Parameters: (All parameters are Uint8 values. [Delay] comes from
 *                     Port 1)
 * -----------------
 *   [x]<Effect>            ( Arg1, Arg2, ..., ArgN, [Delay] )
 *      -----------------   ---------------------------------------------
 *   [1]ColourWipe          ( Red, Green, Blue, [Delay] )
 *
 *   [2]Rainbow             ( [Delay] )
 *
 *   [3]RainbowCycle        ( [Delay] )
 *
 *   [4]TheatreChase        ( Red, Green, Blue, [Delay] )
 *
 *   [5]TheatreChaseRainbow ( [Delay] )
 *
 *   [6]WaterAndEmbers      ( RedLow, GreenLow, BlueLow,
 *                            RedHigh, GreenHigh, BlueHigh,
 *                            Depth, [Delay]
 *                          )
 *
 *   [7]Twinkle             ( Red, Green, Blue, Min[=1], Max[=100],
 *                            Simul[=5], Hold[=20]
 *                          )
 *                             Min = Min delay (x 10ms) before next twinkle
 *                             Max = Max delay (x 10ms) before next twinkle
 *                             Simul = Max # LEDs (triples) allowed to twinkle
 *                                     simultaneously
 *                             Hold = Duration (ms) for which twinkle is held
 *                             - defaulted values [=N] are used for arguments
 *                               with zero value
 *                             
 * When Delay is set to 255, other parameters can change without affecting
 * the current effect until Delay < 255.
 *
 *
 * Notes:
 * =====
 *   Special EEPROM memory locations:
 *      Address   Description
 *      -------   ----------------------------------
 *         0      Firmware Version
 *         1      Node ID (recorded in myNodeId)
 *         2      Mapping data validity (1=valid; 0=invalid)
 *         3      Addressable LED strip frequency (8 = 800 KHz, 4 = 400 KHz)
 *         4      Addressable LED strip LED wiring order (1 = GRB, 2 = RGB)
 *         5      Addressable LED strip length (# controlled tricolour
 *                  LED elements)
 *   Arduino Serial Monitor settings for console I/O:
 *     - 9600 baud
 *     - "Carriage return" as line ending
 *   LED strip wiring:
 *     The LED strips typically require a 5 Vdc signal voltage on the
 *     digital output control pin (ledStripCtrlPin). You should be able to get
 *     away with 3.3 Vdc (such as on a Moteino). (JVS: 3.3 Vdc does
 *     work for me.) The recommendation is, however, 5 Vdc. For 3.3 Vdc,
 *     a high speed 3.3 volt to 5 volt level conversion circuit is
 *     recommended. Also, it is suggested to follow the important Adafruit
 *     notes below.
 *
 * IMPORTANT [Adafruit note, assumes 5 Vdc digital output port voltages]:
 * =========
 *   To reduce NeoPixel burnout risk, add 1000 uF capacitor across pixel
 *   power leads, add 300 - 500 Ohm resistor on first pixel's data input
 *   and minimize distance between Arduino and first pixel. Avoid connecting
 *   on a live circuit...if you must, connect GND first.
 *
 * *** IMPORTANT ***
 * =================
 *   For some reason, when debug logging or console logging is enabled,
 *   it interferes with the control of the addressable LED strip (e.g.
 *   colours appear to be significantly dimmer than they should be).
 *   When operating normally, ensure that the flash is loaded with
 *   with a software version that is compiled without any of DEBUG_ON
 *   LOGGING_ON, or SERIAL_CMDS_ENABLED defined (comment out their #define
 *   statements). With SERIAL_CMDS_ENABLED defined, console input is still
 *   enabled and functions even though there is no feedback; the LEDs will
 *   glow noticeably dimmer. With LOGGING_ON defined, command feedback
 *   can be seen on the console; the LEDs will glow quite dimly.
 *
 *   When initially configuring a node--to set the nodeid, the LED strip
 *   length and, if necessary, change digital output pin used for
 *   controlling the strip--enable SERIAL_CMDS_ENABLED and LOGGING_ON.
 *   Once initial configuration has been completed, SAVEd, and tested,
 *   reflash the device with LOGGING_ON, SERIAL_CMDS_ENABLED, and
 *   DEBUG_ON disabled (comment out their #define statements).
 *
 *   FOLLOW-UP: The problem may be related to calling CheckRam() within
 *              setup().
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
 * 0.21A    14-12-16 J.van Schouwen Modified generic remote node for
 *                                   testing addressable LED strip
 * 1.0      16-01-07 J.van Schouwen Cleaning up code for addressable LED strip
 *                                   remote DMXW node.
 * 1.1      20-03-13 J.van Schouwen Updated from RFM69 library v1.0.0.
 *                                   (Prototype v1.1 released as Release_3.)
 *
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <RFM69.h>
#include <SPI.h>
#include "DMXWNet.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define COPYRIGHT     "(C)2020, A.J. van Schouwen"
#define SW_VERSION_c  "1.1 (2020-03-13)"
#define FW_VERSION_c  7   // Increment (with wraparound) for new F/W;
                          //   clears EEPROM.

//#define DEBUG_ON            // Uncomment to turn off debug output to serial port.
#define LOGGING_ON          // Uncomment to turn off packet logging to serial port.
#define SERIAL_CMDS_ENABLED // Enables command line at serial port

#define PIN_LOCATE    9   // Pin number of digital port connected to
                          // onboard LED (for location purposes)
#define NEO_PIN       4   // Default digital pin for LED strip control
                          // (overidden by ledStripCtrlPin)

#define MAX_SERIAL_BUF_LEN  20
#define EEPROM_FW_ADDR             0
#define EEPROM_NODEID_ADDR         1
#define EEPROM_VALIDITY_ADDR       2
#define EEPROM_STRIP_CTRL_PIN      3
#define EEPROM_STRIP_FREQ_ADDR     4
#define EEPROM_STRIP_WIRING_ADDR   5
#define EEPROM_STRIP_LEN_ADDR      6
#define EEPROM_FIRST_OPEN_ADDR     7

#define SERIAL_BAUD                4800

#define DEL_CHAR                   0x7F  // ASCII Del character


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


Adafruit_NeoPixel *strip;

// IMPORTANT [Adafruit note]: To reduce NeoPixel burnout risk, add 1000 uF
// capacitor across pixel power leads, add 300 - 500 Ohm resistor on first
// pixel's data input and minimize distance between Arduino and first pixel.
// Avoid connecting on a live circuit...if you must, connect GND first.


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
long    nextFxTime = 0;
long    currentTime = 0;


// Port to I/O pin mapping
NodePortMapRecord_t portMap[MAX_PORTS] =
{
  //JVS: Pins are now disconnected from nodeMap[]--i.e. no direct DMXW control
  //     Control the LED strip parameters through the DMXW ports
  //     Below, the outPin assignments are the default (NEO_PIN); these
  //     get altered in EepromLoad().
  //
  //         conflict   is
  //{ outPin,  Port,  Analog, name}
    { NEO_PIN,   -1,   false, "Speed"  }, // Port 1 - Speed (0 = off)
    { NEO_PIN,   -1,   false, "Effect" }, // Port 2 - Effect:
                                          //           1 = Colour Wipe
                                          //           2 = Rainbow
                                          //           3 = Rainbow Cycle
                                          //           4 = Theatre Chase
                                          //           5 = Theatre Chase Rainbow
                                          //           6 = Water Effect
    { NEO_PIN,   -1,   false, "Arg #1" }, // Port 3   - Argument #1
    { NEO_PIN,   -1,   false, "Arg #2" }, // Port 4   - Argument #2
    { NEO_PIN,   -1,   false, "Arg #3" }, // Port 5   - Argument #3
    { NEO_PIN,   -1,   false, "Arg #4" }, // Port 6   - Argument #4
    { NEO_PIN,   -1,   false, "Arg #5" }, // Port 7   - Argument #5
    { NEO_PIN,   -1,   false, "Arg #6" }, // Port 8   - Argument #6
    { NEO_PIN,   -1,   false, "Arg #7" }, // Port 9   - Argument #7
    { NEO_PIN,   -1,   false, " -"     }, // Port 10
    { NEO_PIN,   -1,   false, " -"     }, // Port 11
    { NEO_PIN,   -1,   false, " -"     }, // Port 12
    { NEO_PIN,   -1,   false, " -"     }, // Port 13
    { NEO_PIN,   -1,   false, " -"     }, // Port 14
    { NEO_PIN,   -1,   false, " -"     }, // Port 15
    { NEO_PIN,   -1,   false, " -"     }, // Port 16
};

const Uint8 stripLastPortNum = 9; // Last port # actually used above


// LED strip parameters
Uint8   stripDelay, stripEffect;
Uint8   stripArg1, stripArg2, stripArg3, stripArg4;
Uint8   stripArg5, stripArg6, stripArg7;
Uint8   oldStripEffect = 0;
bool    stripParamChange = false;
bool    newEffect = false;
bool    disableEffects = false;
Uint8   ledStripFlags;

// LED strip configuration settings
Int8   ledStripCtrlPin;  // Configured output pin for LED strip control
                         //   (Default is NEO_PIN.)
                         
Uint8   ledStripFreq;    // Configured LED strip frequency
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)

Uint8   ledStripWiring;  // Configured LED strip colour wiring order
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //     JVS: On the 12V strip that I have, change the colour order in functions
  //          calls from R/G/B to R/B/G

Uint8   ledStripLen;     // # of LED triplets (12V strip has 3 tricolour
                         //   LEDs per WS2811 driver). So the LEDs, in this
                         //   configuration, aren't really individually
                         //   addressable.
                        

// Array, indexed by port # (0 = port #1), mapping port # to DMXW channel #.
Uint8 portToDmxwMap[MAX_PORTS];



// List of defined ports indexed by DMXW channel # (-1 = not used; 0 = chan #1)
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
  ledStripCtrlPin = NEO_PIN;

  if (!dataIsValid)
    return;

  ledStripCtrlPin = EEPROM.read(EEPROM_STRIP_CTRL_PIN);
  ledStripFreq    = EEPROM.read(EEPROM_STRIP_FREQ_ADDR);
  ledStripWiring  = EEPROM.read(EEPROM_STRIP_WIRING_ADDR);
  ledStripLen     = EEPROM.read(EEPROM_STRIP_LEN_ADDR);
      
  for (Uint8 i = 0; i < MAX_PORTS; i++)
  {
    portMap[i].outPin = ledStripCtrlPin;
  }
  
  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    nodeMap[i].port          = EEPROM.read(addr++);
    nodeMap[i].isLogarithmic = EEPROM.read(addr++);
    //nodeMap[i].value       = EEPROM.read(addr++); // Don't store the value
    nodeMap[i].value         = 0;
  }
  
  for (Uint8 i = 0; i < MAX_PORTS; i++)
  {
    portToDmxwMap[i] = EEPROM.read(addr++);
  }
  pinMode(ledStripCtrlPin, OUTPUT);
}

void EepromSave()
{
  const Uint8 dataIsValid = 1;
  int addr;

  EEPROM.write(EEPROM_VALIDITY_ADDR,     dataIsValid);
  EEPROM.write(EEPROM_STRIP_CTRL_PIN,    ledStripCtrlPin);
  EEPROM.write(EEPROM_STRIP_FREQ_ADDR,   ledStripFreq);
  EEPROM.write(EEPROM_STRIP_WIRING_ADDR, ledStripWiring);
  EEPROM.write(EEPROM_STRIP_LEN_ADDR,    ledStripLen);

  addr = EEPROM_FIRST_OPEN_ADDR;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
  {
    EEPROM.write(addr++, nodeMap[i].port);
    EEPROM.write(addr++, nodeMap[i].isLogarithmic);
    //EEPROM.write(addr++, nodeMap[i].value); // Don't store the value
  }
  for (Uint8 i = 0; i < MAX_PORTS; i++)
  {
    EEPROM.write(addr++, portToDmxwMap[i]);
  }
}


bool isPortMapValid(Uint8 port)
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
  if (conflictPort != -1)
  {
    // We have a potential conflict ... but only if both ports are
    // output ports.
    for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    {
      if (nodeMap[i].port == conflictPort)
      {
        logPrint(FLASH("*** DMXW Channel "));
        logPrint(i+1);
        logPrint(FLASH(" is mapped to conflicting output port "));
        logPrintln(conflictPort);
        return false;
      }
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


bool addNodeMap(Uint8 dmxwChan, Uint8 port, bool logarithmic)
{
  DmxwNodeMapRecord_t *tmpMap;
  
  if ( (dmxwChan == 0) || (dmxwChan > MAX_DMXW_CHANS))
  {
    logPrint(FLASH("*** DMXW Chan# out of range - "));
    logPrintln(dmxwChan);
    return false;
  }
  dmxwChan--;
  if (!isPortMapValid(port))
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
  if (portMap[port-1].isAnalog)
    tmpMap->isLogarithmic = logarithmic;
  else
    tmpMap->isLogarithmic = 0;
  tmpMap->value         = 0;
  portToDmxwMap[port-1] = dmxwChan + 1;
  
  return true;
}

bool delNodeMap(Uint8 dmxwChan)
{
  Uint8 port;
  
  if ( (dmxwChan == 0) || (dmxwChan > MAX_DMXW_CHANS))
  {
    logPrint(FLASH("*** DMXW Chan# out of range - "));
    logPrintln(dmxwChan);
    return false;
  }
  dmxwChan--;
  
  if (nodeMap[dmxwChan].port == -1)
  {
    return false;
  }
  port = nodeMap[dmxwChan].port;
  nodeMap[dmxwChan].port = -1;
  portToDmxwMap[port-1] = 0;
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
  Uint8 value, oldValue;
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

  stripParamChange = false;
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
        /* We can check for a value change on the port. If there's
         * a change, set a flag to trigger subsequent handling of
         * effect and parameters changes.
         */
        value = buffer[dmxwChan];
        currReadPos++;  // Not really necessary, but good to keep it updated.
        //Serial.print(", Value:"); Serial.print(value);
        oldValue = currNodeMap->value;
        currNodeMap->value = value;
        if (port <= stripLastPortNum)
        {
          if (oldValue != value)
          {
            //Serial.println("Value changed");
            stripParamChange = true;
            disableEffects = false;
          }
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
  
  if (addNodeMap(dmxwChan, port, logarithmic))
  {
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
  tmpRec.value    = 0;
  for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    nodeMap[i] = tmpRec;
  for (Uint8 i = 0; i < MAX_PORTS; i++)
    portToDmxwMap[i] = 0;
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
  
  /* Set the LED strip control pin and the locate pin low,
   * and stop any ongoing effect.
   */
  digitalWrite(NEO_PIN, LOW);
  digitalWrite(PIN_LOCATE, LOW);
  disableEffects = true;
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

    memset(buffer, 0, sizeof(buffer));
    bufSize = 0;
    command = CMD_RUN;
    buffer[bufSize++] = command;
    for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
    {
      if (i == (dmxwChan - 1))
        buffer[bufSize++] = value;
      else
        buffer[bufSize++] = nodeMap[i].value;
    }
    logPrintln(FLASH("Exec CMD_RUN"));        
    logPrint(FLASH("Buffer: Size["));
    logPrint(bufSize);
    logPrint(FLASH("]  ["));
    for (Uint8 i = 0; i < bufSize; i++)
    {
      logPrint(buffer[i]);
      logPrint(" ");
    }
    logPrintln("] ");
    handleNetRxMessage(command);
  }
  else
    return ACK_EPORT;
/*JVS
  currNodeMap = &nodeMap[dmxwChan - 1]; // Chan map for DMXW chan, dmxwChan
  port = currNodeMap->port;  // Port assigned to dmxwChan
  if (port > 0)
  {
    pin = portMap[port - 1].outPin;
    if ( (pin == PIN_LOCATE) && blinkState )
      blinkState = -1;
    currNodeMap->value = value;
  }
  else
    return ACK_EPORT;
*/

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
/*
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
*/
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
  logPrintln();
  logPrintln(FLASH("Serial port commands are enabled. Commands are:"));
  logPrintln(FLASH("  (spaces may replace commas)"));
  logPrintln(FLASH("<x> in {1,...,512};  <d>,<n> in {1,...,20};"));
  logPrintln(FLASH("<p> in {1,...,16};   <v> in {0,...,255}"));
  logPrintln(FLASH("  d <d>, <v>        - Simulate receipt of new value v for "
                                             "DMXW channel #d."));
  logPrintln(FLASH("  free              - display free RAM"));
  logPrintln(FLASH("  h                 - Print this help text"));
  logPrintln(FLASH("  led <l>, <f>, <c> - Configure addressable LED strip:"));
  logPrintln(FLASH("                        <l> = number of tricolour LEDs"));
  logPrint(  FLASH("                              - ganged triples (12Vdc), "));
  logPrintln(FLASH(                                "singles (5Vdc)"));
  logPrint(  FLASH("                        <f> = 8(800KHz, WS2812 LEDs), "));
  logPrintln(FLASH(                              "4(400KHz, WS2811 drivers)"));
  logPrint(  FLASH("                        <c> = 1(GRB colour wiring), "));
  logPrintln(FLASH(                              "2(RGB colour wiring)"));
  logPrintln(FLASH("  ledCtrl <n>       - Change default output pin for LED "
                                          "ctrl [n in {3-9, 14-21}]"));
  logPrintln(FLASH("  n <d>, <p>, <l>   - Map DMXW chan d to port p, with "
                                         "values to be scaled "));
  logPrintln(FLASH("                       logarithmically if l=1"));
  logPrintln(FLASH("  p                 - Display the Port Mapping."));
  logPrintln(FLASH("  r <d>             - Remove map for DMXW chan d."));
  logPrintln(FLASH("  s                 - Show DMXW channel mapping."));
  logPrintln(FLASH("  save              - Save to EEPROM, DMX-512/DMXW "
                                             "mapping."));
  logPrintln(FLASH("  nodeid <n>        - Set Node Id to n, n in "
                                             "{2, 3, ..., 20} unique"));
  logPrintln();
  logPrintln(FLASH("Effects:\t\t\t\t\tCurrent LED strip parameters:"));
  logPrint(FLASH("  [10] ColourWipe(r, g, b) + [delay]\t\t  Length  <l>\t= "));
  logPrintln(ledStripLen);
  logPrint(FLASH("  [20] Rainbow() + [delay]\t\t\t  Freq    <f>\t= "));
  logPrintln(ledStripFreq);
  logPrint(FLASH("  [30] RainbowCycle() + [delay]\t\t\t  Colours <c>\t= "));
  if (ledStripWiring == 1)
    logPrintln(FLASH("GRB"));
  else
    logPrintln(FLASH("RGB"));
  logPrint(FLASH("  [40] TheatreChase(r, g, b) + [delay]\t\t  Ctrl Pin\t= "));
  logPrintln(ledStripCtrlPin);
  logPrintln(FLASH("  [50] TheatreChaseRainbow() + [delay]"));
  logPrintln(FLASH("  [60] WaterAndEmbers(rLo,gLo,bLo,rHi,gHi,bHi, "
                                         "Depth) + [delay]"));
  logPrintln(FLASH("  [70] Twinkle(r, g, b, min, max, simultaneous, hold)"));
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
  Uint8  stripLen;
  Uint8  stripFreq;
  Uint8  stripCol;
  bool   cmdToProcess = false;
  bool   cmdInvalid = false;
  DmxwNodeMapRecord_t *tmp;
  bool   serialDone = false;


  if (Serial.available() > 0)
  {
    serialBuffer[serialPos] = Serial.read();
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
      while (Serial.available() > 0)
      {
        (void)Serial.read();
      }
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
    logPrintln("] ");
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
        memset(buffer, 0, sizeof(buffer));
        command = CMD_RUN;
        buffer[bufSize++] = command;
        for (Uint8 i = 0; i < MAX_DMXW_CHANS; i++)
        {
          if (i == (dmxwChan - 1))
            buffer[bufSize++] = val;
          else
            buffer[bufSize++] = nodeMap[i].value;
        }
        logPrint(FLASH("Buffer: Size["));
        logPrint(bufSize);
        logPrint(FLASH("]  ["));
        for (Uint8 i = 0; i < bufSize; i++)
        {
          logPrint(buffer[i]);
          logPrint(" ");
        }
        logPrintln("] ");
        handleNetRxMessage(command);
        logPrintln(FLASH("CMD_RUN command simulated"));
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
        
      case 'l':
        // ledCtrl <n>
        // Change default output pin for LED strip control
        if (strstr(serialBuffer, "ledCtrl") != null)
        {
          serialPos += 6;
          val = serialParseInt();
          if ( ((val >=  3) && (val <=  9)) ||
               ((val >= 14) && (val <= 21)) )
          {
            ledStripCtrlPin = (Int8)val;
            for (Uint8 i = 0; i < MAX_PORTS; i++)
            {
              portMap[i].outPin = ledStripCtrlPin;
            }
            logPrint(FLASH("  Output pin "));
            logPrint(ledStripCtrlPin);
            logPrintln(FLASH(" controls the addressable LED strip."));
            logPrintln(FLASH("SAVE CONFIGURATION AND POWER CYCLE THE NODE"));
          }
          else
          {
            logPrint(FLASH("ERROR: Invalid output pin, "));
            logPrintln(val);
          }
          break;
        }
        
        // led <l>, <f>, <c>
        // Configure addressable LED strip
        if (strstr(serialBuffer, "led") != null)
        {
          serialPos += 2;
          stripLen  = serialParseInt();
          stripFreq = serialParseInt();
          stripCol  = serialParseInt();
          if ( (stripFreq != 8) && (stripFreq != 4) )
          {
            logPrint(FLASH("ERROR: Invalid LED strip frequency: "));
            logPrintln(stripFreq);
            break;
          }
          if ( (stripCol != 1) && (stripCol != 2) )
          {
            logPrint(FLASH("ERROR: Invalid LED strip colour wiring: "));
            logPrintln(stripCol);
            break;
          }
          ledStripLen    = stripLen;
          ledStripFreq   = stripFreq;
          ledStripWiring = stripCol;
          logPrintln(FLASH("SAVE CONFIGURATION AND POWER CYCLE THE NODE"));
        }
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
          if (addNodeMap(dmxwChan, port, logarithmic))
          {
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
        logPrintln(FLASH("(ports are mapped to effect selection and "
                         "parameters)"));
        logPrintln(FLASH("Port #\tOut Pin\tConflict  Analog?\tDMX Chan\tName"));
        logPrintln(FLASH("------\t-------\t--------  -------\t--------"
                         "\t--------"));
        for (Uint8 i = 0; i < MAX_PORTS; i++)
        {
          logPrint(i + 1); logPrint(tabChar);
          logPrint(portMap[i].outPin); logPrint(tabChar);
          if (portMap[i].conflictPort != -1)
          {
            logPrint(portMap[i].conflictPort);
          }
          else
          {
            logPrint(FLASH("   -"));
          }
          logPrint(tabChar); logPrint("  ");
          logPrint(portMap[i].isAnalog ? "Y" : "N");
          logPrint(tabChar); logPrint(tabChar);
          if (portToDmxwMap[i] != 0)
            logPrint(portToDmxwMap[i]);
          else
            logPrint("  -");
          logPrint(tabChar); logPrint(tabChar);
          logPrintln(portMap[i].name);
        }
        logPrintln(FLASH("----------------------------------------------"
                         "------------------"));
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
        // save  OR  s
        if (strstr(serialBuffer, "save") != null)
        {
          // save
          // Save to EEPROM, DMX-512/DMXW mapping
          saveMappings = true;
          break;
        }

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
 ************************   Effects Functions   ****************************
 ***************************************************************************/



// Twinkle effect
// Arguments:
//   backgroundColor:I  - Background colour
//   minDelay:I         - Min delay (ms) before next twinkle
//   maxDelay:I         - Max delay (ms) before next twinkle
//   maxSimultaneous:I  - Max # of LEDs (triples) allowed to twinkle
//                          simultaneously
//   holdTime:I         - Duration (ms) for which twinkle is held
void twinkle(uint32_t backgroundColor,
             uint16_t minDelay,
             uint16_t maxDelay,
             uint8_t maxSimultaneous,
             uint8_t holdTime)
{
  static uint16_t i = 0;
  static uint16_t j = 0;
  static boolean newIteration = false;
  static Uint8 phase = 0;
  static long delayEnd = 0;
  static uint16_t numSimultaneous;
  long currTime = 0;
  uint16_t k;
  
  if (newEffect)
  {
    for (i = 0; i < strip->numPixels(); i++)
    {
      strip->setPixelColor(i, backgroundColor);
    }
    strip->show();
    newIteration = true;
  }
  
  if (newIteration)
  {
    newIteration = false;
    i = 0;
    phase = 1;
    numSimultaneous = random(1, maxSimultaneous);
  }
  else if (i < 10)
  {
    switch (phase)
    {
      case 1:
        // Set white pixels randomly
        for (j = 0; j < numSimultaneous; j++)
        {
          k = (uint16_t)random(0, strip->numPixels());
          strip->setPixelColor(k, strip->Color(255,255,255));
        }
        strip->show();
        phase = 2;
        delayEnd = millis() + holdTime;
        break;
      
      case 2:
        // Hold the twinkle; account for timer wraparound
        currTime = millis();
        if (delayEnd >= holdTime)
        {
          if (currTime < delayEnd)
            break;
        }
        else
        {
          if (currTime > delayEnd)
            break;
        }
        phase = 3;
        break;
      
      case 3:
        // Remove white pixels (revert to all background colour)
        for (j = 0; j < strip->numPixels(); j++)
        {
          strip->setPixelColor(j, backgroundColor);
        }
        strip->show();
        delayEnd = millis() + random(minDelay, maxDelay);
        phase = 4;
        break;
      
      case 4:
        // Delay the next twinkle; account for timer wraparound
        currTime = millis();
        if (delayEnd >= holdTime)
        {
          if (currTime < delayEnd)
            break;
        }
        else
        {
          if (currTime > delayEnd)
            break;
        }
        i++;
        if (i >= 10)
          newIteration = true;
        phase = 1;
        break;
      
      default:
        logPrint(FLASH("ERROR: Twinkle entered default case"));
        break;
    }
  }
}


/*----------------------------------------------------------------------------
  Function: waterAndEmbers
  
  Undulating colours effect that can simulate wavy waters and glowing embers.
  
  Parameters:
    redLow:I     - Lower limit of red (0 - 255)
    redHigh:I    - Upper limit of red (0 - 255)
    greenLow:I   - Lower limit of blue (0 - 255)
    greenHigh:I  - Upper limit of green (0 - 255)
    blueLow:I    - Lower limit of green (0 - 255)
    blueHigh:I   - Upper limit of blue (0 - 255)
    depth:I      - Depth of colour changes (0 - 255)
    
  Notes:
    Here are a couple of starting points:
       Water effect
          red range   = [0 - 0]
          green range = [140 - 170]
          blue range  = [50 - 60]
          depth       = 25
          [delay]     = 3
       Glowing embers effect
          red range   = [100 - 255]
          green range = [20 - 30]
          blue range  = [0 - 3]
          depth       = 5
          [delay]     = 3
  ----------------------------------------------------------------------------*/
void waterAndEmbers( uint8_t redLow,  uint8_t greenLow,  uint8_t blueLow,
                     uint8_t redHigh, uint8_t greenHigh, uint8_t blueHigh,
                     uint8_t depth )
{
  static int16_t r = 0;
  static int16_t g = 0;
  static int16_t b = 0;
  static int16_t led = 0;
  static int16_t level = 0;
  static int16_t changeCount = 0;
  static int8_t levelDirection = 0;
  int16_t wait;
  int16_t r_old, g_old, b_old, level_old;
  uint8_t i;
  bool change = true;
  
  if (change)
  {
    r_old = r;
    g_old = g;
    b_old = b;
    
    r = random(redLow, redHigh);
    if (r > 0)
      r += level;
    if (r < 0)
      r = 0;
    if (r > 255)
      r = 255;
    g = random(greenLow, greenHigh);
    if (g > 0)
      g += level;
    if (g < 0)
      g = 0;
    if (g > 255)
      g = 255;
    b = random(blueLow, blueHigh);
    if (b > 0)
      b += level;
    if (b < 0)
      b = 0;
    if (b > 255)
      b = 255;
      
    while ( (abs(r_old - r) < 3) &&
            (abs(g_old - g) < 3) &&
            (abs(b_old - b) < 3) )
    {
      i = random(0, 3);
      switch (i)
      {
        case 0: r++; break;
        case 1: g++; break;
        case 2: b++; break;
      }
    };
              
    level = random(-depth, depth);
    if (abs(level_old - level) < 20)
      if (random(0,2) == 1)
        levelDirection = 1;
      else
        levelDirection = -1;
    while (abs(level_old - level) < 8)
      level += levelDirection * 3;
  }
  wait = random(1, 1);
  change = (random(5) < 1);
  if (changeCount < 3)
  {
    change = false;
    changeCount++;
    if (changeCount > 4)
      change = true;
  }
  if (change)
    changeCount = 0;
    
  strip->setPixelColor(led, strip->Color(r,b,g));
  led = (led + 1) % strip->numPixels();
  strip->show();
  nextFxTime = millis() + wait;
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c)
{
  static uint16_t i = 0;

  if (newEffect)
  {
    i = 0;
  }
  
  if (++i >= strip->numPixels())
    i = 0;

  strip->setPixelColor(i, c);
  strip->show();
}


void rainbow(void)
{
  static uint16_t j = 0;
  uint16_t i = 0;
    
  if (++j >= 256)
    j = 0;  
  
  for(i = 0; i < strip->numPixels(); i++)
  {
    strip->setPixelColor(i, Wheel( (i+j) & 255 ));
  }
  strip->show();
}


// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(void)
{
  static uint16_t j = 0;
  uint16_t i;
  
  if (++j >- (256 * 3))
    j = 0;
    
  // 3 cycles of all colors on wheel
  for(i = 0; i < strip->numPixels(); i++)
  {
    strip->setPixelColor(i, Wheel(((i * 256 / strip->numPixels()) + j) & 255));
  }
  strip->show();
}


//Theatre-style crawling lights.
void theaterChase(uint32_t c)
{
  static uint16_t q = 0;
  
  for (int i = 0; i < strip->numPixels(); i = i+3)
  {
    //turn every third pixel off
    strip->setPixelColor(i + q, 0);
  }

  if (++q >= 3)
    q = 0;
    
  for (int i = 0; i < strip->numPixels(); i = i+3)
  {
    //turn every third pixel on
    strip->setPixelColor(i + q, c);
  }
  strip->show();     
}


//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(void)
{
  static uint16_t q = 0;
  static uint16_t j = 0;
  static boolean ledsOn = false;
  
  ledsOn = !ledsOn;
  
  if (ledsOn)
  {
     for (int i = 0; i < strip->numPixels(); i = i+3)
     {
       //turn every third pixel off
       strip->setPixelColor(i + q, Wheel( (i+j) % 255));
     }
     strip->show();
  }
  else
  {
     for (int i = 0; i < strip->numPixels(); i = i+3)
     {
       //turn every third pixel on
       strip->setPixelColor(i + q, 0);
     }
     
     if (++q >= 3)
     {
       q = 0;
       if (++j >= 256)
         j = 0;
     }
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  if(WheelPos < 85)
  {
   return strip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if(WheelPos < 170)
  {
   WheelPos -= 85;
   return strip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else
  {
   WheelPos -= 170;
   return strip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


/***************************************************************************
 ***************************************************************************/



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
  
  for (Uint8 i = 0; i < MAX_PORTS; i++)
  {
    portToDmxwMap[i] = 0;
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
  Serial.print(FLASH("   (Slave)      Radio frequency: "));
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(FLASH("Mhz"));
  Serial.print(COPYRIGHT);
  Serial.print(FLASH("\t#Resets: "));
  Serial.print(resetCount);
  Serial.println(FLASH("\t\t"));
//  CheckRam();

  Serial.print(FLASH("Addressable LED Strip S/W: "));
  Serial.print(SW_VERSION_c);
  Serial.print(FLASH("\tF/W: "));
  Serial.print(FW_VERSION_c);
  Serial.println();

  #ifndef LOGGING_ON
    Serial.println();
    Serial.println(FLASH("CONSOLE OUTPUT IS DISABLED. To enable output, "
                         "recompile with LOGGING_ON"));
    Serial.println(FLASH("and SERIAL_CMDS_ENABLED defined, and reflash the "
                         "node."));
  #endif

  #ifdef SERIAL_CMDS_ENABLED
    logPrintln();
    logPrintln(FLASH("Enter 'h' for help"));
  #endif
  
 
  // Parameter 1 = number of pixels in strip
  // Parameter 2 = Arduino pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  // JVS: On the 12V strip that I have, change the colour order in functions calls
  //      from R/G/B to R/B/G
  ledStripFlags = 0;
  if (ledStripWiring == 1)
    ledStripFlags = NEO_GRB;
  else
    ledStripFlags = NEO_RGB;
  if (ledStripFreq == 8)
    ledStripFlags += NEO_KHZ800;
  else
    ledStripFlags += NEO_KHZ400;
  if (ledStripLen >= 1)
  {
    // Apparently valid configuration parameters
    strip = new Adafruit_NeoPixel( ledStripLen, (uint8_t)ledStripCtrlPin,
                                   (ledStripWiring + ledStripFreq) );
  }
  else
  {
    // Configuration parameters appear questionable. Create a default strip.
    strip = new Adafruit_NeoPixel( 1, NEO_PIN, NEO_GRB + NEO_KHZ800 );
  }
    
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
/*
  if (handleInput && (rxCount % 1000) == 0)
  {
    logPrint(FLASH("rxCount:"));
    logPrint(rxCount);
    logPrint(FLASH("\t"));
    CheckRam();
  }
*/

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

  /* Override parameter changes if Delay is 255 */
  if (nodeMap[portToDmxwMap[0]-1].value == 255)
  {
    stripParamChange = false;
  }
  
  if (stripParamChange)
  {
    Uint8 tmpDmxwChan;
    
    tmpDmxwChan = portToDmxwMap[0];  // Port 1 --> DMXW chan #
    if (tmpDmxwChan == 0)
      stripDelay = 0;
    else
      stripDelay = nodeMap[tmpDmxwChan-1].value;
      
    tmpDmxwChan = portToDmxwMap[1];
    if (tmpDmxwChan == 0)
      stripEffect = 0;
    else
      stripEffect = nodeMap[tmpDmxwChan-1].value / 10;
      
    tmpDmxwChan = portToDmxwMap[2];
    if (tmpDmxwChan == 0)
      stripArg1 = 0;
    else
      stripArg1 = nodeMap[tmpDmxwChan-1].value;
      
    tmpDmxwChan = portToDmxwMap[3];
    if (tmpDmxwChan == 0)
      stripArg2 = 0;
    else
      stripArg2 = nodeMap[tmpDmxwChan-1].value;
      
    tmpDmxwChan = portToDmxwMap[4];
    if (tmpDmxwChan == 0)
      stripArg3 = 0;
    else
      stripArg3 = nodeMap[tmpDmxwChan-1].value;

    tmpDmxwChan = portToDmxwMap[5];
    if (tmpDmxwChan == 0)
      stripArg4 = 0;
    else
      stripArg4 = nodeMap[tmpDmxwChan-1].value;

    tmpDmxwChan = portToDmxwMap[6];
    if (tmpDmxwChan == 0)
      stripArg5 = 0;
    else
      stripArg5 = nodeMap[tmpDmxwChan-1].value;

    tmpDmxwChan = portToDmxwMap[7];
    if (tmpDmxwChan == 0)
      stripArg6 = 0;
    else
      stripArg6 = nodeMap[tmpDmxwChan-1].value;

    tmpDmxwChan = portToDmxwMap[8];
    if (tmpDmxwChan == 0)
      stripArg7 = 0;
    else
      stripArg7 = nodeMap[tmpDmxwChan-1].value;

 
    switch (stripEffect)
    {
      case 1: logPrint("Colour Wipe");            break;
      case 2: logPrint("Rainbow");                break;
      case 3: logPrint("Rainbow Cycle");          break;
      case 4: logPrint("Theatre Chase");          break;
      case 5: logPrint("Theatre Chase Rainbow");  break;
      case 6: logPrint("Water and Embers");       break;
      case 7: logPrint("Twinkle");                break;
      case 8: logPrint("Ember Effect");           break;
      default:
        logPrint("<unknown effect (");
        logPrint(stripEffect);
        logPrint (")>");
    }
    logPrint(": Delay="); logPrint(stripDelay);
    logPrint("  (");
    logPrint(stripArg1);
    logPrint(", ");
    logPrint(stripArg2);
    logPrint(", ");
    logPrint(stripArg3);
    logPrint(", ");
    logPrint(stripArg4);
    logPrint(", ");
    logPrint(stripArg5);
    logPrint(", ");
    logPrint(stripArg6);
    logPrint(", ");
    logPrint(stripArg7);
    logPrintln(")");
    
    if (stripEffect != oldStripEffect)
    {
      oldStripEffect = stripEffect;
      newEffect = true;
    }
    else
    {
      if ( ((stripEffect == 1) || (stripEffect == 7))
           && stripParamChange)
      {
        newEffect = true;
      }
    }
    stripParamChange = false;
  }

  if ( disableEffects || (stripDelay == 0) )
  {
    // Turn off all LEDs
    for(Uint16 i = 0; i < strip->numPixels(); i++)
    {
      strip->setPixelColor(i, 0);
    }
    strip->show();
  }
  else
  {
    /* Continue running the current effect */
    currentTime = millis();
    if (currentTime >= nextFxTime)
    {
      switch (stripEffect)
      {
        case 1:
          colorWipe(strip->Color(stripArg1, stripArg3, stripArg2));
          break;
  
        case 2:
          rainbow();
          break;
      
        case 3:
          rainbowCycle();
          break;
      
        case 4:
          theaterChase(strip->Color(stripArg1, stripArg3, stripArg2));
          break;
      
        case 5:
          theaterChaseRainbow();
          break;
      
        case 6:
          waterAndEmbers( stripArg1, stripArg2,
                          stripArg3, stripArg4,
                          stripArg5, stripArg6,
                          stripArg7 );
          break;
          
        case 7:
          if (newEffect)
          {
            if (stripArg4 == 0)
              stripArg4 = 1;
            if (stripArg5 == 0)
              stripArg5 = 100;
            else if (stripArg5 == 1)
              stripArg5 = 2;
            if (stripArg6 == 0)
              stripArg6 = 5;
            if (stripArg6 > ledStripLen)
              stripArg6 = ledStripLen;
            if (stripArg7 == 0)
              stripArg7 = 20;
            if (stripArg4 >= stripArg5)
              stripArg4 = stripArg5 - 1;
            stripDelay = 1;
          }
          twinkle(strip->Color(stripArg1, stripArg3, stripArg2),
                  stripArg4*10, stripArg5*10, stripArg6, stripArg7);
          break;
      
        default:
          ; // Ignore unknown Effect value
      }
      newEffect = false;
      
      switch (stripEffect)
      {
        case 5:
          nextFxTime = currentTime + stripDelay/4;
          break;
          
        default:
          nextFxTime = currentTime + stripDelay;
      }
    }
  }
  
}
