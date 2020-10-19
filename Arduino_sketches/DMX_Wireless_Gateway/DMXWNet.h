/* DMXWNet.h */
#ifndef DMXWNet_h
#define DMXWNet_h

/*************************************************************************
 * Definitions for DMX Wireless Network (DMXW)
 *************************************************************************/

#define GATEWAYID      1
#define BROADCASTID    255 // Designated node id for broadcasts
#define NETWORKID      77  // The same for all nodes on the network

// Match frequency to the Moteino's radio hardware
#define FREQUENCY      RF69_433MHZ

//#define IS_RFM69HW     // uncomment only for RFM69HW transceivers.
#define ENCRYPTKEY     "JVS_DMX_Key23456" // exactly 16 chars: same on all nodes
#define ACK_WAIT_TIME  50  // max # of ms to wait for an ack
#define TX_NUM_RETRIES 2   // number of TX transmission attempts when ACK needed

#define MAX_DMX512_CHANS   512
#define MAX_DMXW_CHANS     48
#define MAX_NODES          20
#define MAX_PORTS          16
#define NODEID_UNDEF       0
#define NODEID_MAX         (MAX_DMXW_CHANS + 1)

// Command codes   <Command code>(<arg>...)
// =======================================================
// - every packet sent by any node (including gateway) starts with
//   duplicated source and destination node numbers--used to check
//   for message corruption.
//     - these are implicit and aren't shown in the command syntax summaries
//       below
// - following those is the command code
// - command packet argument, [X], is an implied destination to which a packet
//   is sent, either a specific node X (a unicast message),
//   or if X=ALL (a broadcast message)
// - notation "x:N" means argument x of size N bits
#define CMD_UNDEF     0    // 'uninitialized' command code

#define CMD_RUN       1    // CMD_RUN([ALL], v1:8, ..., vn:8), n = MAX_DMS_CHANS
                           // Wireless network is in Run mode. Gateway
                           // broadcasts a packet of time division multiplexed
                           // DMXW channel values.
                                                      
// ----- Configuration & Test Commands
#define CMD_PING      3    // CMD_PING([n:8]) - gateway requests a CMD_PONG
                           //   liveness/existence response from node n.
#define CMD_PONG      4    // CMD_PONG([g:8]) - Node liveness/existence
                           //   confirmation to DMX gateway, g. Response to
                           //   CMD_PING.
#define CMD_MAP       5    // CMD_MAP([n:8], d:8, p:8, l:8) - Gateway commands node n
                           //   to map DMXW channel #d to port #p, and if
                           //   the output is analog, adjust it to a
                           //   logarithmic scale.
                           //   (Gateway maintains a mapping from DMX-512
                           //    channels to DMXW local wireless network
                           //    channel.)
#define CMD_MAPR      6    // CMD_MAPR([n:8], d:8) - Gateway commands node n
                           //   to remove the mapping for DMXW channel d.
#define CMD_CLRALL    7    // CMD_CLRALL([n:8] | [ALL]) - Gateway commands node
                           //   n (or all nodes) to clear all of their
                           //   DMXW channel mappings.
#define CMD_ECHO      8    // CMD_ECHO([n:8], d:8) - Gateway requests node n to
                           //   report back the port mapping information for
                           //   the port assigned to DMXW channel d. (CMD_CHAN
                           //   is expected as a response.)
#define CMD_CHAN      9    // CMD_CHANS([g:8], d:8, p:8, o:8, c:8, a:8, v:8)
                           //   - node reports to gateway g that, assigned to
                           //     DMX channel d, is port p which is mapped
                           //     to output pin o which is either analog
                           //     (a = 1) or digital (a = 0), and potential
                           //     conflict port c, and current value v.
#define CMD_LOC       10   // CMD_LOC([n:8]) - Gateway requests node n to blink
                           //   location beacon to help locate the node.
                           //   (by default, node blinks the onboard LED)
#define CMD_OFF       11   // CMD_OFF([n:8] | [ALL]) - Gateway commands node n
                           //   (or all nodes) to set all output port values
                           //   to 0 (all ports 'off').
#define CMD_PORT      12   // CMD_PORT([n:8], p:8, v:8) - Gateway commands node n to
                           //   set port p value to v.
#define CMD_CTRL      13   // CMD_CTRL([n:8], d:8, v:8) - Gateway commands node n to
                           //   set port assigned to DMXW channel d to value v.
#define CMD_TEST      254  // CMD_TEST([n:8]) - Gateway sends test command to
                           //   node n. Response is node-defined.
                           //   No behavioural semantics are implied.
#define CMD_SAVE      255  // CMD_SAVE([n:8]) - Gateway commands node n to commit
                           //   current set of DMXW channel # to port # mappings
                           //   to non-volatile storage (e.g. EEPROM)

// ACK codes       ACK(<ACK code>)
#define ACK_OK        0    // No error.
#define ACK_ECMD      1    // Command unsupported.
#define ACK_ETIME     2    // Timed out waiting for ACK.
#define ACK_EDMXW     3    // DMXW Channel access error.
#define ACK_EPORT     4    // Port access error.
#define ACK_ERR       254  // Unspecified error.
#define ACK_NULL      255  // 'undefined' ACK value


#ifndef Int8
  typedef signed char   Int8;
#endif
#ifndef Uint8
  typedef unsigned char Uint8;
#endif
#ifndef Uint16
  typedef unsigned int  Uint16;
#endif

typedef unsigned char AckCode_t;

// Mapping records used by the DMXW gateway
typedef struct GatewayMapping
{
  Uint8    dmxwChan;    // Local DMXW channel. (0 = invalid channel)
  Uint16   dmx512Chan;  // DMX-512 channel that's mapped to dmxwChan.
  Uint8    nodeId;      // Node assigned to dmxwChan.
  Uint8    port;        // Port of node, nodeId, assigned to dmxwChan.
  bool     logarithmic; // Port values are to be scaled logarithmically.
  // ??JVS  We probably don't need to store the value here.
  Uint8    value;       // Last known value for dmxwChan
                        //   (ultimately, a DMX-512 channel value).
} DmxwGwMapRecord_t;


// Mapping records used by DMXW nodes to map DMXW channels to Ports.
typedef struct NodeMapping
{
  Int8   port;          // Port assigned to DMXW channel (-1 = no assignment)
  bool   isOutput;      // Is the port an input (false) or output (true) port?
  bool   isLogarithmic; // Should DMX-512 values on an analog output be
                        //   adjusted to a perceived linear brightness scale?
                        //   Normally, an LED at PWM value 200 looks much less
                        //   than twice as bright as one at 100.
  Uint8  value;
} DmxwNodeMapRecord_t;


// Mapping records used by DMXW nodes to map DMXW channel Ports to I/O pins
typedef struct PortMapping
{
  Int8   inPin;        // Pin used for input. (Not yet used)
  Int8   outPin;       // Pin used for output.
  Int8   conflictPort; // Some digital output pins can act as either digital
                       //   or analog, defined on separate DMXW ports.
                       //   conflictPort identifies the other port that could
                       //   conflict.
  bool   isAnalog;     // Is this an analog port? (Digital otherwise.)
} NodePortMapRecord_t; 

#endif
