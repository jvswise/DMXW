/* DmxwDefs.h */
#ifndef DmxwDefs_h
#define DmxwDefs_h

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
                           // Wireless network Run command. The tester
                           // broadcasts a packet of time division multiplexed
                           // DMXW channel values.
                                                      
// ----- Configuration & Test Commands used by the DMXW Tester
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
#define CMD_OFF       11   // CMD_OFF([n:8] | [ALL]) - Gateway commands node n
                           //   (or all nodes) to set all output port values
                           //   to 0 (all ports 'off').

// ACK codes       ACK(<ACK code>)
#define ACK_OK        0    // No error.
#define ACK_ECMD      1    // Command unsupported.
#define ACK_ETIME     2    // Timed out waiting for ACK.
#define ACK_EDMXW     3    // DMXW Channel access error.
#define ACK_EPORT     4    // Port access error.
#define ACK_ERR       254  // Unspecified error.
#define ACK_NULL      255  // 'undefined' ACK value


typedef unsigned char AckCode_t;


#endif /* DmxwDefs_h */
