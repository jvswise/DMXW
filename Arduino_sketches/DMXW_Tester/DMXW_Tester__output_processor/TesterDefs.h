/* TesterDefs.h */
#ifndef TesterDefs_h
#define TesterDefs_h


/********************************  Constants  *********************************/
#define NUM_CHANS_ONBOARD        5        // # onboard test channels available
#define NUM_CHAN_DMXW           48        // # valid DMXW channels available
#define PIXEL_DELAY_CHAN         5        // Onboard channel # for rainbow
                                          //   effect speed control during
                                          //   onboard pixel test.


#define CHAN_UNDEFINED          -1        // The 'undefined' channel #

#define MIN_CHANNEL_VALUE        0
#define MAX_CHANNEL_VALUE      255

#define MAX_SERIAL_BUF_LEN      64


/***  Interprocessor Communications constants  ***/
#define CHAR_ESC     0xFD   // Escape character (not ASCII code)
#define CHAR_STX     0xFE   // Start-of-text character (not ASCII code)
#define CHAR_ETX     0xFF   // End-of-text character (not ASCII code)

// TSTCMD_SELECT command values for selectCmd parameter 
#define SELECT_ADD   1
#define SELECT_DEL   2
#define SELECT_CLR   3

// TSTCMD_ACK returnCode values
#define TSTACK_OK         0
#define TSTACK_OUTREBOOT  1 // Output processor has been reset.
#define TSTACK_BAD_PARM   2 // Bad parameter value(s) or # of parameters
#define TSTACK_BAD_STATE  3 // Current testing state is invalid for this
                            //   command.
#define TSTACK_CORRUPTED  4 // Corrupted command detected.

// TSTCMD_ACK returnCode values
#define TSTACK_OK         0
#define TSTACK_BAD_PARM   1 // Bad parameter value(s) or # of parameters
#define TSTACK_BAD_STATE  2 // Current testing state is invalid for this
                            //   command.
#define TSTACK_CORRUPTED  3 // Corrupted command detected.

// IPC processing states
#define IPC_SEEK     0  // Initial state: Looking for first CHAR_STX
#define IPC_SEEK2    1  // Looking for 2nd CHAR_STX
#define IPC_SEQNUM   2  // Looking for the message's sequence number
#define IPC_COLLECT  3  // Reading in command characters
#define IPC_ESC      4  // CHAR_ESC found while reading cmd chars
#define IPC_CRC      5  // Collect the CRC8 value
#define IPC_PURGE    6  // Error: Purge buffer until empty or CHAR_STX found.

// Interprocessor Commands
// =======================================================
// - Communication between the Output Processor and HMI Processor
//   takes places over serial ports using the SoftwareSerial Arduino
//   library. (This allows for interprocessor communications while
//   still being able to debug software using a UART serial console
//   connection to either processor.
// - Each command is prepended by a non-escaped pair of CHAR_STX characters,
//   followed by an 8-bit sequence number and terminated by a non-escaped
//   CHAR_ETX character followed by a CRC8 value.
// - Any parameter value of 253, 254, or 255, which would conflict with special
//   character values CHAR_ESC, CHAR_STX, and CHAR_ETX (respectively), needs to
//   be escaped. Thus, the following encoding/decoding needs to be used:
//     Parameter Value        Encoded Sequence
//     ---------------        ----------------
//          253                CHAR_ESC, 253
//          254                CHAR_ESC, 254
//          255                CHAR_ESC, 255
// - All commands are acknowledged (or time out at the HMI Processr) to
//   indicate success or failure.
#define TSTCMD_UNDEF    0  // 'uninitialized' command code

#define TSTCMD_INIT     1  // Startup handshake message from HMI processor.
                           // Parameters: (none)
                           // Return:
                           //   The command always succeeds.
                           // Note: After the output processor reboots, it
                           //       will continue to reply to all IPC messages
                           //       with TSTACK_OUTREBOOT until it receives
                           //       this command.

#define TSTCMD_STATE    2  // Set the current testing state.
                           // Parameters:
                           //  1:state [uint8]
                           //     The new testing state (see TestState_t
                           //     values which need to be cast as uint8).
                           // Return:
                           //   The command always succeeds.
                           // Notes:
                           //  - When state is set to STOPPED, the test type and
                           //    test output are both reset to DISABLED and
                           //    NO_OUTPUT, respectively.

#define TSTCMD_TEST     3  // Select the test type and output for the test to
                           // be run.
                           // Parameters:
                           //  1:output [uint8]
                           //     The output for the test (see TestOutput_t).
                           //  2:type   [uint8]
                           //     The test type to be run (see TestType_t).
                           // Return:
                           //  A failure acknowledgement is sent if the current
                           //  testing state is other than STOPPED.
                           // Notes:
                           //  - See also TSTCMD_STATE.

#define TSTCMD_SELECT   4  // Select channels for testing.
                           // Parameters:
                           //  1:selectCmd [uint8] (see e.g. SELECT_ADD)
                           //     1 - Add the specified channels to the
                           //         set of selected channels.
                           //     2 - Remove the specified channels from the
                           //         set of selected channels.
                           //     3 - Clear the set of selected channels.
                           //  2:numChans [uint8]
                           //     The number of channels that follow. This
                           //     should be set to zero if selecSet is 3.
                           //  3:channelList  [list of uint8]
                           //     The list of channels to be added or removed.
                           //     The list length must equal numChans.
                           // Return:
                           //  A failure acknowledgement is sent if the current
                           //  testing state is other than STOPPED.

#define TSTCMD_VALUE    5  // Set channel values.
                           // Parameters:
                           //  1:numChans [uint8]
                           //     The number of (channel #, value) pairs that
                           //     follow. The value must be greater than zero.
                           //  2:chanValList [list of (uint8, uint8) pairs]
                           //     The first element in each pair is the channel
                           //     number to which the second element (the value)
                           //     is to be applied.
                           //     The list length (in # of pairs) must equal
                           //     numChans. Any channels that are in the current
                           //     test's selection set, but which are missing
                           //     from the list, are unaffected.
                           // Return:
                           //  A failure acknowledgement is sent if any of
                           //  the channels aren't in the current test's
                           //  selection set (refer to TSTCMD_SELECT), or if
                           //  numChans is zero.
                           // Note:
                           //  The command is valid in any test state, but
                           //  applies to the currently selected test outputs.

#define TSTCMD_PIXCFG   6  // Configure an LED pixel string that is to be
                           // connected to the onboard digital output channel.
                           // Parameters:
                           //  1:length [uint16]
                           //     Number of individually controllable pixels.
                           //     (On 5Vdc strings, each tricolour LED tends to
                           //     be controllable. On 12Vdc strings, LEDs tend
                           //     to be grouped in triples; thus, a string with
                           //     30 LEDs would have a length of 10.) The value
                           //     is transmitted in network byte order (most
                           //     significant byte first).
                           // Return:
                           //  This command always succeeds.

#define TSTCMD_ACK      7  // Acknowledge the received command.
                           // Parameters:
                           //  1:cmdCode [uint8]
                           //     The command code (e.g. TSTCMD_STATE) to
                           //     which this is an acknowledgement.
                           //  1:returnCode [uint8]
                           //     A value of zero indicates success. Any other
                           //     value indicates failure.
                           // Return:  (none)

/****************************  Type Definitions  ******************************/
typedef enum
{
  NO_OUTPUT = 0,
  ONBOARD,
  DMXW,
} TestOutput_t;

typedef enum
{
  DISABLED = 0,
  MANUAL,
  CHAN_SWEEP,
  PIXEL
} TestType_t;

typedef enum
{
  STOPPED = 0,
  PAUSED,
  RUNNING
} TestState_t;

#ifndef int8
  typedef signed char     int8;
#endif
#ifndef uint8
  typedef unsigned char  uint8;
#endif
#ifndef int16
  typedef int            int16;
#endif
#ifndef uint16
  typedef unsigned int  uint16;
#endif
#ifndef int32
  typedef signed long    int32;
#endif
#ifndef uint32
  typedef unsigned long uint32;
#endif

#endif /* TesterDefs_h */
