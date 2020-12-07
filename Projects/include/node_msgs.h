/*
 * node_msgs.h
 *
 *  Created on: 27 Feb 2018
 *      Author: TE192184
 */

#ifndef __NODE_MSGS_H__
#define __NODE_MSGS_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>



// v3 ping msg format (for now)
// | ping header | route addr | hop count/contention size | src node addr | dest node addr  | fault node (if fault) | specific report node | spare header | 50 byte payload array |
// |0.5 | 0.5 | 1 | 2 | 2 | 2 | 2 | 2 | 50 |
// Total of 62 bytes

// dest node addr will be zero for a ping with no next hop

// With each of the 5, 10 byte payloads looking like
// | src addr | sink addr | version | 5 byte payload data |
// | 2 | 2 |  1 | 5 |
// Total 10 bytes * 5 reports = 50 bytes

// or if in full firmware update mode the 50 byte ping payload is arranged as
// | payload num | write addr |  44 byte binary firmware data |
// | 2 | 4 |  44 |
// Total 2 bytes + 4 bytes + 44 = 50 bytes

// if patching rather than full can have between 1 and n set of addr+len+data repeats
// | payload num | write addr1 | data len1 | data1 | write addr2 | data len2 | data2 | etc padded with zero towards end if necessary
// | 2 | 4 | 1 |  n | 4 | 1 |  n |
// Total  50 bytes

// firmware update payloads are preceded and followed by firmware config payloads
// | start or end config | full or patch | version from | version to | number of payloads in update | firmware size | firmware crc |
// | 1 | 1 | 1 | 1 | 2 | 4 | 4 |
// Total of 14 bytes


// v3 ack msg format (for now)
// | ack header | route addr  | src node addr | dst node addr | fault node (if fault) | spare header | 7 byte spare payload |
// |0.5 | 0.5 | 2 | 2 | 2 | 4 | 7 |
// Total 18 bytes

// ping/ack header bytes, purpose is to differentiate between nuances of msg type, and also to filter 'interference' messages that are not of our origin
// we will also be checking the packet length
#define PING_MSG_HEADER_ROUTE_FORMING           0x0
#define PING_MSG_HEADER_ROUTE_FORMED            0x1
#define PING_MSG_HEADER_ROUTE_FORMING_AUTO_PING 0x2
#define PING_MSG_HEADER_ROUTE_FORMED_AUTO_PING  0x3
#define PING_MSG_HEADER_NON_ROUTE_NODE_DATA     0x4
#define PING_MSG_HEADER_RESET_ROUTE             0x5 // force route nodes back to searching (once they have forwarded on the ping) contending route nodes should hearing this ping should also drop back
#define PING_MSG_HEADER_FIRMWARE_UPDATE_CONFIG  0x6 // at start and end of firmware send a config ping (route formed assumed)
#define PING_MSG_HEADER_FIRMWARE_UPDATE_SENDING 0x7 // payload is a firmware update block (route formed assumed)
#define ACK_MSG_HEADER_REPLY_ACK                0x8
#define ACK_MSG_HEADER_REPLY_NAK                0x9
#define ACK_MSG_HEADER_REPLY_ACK_ROUTE_FAULT    0xA // payload is the node id of a faulty node that is not currently ack'ing the ping
#define ACK_MSG_HEADER_REPLY_BS_ACK             0xB // ack back from end base station (WORKAROUND so a node can leave more of a window for acks if it will be pinging the base station, as these do not reply with the same timing consistency)
#define ACK_MSG_HEADER_REPLY_REPORT_ACK         0xC
#define ACK_MSG_HEADER_REPLY_RESET_ROUTE        0xD // force route nodes back to searching via ack initiated at end basestation (when autopinging is active)

// ping msg lengths and offsets
#define PING_OVERALL_SIZE_BYTES                 62
#define PING_OVERALL_SIZE_BITS                  (PING_OVERALL_SIZE_BYTES * 8)

#define PING_MSG_HEADER_LENGTH_BITS              4
#define PING_MSG_ROUTE_ADDR_LENGTH_BITS          4

#define PING_MSG_HOP_COUNT_ADDR_OFFSET_BITS      (PING_MSG_HEADER_LENGTH_BITS + PING_MSG_ROUTE_ADDR_LENGTH_BITS)
#define PING_MSG_HOP_COUNT_ADDR_OFFSET_BYTES     (PING_MSG_HOP_COUNT_ADDR_OFFSET_BITS / 8)
#define PING_MSG_HOP_COUNT_ADDR_LENGTH_BITS      8

#define PING_MSG_SRC_NODE_ADDR_OFFSET_BITS      (PING_MSG_HOP_COUNT_ADDR_OFFSET_BITS + PING_MSG_HOP_COUNT_ADDR_LENGTH_BITS)
#define PING_MSG_SRC_NODE_ADDR_OFFSET_BYTES     (PING_MSG_SRC_NODE_ADDR_OFFSET_BITS / 8)
#define PING_MSG_SRC_NODE_ADDR_LENGTH_BITS      16

#define PING_MSG_DEST_NODE_ADDR_OFFSET_BITS     (PING_MSG_SRC_NODE_ADDR_OFFSET_BITS + PING_MSG_SRC_NODE_ADDR_LENGTH_BITS)
#define PING_MSG_DEST_NODE_ADDR_OFFSET_BYTES    (PING_MSG_DEST_NODE_ADDR_OFFSET_BITS / 8)
#define PING_MSG_DEST_NODE_ADDR_LENGTH_BITS     16

// fault address
#define PING_MSG_FAULT_NODE_ADDR_OFFSET_BITS     (PING_MSG_DEST_NODE_ADDR_OFFSET_BITS + PING_MSG_DEST_NODE_ADDR_LENGTH_BITS)
#define PING_MSG_FAULT_NODE_ADDR_OFFSET_BYTES    (PING_MSG_FAULT_NODE_ADDR_OFFSET_BITS / 8)
#define PING_MSG_FAULT_NODE_ADDR_LENGTH_BITS     16

// report address (used by non-route nodes to see if they have been chosen specifically to report this frame)
#define PING_MSG_REPORT_NODE_ADDR_OFFSET_BITS     (PING_MSG_FAULT_NODE_ADDR_OFFSET_BITS + PING_MSG_FAULT_NODE_ADDR_LENGTH_BITS)
#define PING_MSG_REPORT_NODE_ADDR_OFFSET_BYTES    (PING_MSG_REPORT_NODE_ADDR_OFFSET_BITS / 8)
#define PING_MSG_REPORT_NODE_ADDR_LENGTH_BITS     16

// spare header
#define PING_MSG_SPARE_HEADER_OFFSET_BITS       (PING_MSG_REPORT_NODE_ADDR_OFFSET_BITS + PING_MSG_REPORT_NODE_ADDR_LENGTH_BITS)
#define PING_MSG_SPARE_HEADER_OFFSET_BYTES      (PING_MSG_SPARE_HEADER_OFFSET_BITS / 8)
#define PING_MSG_SPARE_HEADER_LENGTH_BITS       16

// payload
#define PING_MSG_PAYLOAD_OFFSET_BITS            (PING_MSG_SPARE_HEADER_OFFSET_BITS + PING_MSG_SPARE_HEADER_LENGTH_BITS )
#define PING_MSG_PAYLOAD_OFFSET_BYTES           (PING_MSG_PAYLOAD_OFFSET_BITS / 8)
#define PING_MSG_PAYLOAD_LENGTH_BITS            (PING_OVERALL_SIZE_BITS - PING_MSG_HEADER_LENGTH_BITS - PING_MSG_ROUTE_ADDR_LENGTH_BITS - PING_MSG_HOP_COUNT_ADDR_LENGTH_BITS - PING_MSG_SRC_NODE_ADDR_LENGTH_BITS - PING_MSG_DEST_NODE_ADDR_LENGTH_BITS - PING_MSG_FAULT_NODE_ADDR_LENGTH_BITS - PING_MSG_REPORT_NODE_ADDR_LENGTH_BITS - PING_MSG_SPARE_HEADER_LENGTH_BITS)
#define PING_MSG_PAYLOAD_LENGTH_BYTES           (PING_MSG_PAYLOAD_LENGTH_BITS / 8)


// ack msg lengths
#define ACK_OVERALL_SIZE_BYTES                  18
#define ACK_OVERALL_SIZE_BITS                   (ACK_OVERALL_SIZE_BYTES * 8)

#define ACK_MSG_HEADER_LENGTH_BITS              4
#define ACK_MSG_ROUTE_ADDR_LENGTH_BITS          4

#define ACK_MSG_SRC_NODE_ADDR_OFFSET_BITS       (ACK_MSG_HEADER_LENGTH_BITS + ACK_MSG_ROUTE_ADDR_LENGTH_BITS)
#define ACK_MSG_SRC_NODE_ADDR_OFFSET_BYTES      (ACK_MSG_SRC_NODE_ADDR_OFFSET_BITS / 8)
#define ACK_MSG_SRC_NODE_ADDR_LENGTH_BITS       16

#define ACK_MSG_DEST_NODE_ADDR_OFFSET_BITS      (ACK_MSG_SRC_NODE_ADDR_OFFSET_BITS + ACK_MSG_SRC_NODE_ADDR_LENGTH_BITS)
#define ACK_MSG_DEST_NODE_ADDR_OFFSET_BYTES     (ACK_MSG_DEST_NODE_ADDR_OFFSET_BITS / 8)
#define ACK_MSG_DEST_NODE_ADDR_LENGTH_BITS      16

#define ACK_MSG_FAULT_NODE_ADDR_OFFSET_BITS     (ACK_MSG_DEST_NODE_ADDR_OFFSET_BITS + ACK_MSG_DEST_NODE_ADDR_LENGTH_BITS)
#define ACK_MSG_FAULT_NODE_ADDR_OFFSET_BYTES    (ACK_MSG_FAULT_NODE_ADDR_OFFSET_BITS / 8)
#define ACK_MSG_FAULT_NODE_ADDR_LENGTH_BITS     16

// spare header
#define ACK_MSG_SPARE_HEADER_OFFSET_BITS       (ACK_MSG_FAULT_NODE_ADDR_OFFSET_BITS + ACK_MSG_FAULT_NODE_ADDR_LENGTH_BITS)
#define ACK_MSG_SPARE_HEADER_OFFSET_BYTES      (ACK_MSG_SPARE_HEADER_OFFSET_BITS / 8)
#define ACK_MSG_SPARE_HEADER_LENGTH_BITS       32

// payload
#define ACK_MSG_PAYLOAD_OFFSET_BITS            (ACK_MSG_SPARE_HEADER_OFFSET_BITS + ACK_MSG_SPARE_HEADER_LENGTH_BITS )
#define ACK_MSG_PAYLOAD_OFFSET_BYTES           (ACK_MSG_PAYLOAD_OFFSET_BITS / 8)
#define ACK_MSG_PAYLOAD_LENGTH_BITS            (ACK_OVERALL_SIZE_BITS - ACK_MSG_HEADER_LENGTH_BITS - ACK_MSG_ROUTE_ADDR_LENGTH_BITS - ACK_MSG_SRC_NODE_ADDR_LENGTH_BITS - ACK_MSG_DEST_NODE_ADDR_LENGTH_BITS - ACK_MSG_FAULT_NODE_ADDR_LENGTH_BITS - ACK_MSG_SPARE_HEADER_LENGTH_BITS)
#define ACK_MSG_PAYLOAD_LENGTH_BYTES           (ACK_MSG_PAYLOAD_LENGTH_BITS / 8)


#define PING_ADDR_NO_NEXT_HOP                   0

 bool isPingMsg(uint8_t buffer[], uint16_t length);

 bool isResetPingMsg(uint8_t buffer[], uint16_t length);

 bool isReportPingMsg(uint8_t buffer[], uint16_t length);

 bool isPingOnOurRoute(uint8_t buffer[], uint8_t ourRoute);

 bool isPingAnAutoPing(uint8_t buffer[]);

 bool hasRouteFormed(uint8_t buffer[]);

 bool isPingAFirmwareUpdatePing(uint8_t buffer[]);

 bool isPingAFirmwareUpdateConfigPing(uint8_t buffer[]);

 uint8_t getHopNumberFromPing(uint8_t buffer[]);

 uint8_t geContentionWindowSizeFromPing(uint8_t buffer[]);

 uint16_t getSrcAddressFromPing(uint8_t buffer[]);

 uint16_t getDestAddressFromPing(uint8_t buffer[]);

 uint16_t getFaultAddressFromPing(uint8_t buffer[]);

 uint16_t getReportAddressFromPing(uint8_t buffer[]);

 bool isPingWithNoNextHop(uint8_t buffer[]);

 bool isPingForUs(uint8_t buffer[], uint16_t ourAddress);

 bool isPingFrom(uint8_t buffer[], uint16_t fromAddress);

 void formPingMessageHeader(uint8_t header, uint8_t route, uint8_t hopCountOrContentionWindowSize, uint16_t ourAddress, uint16_t nextHopAddress, uint16_t faultAddress, uint16_t reportAddress, uint8_t buffer[]);

 bool isAckMsg(uint8_t buffer[], uint16_t length);

 bool isBsAckMsg(uint8_t buffer[]);

 bool isAckWithFaultMsg(uint8_t buffer[]);

 bool isReportAckMsg(uint8_t buffer[], uint16_t length);

 bool isNakMsg(uint8_t buffer[], uint16_t length);

 bool  isResetAckMsg(uint8_t buffer[], uint16_t length);

 bool isAckOrNakOnOurRoute(uint8_t buffer[], uint8_t ourRoute);

 uint16_t getSrcAddressFromAck(uint8_t buffer[]);

 uint16_t getDestAddressFromAck(uint8_t buffer[]);

 uint16_t getFaultAddressFromAck(uint8_t buffer[]);

 void formAckOrNakMessage(uint8_t header, uint8_t route, uint16_t srcAddress, uint16_t destAddress, uint16_t faultAddress, uint8_t buffer[]);

#endif /* __NODE_MSGS_H__ */
