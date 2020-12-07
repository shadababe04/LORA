/*
 * node_msgs.c
 *
 *  Created on: 27 Feb 2018
 *      Author: TE192184
 */

#include <string.h> // for memcpy

#include "node_msgs.h"

 bool isPingMsg(uint8_t buffer[], uint16_t length) {
    if (length != PING_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    // check for all types of ping that we want to treat in the same way
    if ((header != PING_MSG_HEADER_ROUTE_FORMING)
            && (header != PING_MSG_HEADER_ROUTE_FORMED)
            && (header != PING_MSG_HEADER_ROUTE_FORMING_AUTO_PING)
            && (header != PING_MSG_HEADER_ROUTE_FORMED_AUTO_PING)
            && (header != PING_MSG_HEADER_FIRMWARE_UPDATE_CONFIG)
            && (header != PING_MSG_HEADER_FIRMWARE_UPDATE_SENDING)) {
        return false;
    }
    return true;
}

// some pings we treat differently from the start, and are not included in above list (reset pings and report pings sent from non-route nodes)

 bool isResetPingMsg(uint8_t buffer[], uint16_t length) {
    if (length != PING_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != PING_MSG_HEADER_RESET_ROUTE) {
        return false;
    }
    return true;
}


 bool isReportPingMsg(uint8_t buffer[], uint16_t length) {
    if (length != PING_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != PING_MSG_HEADER_NON_ROUTE_NODE_DATA) {
        return false;
    }
    return true;
}


 bool isPingOnOurRoute(uint8_t buffer[], uint8_t ourRoute) {
    // assume it has already been tested as a ping msg

    // extract the route
    uint8_t route =  (buffer[0] & 0x0F);

    if (route != ourRoute) {
        return false;
    }

    return true;
}

 bool isPingAnAutoPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if ((header != PING_MSG_HEADER_ROUTE_FORMING_AUTO_PING) &&
        (header != PING_MSG_HEADER_ROUTE_FORMED_AUTO_PING)) {
        return false;
    }
    return true;
}


 bool hasRouteFormed(uint8_t buffer[]) {
    // assume it has already been tested as a ping msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if ((header != PING_MSG_HEADER_ROUTE_FORMED) &&
        (header != PING_MSG_HEADER_ROUTE_FORMED_AUTO_PING) &&
        (header != PING_MSG_HEADER_FIRMWARE_UPDATE_CONFIG) &&
        (header != PING_MSG_HEADER_FIRMWARE_UPDATE_SENDING)) {
        return false;
    }
    return true;
}


 bool isPingAFirmwareUpdatePing(uint8_t buffer[]) {
    // assume it has already been tested as a ping msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if ((header != PING_MSG_HEADER_FIRMWARE_UPDATE_CONFIG) &&
        (header != PING_MSG_HEADER_FIRMWARE_UPDATE_SENDING)) {
        return false;
    }
    return true;
}


 bool isPingAFirmwareUpdateConfigPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != PING_MSG_HEADER_FIRMWARE_UPDATE_CONFIG) {
        return false;
    }
    return true;
}




 uint8_t getHopNumberFromPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping, and on correct route

    // extract the hop count from the correct location of the buffer
    return buffer[PING_MSG_HOP_COUNT_ADDR_OFFSET_BYTES];
}

 uint8_t geContentionWindowSizeFromPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping, and on correct route

    // currently overlaid with the hop number(hop if route not formed, contention when route is formed)
    return getHopNumberFromPing(buffer);
}


 uint16_t getSrcAddressFromPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping, and on correct route

    // get offset to dst address of ping message buffer
    uint8_t *srcAddressPtr = &buffer[PING_MSG_SRC_NODE_ADDR_OFFSET_BYTES];

    uint16_t srcAddress;

    // extract address
    srcAddress = srcAddressPtr[0] << 8;
    srcAddress += srcAddressPtr[1];

    // and return the address
    return (srcAddress);
}


 uint16_t getDestAddressFromPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping, and on correct route

    // get offset to dst address of ping message buffer
    uint8_t *destAddressPtr = &buffer[PING_MSG_DEST_NODE_ADDR_OFFSET_BYTES];

    uint16_t destAddress;

    // extract address
    destAddress = destAddressPtr[0] << 8;
    destAddress += destAddressPtr[1];

    // and return the address
    return (destAddress);
}

 uint16_t getFaultAddressFromPing(uint8_t buffer[]) {
    // assume it has already been tested as an auto-ping, and on correct route

    // get offset to fault address of ping message buffer
    uint8_t *faultAddressPtr = &buffer[PING_MSG_FAULT_NODE_ADDR_OFFSET_BYTES];

    uint16_t faultAddress;

    // extract address
    faultAddress = faultAddressPtr[0] << 8;
    faultAddress += faultAddressPtr[1];

    // and return the address
    return (faultAddress);
}

 uint16_t getReportAddressFromPing(uint8_t buffer[]) {
    // assume it has already been tested as a ping, and on correct route

    // get offset to report address of ping message buffer
    uint8_t *reportAddressPtr = &buffer[PING_MSG_REPORT_NODE_ADDR_OFFSET_BYTES];

    uint16_t reportAddress;

    // extract address
    reportAddress = reportAddressPtr[0] << 8;
    reportAddress += reportAddressPtr[1];

    // and return the address
    return (reportAddress);
}

 bool isPingWithNoNextHop(uint8_t buffer[]) {
    // assume it has already been tested as a ping msg and on correct route

    return (getDestAddressFromPing(buffer) == PING_ADDR_NO_NEXT_HOP);
}


 bool isPingForUs(uint8_t buffer[], uint16_t ourAddress) {
    // assume it has already been tested as a ping msg, and on correct route

    return (getDestAddressFromPing(buffer) == ourAddress);
}

 bool isPingFrom(uint8_t buffer[], uint16_t fromAddress) {
    // assume it has already been tested as a ping msg, and on correct route

    return (getSrcAddressFromPing(buffer) == fromAddress);
}

 void formPingMessageHeader(uint8_t header, uint8_t route, uint8_t hopCountOrContentionWindowSize, uint16_t ourAddress, uint16_t nextHopAddress, uint16_t faultAddress, uint16_t reportAddress, uint8_t buffer[]) {

    uint8_t *fieldPtr;
    // start with adding the header and route address
    buffer[0] = (header << 4) + (route & 0x0F);
    // then hop count / contention window size
    buffer[PING_MSG_HOP_COUNT_ADDR_OFFSET_BYTES] = hopCountOrContentionWindowSize;
    // then src address
    fieldPtr = &buffer[PING_MSG_SRC_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((ourAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(ourAddress & 0xFF);
    // then next hop (dest) address
    fieldPtr = &buffer[PING_MSG_DEST_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((nextHopAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(nextHopAddress & 0xFF);
    // then faulty node address (zero if no faulty node)
    fieldPtr = &buffer[PING_MSG_FAULT_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((faultAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(faultAddress & 0xFF);
    // then report node address (zero for now until implemented)
    fieldPtr = &buffer[PING_MSG_REPORT_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((reportAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(reportAddress & 0xFF);
    // and clear the payload area too
    memset(&buffer[PING_MSG_PAYLOAD_OFFSET_BYTES], 0, PING_MSG_PAYLOAD_LENGTH_BYTES);
}


 bool isAckMsg(uint8_t buffer[], uint16_t length) {
    if (length != ACK_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if ((header != ACK_MSG_HEADER_REPLY_ACK) && (header != ACK_MSG_HEADER_REPLY_BS_ACK) && (header != ACK_MSG_HEADER_REPLY_ACK_ROUTE_FAULT)) {
        return false;
    }
    return true;
}


 bool isBsAckMsg(uint8_t buffer[]) {
    // assume it has already been tested as an ack msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != ACK_MSG_HEADER_REPLY_BS_ACK) {
        return false;
    }
    return true;
}


 bool isAckWithFaultMsg(uint8_t buffer[]) {
    // assume it has already been tested as an ack msg

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != ACK_MSG_HEADER_REPLY_ACK_ROUTE_FAULT) {
        return false;
    }
    return true;
}



 bool isReportAckMsg(uint8_t buffer[], uint16_t length) {
    if (length != ACK_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != ACK_MSG_HEADER_REPLY_REPORT_ACK) {
        return false;
    }
    return true;
}

 bool isNakMsg(uint8_t buffer[], uint16_t length) {
    if (length != ACK_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != ACK_MSG_HEADER_REPLY_NAK) {
        return false;
    }
    return true;

}

 bool  isResetAckMsg(uint8_t buffer[], uint16_t length) {
    if (length != ACK_OVERALL_SIZE_BYTES) {
        return false;
    }

    uint8_t header = (buffer[0] >> 4) & 0x0F;

    if (header != ACK_MSG_HEADER_REPLY_RESET_ROUTE) {
        return false;
    }
    return true;

}

 bool isAckOrNakOnOurRoute(uint8_t buffer[], uint8_t ourRoute) {
    // assume it has already been tested as an ack or nak msg

    // extract the route
    uint8_t route =  (buffer[0] & 0x0F);

    if (route != ourRoute) {
        return false;
    }

    return true;
}

 uint16_t getSrcAddressFromAck(uint8_t buffer[]) {
    // assume it has already been tested as a ack or nak msg, and on correct route

     // get offset to src address of ack message buffer
     uint8_t *addressPtr = &buffer[ACK_MSG_SRC_NODE_ADDR_OFFSET_BYTES];
     uint16_t address;

     // extract address
     address = addressPtr[0] << 8;
     address += addressPtr[1];

    // and return the address
    return (address);
}

 uint16_t getDestAddressFromAck(uint8_t buffer[]) {
    // assume it has already been tested as a ack or nak msg, and on correct route

     // get offset to dest address of ack message buffer
     uint8_t *addressPtr = &buffer[ACK_MSG_DEST_NODE_ADDR_OFFSET_BYTES];
     uint16_t address;

     // extract address
     address = addressPtr[0] << 8;
     address += addressPtr[1];

    // and return the address
    return (address);
}

 uint16_t getFaultAddressFromAck(uint8_t buffer[]) {
    // assume it has already been tested as a ack or nak msg, and on correct route

     // get offset to dest address of ack message buffer
     uint8_t *addressPtr = &buffer[ACK_MSG_FAULT_NODE_ADDR_OFFSET_BYTES];
     uint16_t address;

     // extract address
     address = addressPtr[0] << 8;
     address += addressPtr[1];

    // and return the address
    return (address);
}

 void formAckOrNakMessage(uint8_t header, uint8_t route, uint16_t srcAddress, uint16_t destAddress, uint16_t faultAddress, uint8_t buffer[]) {

    uint8_t *fieldPtr;
    // start with adding the header and route address
    buffer[0] = (header << 4) + (route & 0x0F);
    // then src address
    fieldPtr = &buffer[ACK_MSG_SRC_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((srcAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(srcAddress & 0xFF);
    // then dest address
    fieldPtr = &buffer[ACK_MSG_DEST_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((destAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(destAddress & 0xFF);
    // then fault address (optional)
    fieldPtr = &buffer[ACK_MSG_FAULT_NODE_ADDR_OFFSET_BYTES];
    fieldPtr[0] = (uint8_t)((faultAddress >> 8) & 0xFF);
    fieldPtr[1] = (uint8_t)(faultAddress & 0xFF);
}
