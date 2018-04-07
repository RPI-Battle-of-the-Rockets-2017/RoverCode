#include "Telemetry.h"

namespace Rover {

Telemetry::Telemetry(HardwareSerial *ser, HardwareSerial *debug) {
    ser->begin(9600);
    pack_ser.setStream(ser);
    sequence_num = 0;
    debug_ser = debug;

    comms_established = false;
    message_confirmed = true;
    message_in_queue = false;    

    time_false = 0;
}

void Telemetry::update() {
    pack_ser.update();
    //if message_confrmed has been false for a long period of time, start sending establish_comm
    time_false++;
    //debug_ser->print("time_false: ");
    //debug_ser->println(time_false);
    if (time_false > LOOP_THRESHOLD) {
        comms_established = false;
        establishComm();
        time_false -= LOOP_MESSAGE_DELAY;
    }
    if (comms_established && message_in_queue && message_confirmed) {
        sendQueuedPacket();
        message_in_queue = false;
    }
}

void Telemetry::establishComm() {
    //debug_ser->println("Establishing comms");
    sendPacket(0, MSG_TYPE::ESTABLISH_COMM, 0);
}

void Telemetry::initiateImage() {
    while (!comms_established) {
        update();
    }
    sendPacket(0, MSG_TYPE::INITIATE_IMAGE, 0);
    while (!message_confirmed) {
        update();
    }
}

bool Telemetry::sendImage(uint8_t *buffer, uint8_t size) {
    img_sent_size = size;
    sendPacket(buffer, MSG_TYPE::IMAGE, size);
    while (img_result == IMAGE_MSG_RESULT::WAITING) {
        pack_ser.update();
    }
    if (img_result == IMAGE_MSG_RESULT::CORRECT) {
        return true;
    } else {
        return false;
    }
}

void Telemetry::endImage() {
    sendPacket(0, MSG_TYPE::END_IMAGE, 0);
}

void Telemetry::sendBaroHeight(int height) {
    /*
    debug_ser->print("comms_established: ");
    debug_ser->println(comms_established);
    */
    if (comms_established) {
        send_buf[0] = height & 0xFF;
        send_buf[1] = (height >> 8) & 0xFF;
        sendPacket(send_buf, MSG_TYPE::BARO, sizeof(int));
    }
}

void Telemetry::sendGPSData(long latitude, long longitude)
{
    if (comms_established) {
	debug_ser->print("HERE");

        send_buf[0] = latitude & 0xFF;
        send_buf[1] = (latitude >> 8) & 0xFF;
        send_buf[2] = (latitude >> 16) & 0xFF;
        send_buf[3] = (latitude >> 24) & 0xFF;

        send_buf[4] = longitude & 0xFF;
        send_buf[5] = (longitude >> 8) & 0xFF;
        send_buf[6] = (longitude >> 16) & 0xFF;
        send_buf[7] = (longitude >> 24) & 0xFF;

        sendPacket(send_buf, MSG_TYPE::GPS, 2 * sizeof(long));
    }
}

void Telemetry::processTelem(const uint8_t *buffer, uint16_t size) {
    //Size is 1 for normal confirmation
    if (size == 1) {
        if (buffer[0] == sequence_num) {
            sequence_num++;
            message_confirmed = true;
            time_false = 0;
            comms_established = true;

            //debug_ser->println("Confirmation");
        }
    } else if (size == 2) { // 2 for image confirmation
        if (buffer[0] == sequence_num) {
            debug_ser->println(buffer[1]);
            uint8_t image_size = buffer[1];
            if (image_size == img_sent_size) {
                img_result = IMAGE_MSG_RESULT::CORRECT;
            } else {
                img_result = IMAGE_MSG_RESULT::INCORRECT;
            }
            sequence_num++;
            message_confirmed = true;
            time_false = 0;
            comms_established = true;
        }
    }
}

void Telemetry::sendPacket(uint8_t *buffer, MSG_TYPE type, uint8_t len) {
    for (int i = 0; i < BUF_MAX; i++) {
        buf[i] = 0;
    }

    Packet_Header header;
    header.length = sizeof(Packet_Header) + len;
    header.sequence_num = sequence_num;
    header.msg_type = type;

    uint8_t *bytePtr = (uint8_t*)&header;
    memcpy(buf, bytePtr, sizeof(Packet_Header));
    memcpy(buf + sizeof(Packet_Header), buffer, len);

    if (message_confirmed || type == MSG_TYPE::ESTABLISH_COMM) {
        pack_ser.send(buf, sizeof(Packet_Header) + len);
        message_confirmed = false;
        /*
        debug_ser->print("Sending message with sequence_num: ");
        debug_ser->println(sequence_num, HEX);
        */
    } else {
        message_in_queue = true;
    }
}

void Telemetry::sendQueuedPacket() {
    message_in_queue = false;
    pack_ser.send(buf, buf[1]);
}

}
