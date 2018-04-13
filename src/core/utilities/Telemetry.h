#ifndef ROVER_TELEMETRY
#define ROVER_TELEMETRY

#include "PacketSerial.h"

#define BUF_MAX             32


namespace Rover {
struct Packet_Header{
    unsigned char header = 0xFF;
    unsigned char length;
    unsigned char sequence_num;
    unsigned char msg_type;
};

enum MSG_TYPE{
    ESTABLISH_COMM = 0,
    INITIATE_IMAGE = 1,
    IMAGE = 2,
    END_IMAGE = 3,
    BARO = 4,
    GPS = 5
};

class Telemetry {
public:
        Telemetry(HardwareSerial *ser, HardwareSerial *debug);
        void update();

        PacketSerial pack_ser;
        void processTelem(const uint8_t *buffer, size_t size);

        void sendBaroHeight(int height);
        void sendGPSData(long latitude, long longitude);
        void establishComm();

        bool initiateImage();
        bool sendImage(uint8_t *buffer, uint8_t size);
        bool endImage();
private:
        void sendPacket(uint8_t *buffer, MSG_TYPE type, uint8_t len);
        void sendQueuedPacket();

        unsigned char sequence_num;
        uint8_t buf[BUF_MAX];
        uint8_t send_buf[BUF_MAX];

        HardwareSerial *debug_ser;
        bool comms_established;
        bool message_confirmed;
        bool message_in_queue;

        long time_false;
        const static long LOOP_THRESHOLD = 100000;
        const static long LOOP_MESSAGE_DELAY = 10000;
};

}

#endif // ROVER_STANDBY
