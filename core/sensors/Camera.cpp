#include "Camera.h"

// Initialization code used by all constructor types
void Camera::common_init(void) {
    hwSerial = NULL;
    frameptr = 0;
    bufferLen = 0;
    serialNum = 0;
}

// Constructor when using HardwareSerial
Camera::Camera(HardwareSerial *ser) {
    common_init();  // Set everything to common state, then...
    hwSerial = ser; // ...override hwSerial with value passed.
}

boolean Camera::begin(uint16_t baud) {
    hwSerial->begin(baud);
    return reset();
}

boolean Camera::reset() {
    uint8_t args[] = {0x0};

    return runCommand(VC0706_RESET, args, 1, 5);
}

uint8_t Camera::getImageSize() {
    uint8_t args[] = {0x4, 0x4, 0x1, 0x00, 0x19};
    if (!runCommand(VC0706_READ_DATA, args, sizeof(args), 6))
        return -1;

    return camerabuff[5];
}

boolean Camera::setImageSize(uint8_t x) {
    uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, x};

    return runCommand(VC0706_WRITE_DATA, args, sizeof(args), 5);
}

/****************** downsize image control */

uint8_t Camera::getDownsize(void) {
    uint8_t args[] = {0x0};
    if (!runCommand(VC0706_DOWNSIZE_STATUS, args, 1, 6))
        return -1;

    return camerabuff[5];
}

boolean Camera::setDownsize(uint8_t newsize) {
    uint8_t args[] = {0x01, newsize};

    return runCommand(VC0706_DOWNSIZE_CTRL, args, 2, 5);
}

/***************** other high level commands */

char *Camera::getVersion(void) {
    uint8_t args[] = {0x01};

    sendCommand(VC0706_GEN_VERSION, args, 1);
    // get reply
    if (!readResponse(CAMERABUFFSIZ, 200))
        return 0;
    camerabuff[bufferLen] = 0; // end it!
    return (char *)camerabuff; // return it!
}

/****************** high level photo comamnds */

// Between 0 and 100
boolean Camera::setCompression(uint8_t c) {
    uint8_t args[] = {0x5, 0x1, 0x1, 0x12, 0x04, c};
    return runCommand(VC0706_WRITE_DATA, args, sizeof(args), 5);
}

uint8_t Camera::getCompression(void) {
    uint8_t args[] = {0x4, 0x1, 0x1, 0x12, 0x04};
    runCommand(VC0706_READ_DATA, args, sizeof(args), 6);
    // printBuff();
    return camerabuff[5];
}

boolean Camera::setPTZ(uint16_t wz, uint16_t hz, uint16_t pan,
                                uint16_t tilt) {
    uint8_t args[] = {0x08,     wz >> 8, wz,        hz >> 8, wz,
                      pan >> 8, pan,     tilt >> 8, tilt};

    return (!runCommand(VC0706_SET_ZOOM, args, sizeof(args), 5));
}

boolean Camera::getPTZ(uint16_t &w, uint16_t &h, uint16_t &wz,
                                uint16_t &hz, uint16_t &pan, uint16_t &tilt) {
    uint8_t args[] = {0x0};

    if (!runCommand(VC0706_GET_ZOOM, args, sizeof(args), 16))
        return false;
    // printBuff();

    w = camerabuff[5];
    w <<= 8;
    w |= camerabuff[6];

    h = camerabuff[7];
    h <<= 8;
    h |= camerabuff[8];

    wz = camerabuff[9];
    wz <<= 8;
    wz |= camerabuff[10];

    hz = camerabuff[11];
    hz <<= 8;
    hz |= camerabuff[12];

    pan = camerabuff[13];
    pan <<= 8;
    pan |= camerabuff[14];

    tilt = camerabuff[15];
    tilt <<= 8;
    tilt |= camerabuff[16];

    return true;
}

boolean Camera::takePicture() {
    frameptr = 0;
    return cameraFrameBuffCtrl(VC0706_STOPCURRENTFRAME);
}

boolean Camera::resumeVideo() {
    return cameraFrameBuffCtrl(VC0706_RESUMEFRAME);
}

boolean Camera::cameraFrameBuffCtrl(uint8_t command) {
    uint8_t args[] = {0x1, command};
    return runCommand(VC0706_FBUF_CTRL, args, sizeof(args), 5);
}

uint32_t Camera::frameLength(void) {
    uint8_t args[] = {0x01, 0x00};
    if (!runCommand(VC0706_GET_FBUF_LEN, args, sizeof(args), 9))
        return 0;

    uint32_t len;
    len = camerabuff[5];
    len <<= 8;
    len |= camerabuff[6];
    len <<= 8;
    len |= camerabuff[7];
    len <<= 8;
    len |= camerabuff[8];

    return len;
}

uint8_t Camera::available(void) { return bufferLen; }

uint8_t *Camera::readPicture(uint8_t n) {
    uint8_t args[] = {0x0C,
                      0x0,
                      0x0A,
                      0,
                      0,
                      frameptr >> 8,
                      frameptr & 0xFF,
                      0,
                      0,
                      0,
                      n,
                      CAMERADELAY >> 8,
                      CAMERADELAY & 0xFF};

    if (!runCommand(VC0706_READ_FBUF, args, sizeof(args), 5, false))
        return 0;

    // read into the buffer PACKETLEN!
    if (readResponse(n + 5, CAMERADELAY) == 0)
        return 0;

    frameptr += n;

    return camerabuff;
}

/**************** low level commands */

boolean Camera::runCommand(uint8_t cmd, uint8_t *args, uint8_t argn,
                                    uint8_t resplen, boolean flushflag) {
    // flush out anything in the buffer?
    if (flushflag) {
        readResponse(100, 10);
    }

    sendCommand(cmd, args, argn);
    if (readResponse(resplen, 200) != resplen)
        return false;
    if (!verifyResponse(cmd))
        return false;
    return true;
}

void Camera::sendCommand(uint8_t cmd, uint8_t args[] = 0,
                                  uint8_t argn = 0) {
#if ARDUINO >= 100
    hwSerial->write((byte)0x56);
    hwSerial->write((byte)serialNum);
    hwSerial->write((byte)cmd);

    for (uint8_t i = 0; i < argn; i++) {
        hwSerial->write((byte)args[i]);
        // Serial.print(" 0x");
        // Serial.print(args[i], HEX);
    }
#else
    hwSerial->print(0x56, BYTE);
    hwSerial->print(serialNum, BYTE);
    hwSerial->print(cmd, BYTE);

    for (uint8_t i = 0; i < argn; i++) {
        hwSerial->print(args[i], BYTE);
        // Serial.print(" 0x");
        // Serial.print(args[i], HEX);
    }
#endif
    // Serial.println();
}

uint8_t Camera::readResponse(uint8_t numbytes, uint8_t timeout) {
    uint8_t counter = 0;
    bufferLen = 0;
    int avail;

    while ((timeout != counter) && (bufferLen != numbytes)) {
        avail = hwSerial->available();
        if (avail <= 0) {
            delay(1);
            counter++;
            continue;
        }
        counter = 0;
        // there's a byte!
        camerabuff[bufferLen++] = hwSerial->read();
    }
    // printBuff();
    // camerabuff[bufferLen] = 0;
    // Serial.println((char*)camerabuff);
    return bufferLen;
}

boolean Camera::verifyResponse(uint8_t command) {
    if ((camerabuff[0] != 0x76) || (camerabuff[1] != serialNum) ||
        (camerabuff[2] != command) || (camerabuff[3] != 0x0))
        return false;
    return true;
}

void Camera::printBuff() {
    for (uint8_t i = 0; i < bufferLen; i++) {
        Serial.print(" 0x");
        Serial.print(camerabuff[i], HEX);
    }
    Serial.println();
}
