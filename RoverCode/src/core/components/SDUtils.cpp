#include "SDUtils.h"

namespace Rover {

SDUtils::SDUtils(int chipSelect) {
    if (!SD.begin(chipSelect)) {
        // ERR
    }
}

void SDUtils::writeImage(Camera &cam) {
    char filename[13];
    strcpy(filename, "DOOR00.JPG");
    for (int i = 0; i < 100; i++) {
        filename[4] = '0' + i / 10;
        filename[5] = '0' + i % 10;
        // create if does not exist, do not open existing, write, sync after
        // write
        if (!SD.exists(filename)) {
            break;
        }
    }

    currFile = SD.open(filename, FILE_WRITE);

    uint16_t jpglen = cam.frameLength();
    Serial.println(jpglen);
    while (jpglen > 0) {
        uint8_t *buffer;
        uint8_t bytesToRead = min(64, jpglen);
        buffer = cam.readPicture(bytesToRead);
        currFile.write(buffer, bytesToRead);
        jpglen -= bytesToRead;
    }
    currFile.close();

    Serial.println("Write complete");
    if (!SD.exists(filename)) {
        Serial.println("File not found");
    }
}

} // namespace Rover
