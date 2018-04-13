#include "SDUtils.h"

namespace Rover {

SDUtils::SDUtils(int chipSelect) {
	int i = 0;
    while (!SD.begin(chipSelect)) {
        // ERR
        Serial.println("SD card not found");
		delay(250);
		i++;
		if (i > 100) break;
    }
}

void SDUtils::writeImage(Camera &cam, Telemetry &tel, int index) {
	/*
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
	*/
	Serial.print("Index: ");
	Serial.println(index);
	
	char filename[13];
	switch(index) {
		case 0:
			strcpy(filename, "FWRD00.JPG");
			break;
		case 1:
			strcpy(filename, "RGHT00.JPG");
			break;
		case 2:
			strcpy(filename, "BACK00.JPG");
			break;
		case 3:
			strcpy(filename, "LEFT00.JPG");
			break;
		default:
			strcpy(filename, "DOOR00.JPG");
			break;
	}
	
	for (int i = 0; i < 100; i++) {
		filename[4] = '0' + i / 10;
		filename[5] = '0' + i % 10;
		
		if (!SD.exists(filename)) {
			break;
		}
	}
	
	bool tele_good;

    currFile = SD.open(filename, FILE_WRITE);
	
   // tele_good = tel.initiateImage();

    uint16_t jpglen = cam.frameLength();
    Serial.println(jpglen);
    while (jpglen > 0) {
        uint8_t *buffer;
        uint8_t bytesToRead = min(4, jpglen);
        //Serial.println("Before read bytes");
        buffer = cam.readPicture(bytesToRead);
        //Serial.println("After read bytes");
        currFile.write(buffer, bytesToRead);
        //while(tel.sendImage(buffer, bytesToRead));
		/*
		if (tele_good) {
			tele_good = tel.sendImage(buffer, bytesToRead);
		}
		delay(1);
		*/
		
        jpglen -= bytesToRead;
    }
    currFile.close();
	
	/*
	if (tele_good) {
		tele_good = tel.endImage();
	}
	*/
	

    Serial.println("Write complete");
    if (!SD.exists(filename)) {
        Serial.println("File not found");
    }
}
}

