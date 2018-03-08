#ifndef ROVER_SDUTILS_H
#define ROVER_SDUTILS_H

#include <SD.h>
#include "../sensors/Camera.h"

/**
 * Wiring instructions:
 * CLK: 52
 * DO:  50
 * DI:  51
 * CS:  53
 **/

namespace Rover {

// or call Servo, and wrap the servo library into here. Allows easier changing.
class SDUtils {
  public:
    SDUtils(int chipSelect = 53);
    void writeImage(Camera &cam);
    File* getFile() { return &currFile; };

  private:
    File currFile;
};

} // namespace Rover
#endif // ROVER_SERVOUTILS_H
