#ifndef ROVER_VECTOR_H
#define ROVER_VECTOR_H

#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

/**
 * This class defines the 3-axis vector type and provides additional utilities for
 * the 3d vector as well as other vector calculations.
 */

namespace Rover {

//todo: Consider using a class with this union inside?
typedef union {
    float u[3];
    struct {
        float x;
        float y;
        float z;
    };
    /* Orientation sensors */
    struct {
        float roll;     /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -pi/2<=roll<=pi/2 */
        float pitch;    /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -pi°<=pitch<=pi) */
        float heading;  /**< Angle between the longitudinal axis (the plane body) and magnetic east, measured counterclockwise when viewing from the top of the device. 0-2pi */
    };
} Vector;

//It would look like this
//class Vector{
//public:
//    union {
//        float u[3];
//        struct {
//            float x;
//            float y;
//            float z;
//        };
//        /* Orientation sensors */
//        struct {
//            float roll;     /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -pi/2<=roll<=pi/2 */
//            float pitch;    /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -pi°<=pitch<=pi) */
//            float heading;  /**< Angle between the longitudinal axis (the plane body) and magnetic east, measured counterclockwise when viewing from the top of the device. 0-2pi */
//        };
//    };
//
//    //Add utilities here...
//
//};

//Add utilities here...

}

#endif
