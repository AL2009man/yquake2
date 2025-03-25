#ifndef GAMEPAD_MOTION_H
#define GAMEPAD_MOTION_H

#define GamepadMotion_WRAPPER __attribute__((visibility("default")))

#ifdef __cplusplus
extern "C" {
#endif

    // Full definition of GamepadMotion struct
    typedef struct GamepadMotion {
        float gyroX, gyroY, gyroZ;
        float accelX, accelY, accelZ;
        float gravX, gravY, gravZ;
        float deltaTime;
    } GamepadMotion;

    // C-Compatible Interface Declarations
    GamepadMotion_WRAPPER GamepadMotion* CreateGamepadMotion(void);
    GamepadMotion_WRAPPER void DeleteGamepadMotion(GamepadMotion* motion);
    GamepadMotion_WRAPPER void ResetGamepadMotion(GamepadMotion* motion);
    GamepadMotion_WRAPPER void ProcessMotion(GamepadMotion* motion, float gyroX, float gyroY, float gyroZ,
        float accelX, float accelY, float accelZ, float deltaTime);
    GamepadMotion_WRAPPER void GetCalibratedGyro(GamepadMotion* motion, float* x, float* y, float* z);
    GamepadMotion_WRAPPER void GetGravity(GamepadMotion* motion, float* x, float* y, float* z);
    GamepadMotion_WRAPPER void GetProcessedAcceleration(GamepadMotion* motion, float* x, float* y, float* z);
    GamepadMotion_WRAPPER void GetOrientation(GamepadMotion* motion, float* w, float* x, float* y, float* z);
    GamepadMotion_WRAPPER void GetPlayerSpaceGyro(GamepadMotion* motion, float* x, float* y, float yawRelaxFactor);
    GamepadMotion_WRAPPER void GetWorldSpaceGyro(GamepadMotion* motion, float* x, float* y, float sideReductionThreshold);

    // Gyro Calibration Functions
    GamepadMotion_WRAPPER void StartContinuousCalibration(GamepadMotion* motion);
    GamepadMotion_WRAPPER void PauseContinuousCalibration(GamepadMotion* motion);
    GamepadMotion_WRAPPER void ResetContinuousCalibration(GamepadMotion* motion);
    GamepadMotion_WRAPPER void GetCalibrationOffset(GamepadMotion* motion, float* xOffset, float* yOffset, float* zOffset);
    GamepadMotion_WRAPPER void SetCalibrationOffset(GamepadMotion* motion, float xOffset, float yOffset, float zOffset, int weight);
    GamepadMotion_WRAPPER float GetAutoCalibrationConfidence(GamepadMotion* motion);
    GamepadMotion_WRAPPER void SetAutoCalibrationConfidence(GamepadMotion* motion, float newConfidence);
    GamepadMotion_WRAPPER bool GetAutoCalibrationIsSteady(GamepadMotion* motion);
    GamepadMotion_WRAPPER int GetCalibrationMode(GamepadMotion* motion); // Changed to int for C compatibility
    GamepadMotion_WRAPPER void SetCalibrationMode(GamepadMotion* motion, int calibrationMode);
    GamepadMotion_WRAPPER void ResetMotion(GamepadMotion* motion);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Include GamepadMotion.cpp directly to make it accessible in C++
#include "GamepadMotion.cpp"

// Inline functions to adapt pointers for C++
inline void GetCalibratedGyro(GamepadMotion* motion, float* x, float* y, float* z) {
    if (motion && x && y && z) {
        motion->GetCalibratedGyro(*x, *y, *z);
    }
}

inline void GetGravity(GamepadMotion* motion, float* x, float* y, float* z) {
    if (motion && x && y && z) {
        motion->GetGravity(*x, *y, *z);
    }
}

inline void GetPlayerSpaceGyro(GamepadMotion* motion, float* x, float* y, float yawRelaxFactor) {
    if (motion && x && y) {
        motion->GetPlayerSpaceGyro(*x, *y, yawRelaxFactor);
    }
}

inline void GetWorldSpaceGyro(GamepadMotion* motion, float* x, float* y, float sideReductionThreshold) {
    if (motion && x && y) {
        motion->GetWorldSpaceGyro(*x, *y, sideReductionThreshold);
    }
}

#endif // __cplusplus

#endif // GAMEPAD_MOTION_H
