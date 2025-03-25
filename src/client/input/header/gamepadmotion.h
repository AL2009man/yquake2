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
        float gravX, gravY, gravZ;  // Independent gravity vector for motion calculations
        float deltaTime;
    } GamepadMotion;

    // Independent gravity vector for GamepadMotionHelper logic
    static struct {
        float gravX, gravY, gravZ;
    } HelperGravity = { 0.0f, 1.0f, 0.0f }; // Default gravity vector

    // Update independent gravity vector
    static inline void UpdateHelperGravity(float x, float y, float z) {
        float magnitude = sqrt(x * x + y * y + z * z);
        if (magnitude > 0.0f) {
            HelperGravity.gravX = x / magnitude;
            HelperGravity.gravY = y / magnitude;
            HelperGravity.gravZ = z / magnitude;
        }
        else {
            // Reset to default gravity if invalid input
            HelperGravity.gravX = 0.0f;
            HelperGravity.gravY = 1.0f;
            HelperGravity.gravZ = 0.0f;
        }
    }

    // C-Compatible Interface Declarations
    GamepadMotion_WRAPPER GamepadMotion* CreateGamepadMotion(void);
    GamepadMotion_WRAPPER void DeleteGamepadMotion(GamepadMotion* motion);
    GamepadMotion_WRAPPER void ProcessMotion(GamepadMotion* motion, float gyroX, float gyroY, float gyroZ,
        float accelX, float accelY, float accelZ, float deltaTime);
    GamepadMotion_WRAPPER void GetCalibratedGyro(GamepadMotion* motion, float* x, float* y, float* z);
    GamepadMotion_WRAPPER void GetHelperGravity(float* x, float* y, float* z); // Get independent gravity vector
    GamepadMotion_WRAPPER void GetPlayerSpaceGyro(GamepadMotion* motion, float* x, float* y, float yawRelaxFactor);
    GamepadMotion_WRAPPER void GetWorldSpaceGyro(GamepadMotion* motion, float* x, float* y, float sideReductionThreshold);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace {
    class GamepadMotion {
    public:
        float gyroX, gyroY, gyroZ;
        float accelX, accelY, accelZ;
        float gravX, gravY, gravZ; // Gravity tied to GamepadMotion instance
        float deltaTime;

        GamepadMotion() : gyroX(0), gyroY(0), gyroZ(0),
            accelX(0), accelY(0), accelZ(0),
            gravX(HelperGravity.gravX), gravY(HelperGravity.gravY), gravZ(HelperGravity.gravZ),
            deltaTime(0) {
        }

        void ProcessMotion(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
            gyroX = gx; gyroY = gy; gyroZ = gz;
            accelX = ax; accelY = ay; accelZ = az;
            deltaTime = dt;

            // Use independent gravity vector
            UpdateHelperGravity(accelX, accelY, accelZ);
            gravX = HelperGravity.gravX;
            gravY = HelperGravity.gravY;
            gravZ = HelperGravity.gravZ;
        }

        void GetCalibratedGyro(float& x, float& y, float& z) const {
            x = gyroX; y = gyroY; z = gyroZ;
        }

        void GetHelperGravity(float& x, float& y, float& z) const {
            x = HelperGravity.gravX;
            y = HelperGravity.gravY;
            z = HelperGravity.gravZ;
        }

        void GetPlayerSpaceGyro(float& x, float& y, float yawRelaxFactor) const {
            x = gyroX * yawRelaxFactor;
            y = gyroY * yawRelaxFactor;
        }

        void GetWorldSpaceGyro(float& x, float& y, float sideReductionThreshold) const {
            x = gyroX * sideReductionThreshold;
            y = gyroY * sideReductionThreshold;
        }
    };
}

// Inline functions for C++ context
inline GamepadMotion* CreateGamepadMotion(void) {
    return new GamepadMotion();
}

inline void DeleteGamepadMotion(GamepadMotion* motion) {
    delete motion;
}

inline void ProcessMotion(GamepadMotion* motion, float gyroX, float gyroY, float gyroZ,
    float accelX, float accelY, float accelZ, float deltaTime) {
    if (motion) {
        motion->ProcessMotion(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, deltaTime);
    }
}

inline void GetCalibratedGyro(GamepadMotion* motion, float* x, float* y, float* z) {
    if (motion && x && y && z) {
        motion->GetCalibratedGyro(*x, *y, *z);
    }
}

inline void GetHelperGravity(GamepadMotion* motion, float* x, float* y, float* z) {
    if (motion && x && y && z) {
        motion->GetHelperGravity(*x, *y, *z);
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

// Implementations for C context
#ifdef __cplusplus
extern "C" {
#endif

    GamepadMotion* CreateGamepadMotion(void) {
        GamepadMotion* motion = (GamepadMotion*)malloc(sizeof(GamepadMotion));
        if (motion) {
            motion->gyroX = 0.0f;
            motion->gyroY = 0.0f;
            motion->gyroZ = 0.0f;
            motion->accelX = 0.0f;
            motion->accelY = 0.0f;
            motion->accelZ = 0.0f;
            motion->gravX = HelperGravity.gravX; // Independent gravity
            motion->gravY = HelperGravity.gravY;
            motion->gravZ = HelperGravity.gravZ;
            motion->deltaTime = 0.0f;
        }
        return motion;
    }

    void DeleteGamepadMotion(GamepadMotion* motion) {
        free(motion);
    }

    void ProcessMotion(GamepadMotion* motion, float gyroX, float gyroY, float gyroZ,
        float accelX, float accelY, float accelZ, float deltaTime) {
        if (motion) {
            motion->gyroX = gyroX;
            motion->gyroY = gyroY;
            motion->gyroZ = gyroZ;
            motion->accelX = accelX;
            motion->accelY = accelY;
            motion->accelZ = accelZ;

            UpdateHelperGravity(accelX, accelY, accelZ);
            motion->gravX = HelperGravity.gravX;
            motion->gravY = HelperGravity.gravY;
            motion->gravZ = HelperGravity.gravZ;

            motion->deltaTime = deltaTime;
        }
    }

    void GetCalibratedGyro(GamepadMotion* motion, float* x, float* y, float* z) {
        if (motion && x && y && z) {
            *x = motion->gyroX;
            *y = motion->gyroY;
            *z = motion->gyroZ;
        }
    }

    void GetHelperGravity(float* x, float* y, float* z) {
        if (x && y && z) {
            *x = HelperGravity.gravX;
            *y = HelperGravity.gravY;
            *z = HelperGravity.gravZ;
        }
    }

    void GetPlayerSpaceGyro(GamepadMotion* motion, float* x, float* y, float yawRelaxFactor) {
        if (motion && x && y) {
            *x = motion->gyroX * yawRelaxFactor;
            *y = motion->gyroY * yawRelaxFactor;
        }
    }

    void GetWorldSpaceGyro(GamepadMotion* motion, float* x, float* y, float sideReductionThreshold) {
        if (motion && x && y) {
            *x = motion->gyroX * sideReductionThreshold;
            *y = motion->gyroY * sideReductionThreshold;
        }
    }

#ifdef __cplusplus
}
#endif

#endif // GAMEPAD_MOTION_H
