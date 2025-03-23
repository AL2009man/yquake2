/*
 * =======================================================================
 *
 * Gyro Space header
 * Provides functionality for transforming gyro inputs into Local Space,
 * Player Space, and World Space, while handling sensitivity adjustments
 * and gravity alignment. Compatible with both C and C++ environments.
 *
 * Based on the work by Jibb Smart (JoyShockMapper, GamepadMotionHelpers,
 * Fortnite v.19.30's Gyro Aim/Flick Stick implementation)
 *
 * Links:
 * http://gyrowiki.jibbsmart.com/blog:player-space-gyro-and-alternatives-explained
 * https://github.com/JibbSmart/GamepadMotionHelpers
 *
 * =======================================================================
 */

#ifndef GYROSPACE_H
#define GYROSPACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cmath>
#include <cstdio>
#include <cstring>

    // ---- Type Definitions ----

    struct Vector3 {
        float x, y, z;
    };

    struct Matrix4 {
        float m[4][4];  // Transformation matrix
    };

    // ---- Utility Functions ----

    // Clamp value between min and max
    inline float clamp(float value, float min, float max) {
        return std::max(std::min(value, max), min);
    }

    // ---- Vector Operations ----

    // Create a new vector
    inline Vector3 Vec3_New(float x, float y, float z) {
        return { x, y, z };
    }

    // Subtract one vector from another
    inline Vector3 Vec3_Subtract(Vector3 a, Vector3 b) {
        return Vec3_New(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    // Scale a vector by a scalar
    inline Vector3 Vec3_Scale(Vector3 v, float scalar) {
        return Vec3_New(v.x * scalar, v.y * scalar, v.z * scalar);
    }

    // Compute the dot product of two vectors
    inline float Vec3_Dot(Vector3 a, Vector3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    // Normalize a vector (throws warning if near-zero vector)
    inline Vector3 Vec3_Normalize(Vector3 v) {
        float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
        if (lengthSquared < EPSILON) {
            std::cerr << "Warning: Attempted to normalize a near-zero vector." << std::endl;
            return Vec3_New(0.0f, 0.0f, 0.0f);
        }
        return Vec3_Scale(v, 1.0f / std::sqrt(lengthSquared));
    }

    // ---- Matrix Operations ----

    // Multiply a vector by a matrix (row-major order)
    inline Vector3 MultiplyMatrixVector(Matrix4 matrix, Vector3 vector) {
        return {
            matrix.m[0][0] * vector.x + matrix.m[1][0] * vector.y + matrix.m[2][0] * vector.z,
            matrix.m[0][1] * vector.x + matrix.m[1][1] * vector.y + matrix.m[2][1] * vector.z,
            matrix.m[0][2] * vector.x + matrix.m[1][2] * vector.y + matrix.m[2][2] * vector.z
        };
    }

    // ---- Gyro Space Transformations ----

    // Local Space Transformation
    static Vector3 TransformToLocalSpace(float yaw, float pitch, float roll) {
        // Apply coupling factor for Yaw-Roll interaction
        float couplingFactor = 0.1f;
        float adjustedRoll = roll - (yaw * couplingFactor);
        Vector3 adjustedGyro = Vec3_New(yaw, pitch, adjustedRoll);

        // Define Local Transformation Matrix
        Matrix4 localTransformMatrix = {
            .m = {
                {1.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 1.0f}
            }
        };
        Vector3 localGyro = MultiplyMatrixVector(localTransformMatrix, adjustedGyro);
        localGyro.z = -localGyro.z;  // Lean Fix
        return localGyro;
    }

    // Player Space Transformation
    inline Vector3 TransformToPlayerSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm) {
        // Normalize gravity vector
        if (Vec3_IsZero(gravNorm)) {
            gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);
            std::cerr << "Warning: gravNorm was zero, defaulting to (0, 1, 0)" << std::endl;
        }
        gravNorm = Vec3_Normalize(gravNorm);

        // Adjust Yaw and Roll inputs based on gravity alignment
        float yawSensitivity = 1.0f;   // Sensitivity scaling for Yaw
        float rollSensitivity = 1.0f;  // Sensitivity scaling for Roll
        float adjustedYaw = (yaw_input * yawSensitivity) * gravNorm.y + pitch_input * gravNorm.z;
        float adjustedRoll = (roll_input * rollSensitivity) * gravNorm.x;
        Vector3 adjustedGyro = Vec3_New(adjustedYaw, pitch_input, adjustedRoll);

        // Apply Player View Matrix Transformation
        Matrix4 playerViewMatrix = {
            .m = {
                {1.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 1.0f}
            }
        };
        return MultiplyMatrixVector(playerViewMatrix, adjustedGyro);
    }

    // World Space Transformation
    inline Vector3 TransformToWorldSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm) {
        // Normalize gravity vector
        if (Vec3_IsZero(gravNorm)) {
            gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);
            std::cerr << "Warning: gravNorm was zero, defaulting to (0, 1, 0)" << std::endl;
        }
        gravNorm = Vec3_Normalize(gravNorm);

        // Map inputs to World Space axes
        float yawSensitivity = 1.0f;   // Sensitivity scaling for Yaw
        float rollSensitivity = 1.0f;  // Sensitivity scaling for Roll
        Vector3 rawGyro = Vec3_New(
            pitch_input,
            -yaw_input * yawSensitivity,
            roll_input * rollSensitivity
        );

        // Align pitch inputs with gravity
        float gravDotPitch = Vec3_Dot(gravNorm, Vec3_New(1.0f, 0.0f, 0.0f));
        Vector3 pitchAxis = Vec3_Subtract(Vec3_New(1.0f, 0.0f, 0.0f), Vec3_Scale(gravNorm, gravDotPitch));
        if (!Vec3_IsZero(pitchAxis)) {
            pitchAxis = Vec3_Normalize(pitchAxis);
        }

        return Vec3_New(
            -Vec3_Dot(rawGyro, gravNorm),  // Yaw Alignment
            Vec3_Dot(rawGyro, pitchAxis), // Pitch Alignment
            rawGyro.z                     // Roll Mapping
        );
    }

#ifdef __cplusplus
}
#endif

#endif // GYROSPACE_H
