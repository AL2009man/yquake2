/*
 * =======================================================================
 *
 * Gyro Space header.
 *
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
#include <cmath>
#include <cstdio>
#include <cstdbool>
extern "C" {
#else
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#endif

#ifndef EPSILON
#define EPSILON 1e-5
#endif

 // ---- Debugging and Logging ----
#ifdef ENABLE_DEBUG_LOGS
#define DEBUG_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...)
#endif

// ---- Type Definitions ----

typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    float m[4][4]; // Placeholder for matrix logic
} Matrix4;

// ---- Utility Functions ----

// Clamps a value between a specified minimum and maximum
static inline float clamp(float value, float min, float max) {
    return fmaxf(fminf(value, max), min);
}

// ---- Vector Operations ----

/**
 * Creates a new vector.
 */
static inline Vector3 Vec3_New(float x, float y, float z) {
    return (Vector3) { x, y, z };
}

/**
 * Subtracts one vector from another.
 */
static inline Vector3 Vec3_Subtract(Vector3 a, Vector3 b) {
    return Vec3_New(a.x - b.x, a.y - b.y, a.z - b.z);
}

/**
 * Scales a vector by a scalar.
 */
static inline Vector3 Vec3_Scale(Vector3 v, float scalar) {
    return Vec3_New(v.x * scalar, v.y * scalar, v.z * scalar);
}

/**
 * Computes the dot product of two vectors.
 */
static inline float Vec3_Dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Computes the cross product of two vectors.
 */
static inline Vector3 Vec3_Cross(Vector3 a, Vector3 b) {
    return Vec3_New(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

/**
 * Computes the magnitude (length) of a vector.
 */
static inline float Vec3_Magnitude(Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Normalizes a vector (returns a zero vector if length is near 0).
 */
static inline Vector3 Vec3_Normalize(Vector3 v) {
    float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
    if (lengthSquared < EPSILON) {
        DEBUG_LOG("Warning: Attempted to normalize a near-zero vector.\n");
        return Vec3_New(0.0f, 0.0f, 0.0f);
    }
    return Vec3_Scale(v, 1.0f / sqrtf(lengthSquared));
}

/**
 * Checks if a vector is near-zero.
 */
static inline bool Vec3_IsZero(Vector3 v) {
    return (fabsf(v.x) < EPSILON && fabsf(v.y) < EPSILON && fabsf(v.z) < EPSILON);
}

/**
 * Linearly interpolates between two vectors.
 */
static inline Vector3 Vec3_Lerp(Vector3 a, Vector3 b, float t) {
    return Vec3_New(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z)
    );
}

/**
 * Reflects a vector about a normal.
 */
static inline Vector3 Vec3_Reflect(Vector3 v, Vector3 normal) {
    return Vec3_Subtract(v, Vec3_Scale(normal, 2.0f * Vec3_Dot(v, normal)));
}

// ---- Matrix Operations ----

/**
 * Multiplies a vector by a matrix (row-major).
 */
static inline Vector3 MultiplyMatrixVector(Matrix4 matrix, Vector3 vector) {
    return (Vector3) {
        matrix.m[0][0] * vector.x + matrix.m[1][0] * vector.y + matrix.m[2][0] * vector.z,
            matrix.m[0][1] * vector.x + matrix.m[1][1] * vector.y + matrix.m[2][1] * vector.z,
            matrix.m[0][2] * vector.x + matrix.m[1][2] * vector.y + matrix.m[2][2] * vector.z
    };
}

/**
 * Generates an identity matrix.
 */
static inline Matrix4 Matrix4_Identity() {
    return (Matrix4) {
        .m = {
{1.0f, 0.0f, 0.0f, 0.0f},
{0.0f, 1.0f, 0.0f, 0.0f},
{0.0f, 0.0f, 1.0f, 0.0f},
{0.0f, 0.0f, 0.0f, 1.0f}
        }
    };
}

// ---- Global Gravity Vector Management ----

static Vector3 gravNorm = { 0.0f, 1.0f, 0.0f }; // Default gravity vector

/**
 * Sets a new gravity vector.
 */
static inline void SetGravityVector(float x, float y, float z) {
    if (isnan(x) || isnan(y) || isnan(z)) {
        DEBUG_LOG("Error: Gravity vector contains NaN values. Retaining default value.\n");
        return;
    }
    Vector3 newGravNorm = Vec3_New(x, y, z);
    if (Vec3_IsZero(newGravNorm)) {
        DEBUG_LOG("Warning: Gravity vector cannot be zero. Retaining default value.\n");
        return;
    }
    gravNorm = Vec3_Normalize(newGravNorm);
}

/**
 * Resets gravity vector to default.
 */
static inline void ResetGravityVector(void) {
    gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);
}


// ---- Gyro Space Transformations ----

/**
 * Local Space Transformation
 * Transforms gyro inputs to Local Space using a transformation matrix.
 */
static Vector3 TransformToLocalSpace(float yaw, float pitch, float roll,
    float yawSensitivity, float pitchSensitivity, float rollSensitivity, float couplingFactor) {
    // ---- Adjust Roll and Combine Inputs ----
    float adjustedRoll = (roll * rollSensitivity) - (yaw * couplingFactor);
    Vector3 rawGyro = Vec3_New(
        yaw * yawSensitivity - adjustedRoll,
        pitch * pitchSensitivity,
        0.0f
    );

    // ---- Define Local Transformation Matrix ----
    Matrix4 localTransformMatrix = Matrix4_Identity(); // Identity matrix for simplicity

    // ---- Apply Transformation ----
    Vector3 localGyro = MultiplyMatrixVector(localTransformMatrix, rawGyro);

    // ---- Lean Fix for Roll ----
    localGyro.z = -localGyro.z;

    // ---- Return the Transformed Vector ----
    return localGyro;
}

/**
 * Player Space Transformation
 * Transforms gyro inputs to Player Space, considering gravity alignment.
 */
static Vector3 TransformToPlayerSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm,
    float yawSensitivity, float pitchSensitivity, float rollSensitivity) {
    // ---- Normalize Gravity Vector ----
    if (Vec3_IsZero(gravNorm)) {
        gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);  // Default gravity vector
        DEBUG_LOG("Warning: gravNorm was zero, defaulting to (0, 1, 0)\n");
    }
    gravNorm = Vec3_Normalize(gravNorm);

    // ---- Adjust Inputs ----
    float adjustedYaw = (yaw_input * yawSensitivity) * gravNorm.y + pitch_input * gravNorm.z;
    float adjustedPitch = pitch_input * pitchSensitivity;
    float adjustedRoll = (roll_input * rollSensitivity) * gravNorm.x;

    Vector3 adjustedGyro = Vec3_New(adjustedYaw, adjustedPitch, adjustedRoll);

    // ---- Apply Player View Matrix ----
    Matrix4 playerViewMatrix = Matrix4_Identity(); // Placeholder matrix

    Vector3 playerGyro = MultiplyMatrixVector(playerViewMatrix, adjustedGyro);

    // ---- Return the Transformed Vector ----
    return playerGyro;
}

/**
 * World Space Transformation
 * Transforms gyro inputs to World Space, considering gravity orientation.
 */
static Vector3 TransformToWorldSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm,
    float yawSensitivity, float pitchSensitivity, float rollSensitivity) {
    // ---- Normalize Gravity Vector ----
    if (Vec3_IsZero(gravNorm)) {
        gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);  // Default gravity vector
        DEBUG_LOG("Warning: gravNorm was zero, defaulting to (0, 1, 0)\n");
    }
    gravNorm = Vec3_Normalize(gravNorm);

    // ---- Map Inputs ----
    Vector3 rawGyro = Vec3_New(
        pitch_input * pitchSensitivity,
        -yaw_input * yawSensitivity,
        roll_input * rollSensitivity
    );

    // ---- Align Inputs Using Gravity ----
    float gravDotPitch = Vec3_Dot(gravNorm, Vec3_New(1.0f, 0.0f, 0.0f));
    Vector3 pitchAxis = Vec3_Subtract(Vec3_New(1.0f, 0.0f, 0.0f), Vec3_Scale(gravNorm, gravDotPitch));
    if (!Vec3_IsZero(pitchAxis)) {
        pitchAxis = Vec3_Normalize(pitchAxis);
    }

    // ---- Calculate Transformed Values ----
    Vector3 worldGyro = Vec3_New(
        -Vec3_Dot(rawGyro, gravNorm),  // Yaw Alignment
        Vec3_Dot(rawGyro, pitchAxis), // Pitch Alignment
        rawGyro.z                     // Roll Mapping
    );

    // ---- Return the Transformed Vector ----
    return worldGyro;
}


#ifdef __cplusplus
}
#endif

#endif // GYROSPACE_H