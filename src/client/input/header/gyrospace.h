/*
 * =======================================================================
 *
 * This is Gyro Space header. 
 * 
 * Gyro Space (Local Space, Player Space, World Space) is based on the work
 * by Jibb Smart. Creator of JoyShockMapper, GamepadMotionHelpers and
 * Fortnite v19.30's Gyro Aim implementation.
 * 
 * Link:
 * http://gyrowiki.jibbsmart.com/blog:player-space-gyro-and-alternatives-explained
 * https://github.com/JibbSmart/GamepadMotionHelpers
 *
 * =======================================================================
 */

#include "math.h"
#include "stdbool.h"
#include "stdio.h"

#ifndef EPSILON
#define EPSILON 1e-5
#endif

 // ---- Type Definitions ----

typedef struct {
	float x, y, z;
} Vector3;

typedef struct {
	float m[4][4];  // Placeholder for matrix logic
} Matrix4;

// ---- Utility Functions ----

// Clamps a value between a specified minimum and maximum
static inline float clamp(float value, float min, float max) {
	return fmaxf(fminf(value, max), min);
}

// ---- Vector Operations ----

// Creates a new vector
static inline Vector3 Vec3_New(float x, float y, float z) {
	return (Vector3) { x, y, z };
}

// Subtracts one vector from another
static inline Vector3 Vec3_Subtract(Vector3 a, Vector3 b) {
	return Vec3_New(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Scales a vector by a scalar
static inline Vector3 Vec3_Scale(Vector3 v, float scalar) {
	return Vec3_New(v.x * scalar, v.y * scalar, v.z * scalar);
}

// Computes the dot product of two vectors
static inline float Vec3_Dot(Vector3 a, Vector3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Computes the cross product of two vectors
static inline Vector3 Vec3_Cross(Vector3 a, Vector3 b) {
	return Vec3_New(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

// Computes the magnitude (length) of a vector
static inline float Vec3_Magnitude(Vector3 v) {
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Normalizes a vector (returns a zero vector if length is near 0)
static inline Vector3 Vec3_Normalize(Vector3 v) {
	float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
	if (lengthSquared < EPSILON) {
		printf("Warning: Attempted to normalize a near-zero vector.\n");
		return Vec3_New(0.0f, 0.0f, 0.0f);
	}
	return Vec3_Scale(v, 1.0f / sqrtf(lengthSquared));
}

// Checks if a vector is near-zero
static inline bool Vec3_IsZero(Vector3 v) {
	return (fabsf(v.x) < EPSILON && fabsf(v.y) < EPSILON && fabsf(v.z) < EPSILON);
}

// ---- Matrix Operations ----

// Multiplies a vector by a matrix (row-major)
static inline Vector3 MultiplyMatrixVector(Matrix4 matrix, Vector3 vector) {
	return (Vector3) {
		matrix.m[0][0] * vector.x + matrix.m[1][0] * vector.y + matrix.m[2][0] * vector.z,
			matrix.m[0][1] * vector.x + matrix.m[1][1] * vector.y + matrix.m[2][1] * vector.z,
			matrix.m[0][2] * vector.x + matrix.m[1][2] * vector.y + matrix.m[2][2] * vector.z
	};
}

// ---- Global Gravity Vector Management ----

static Vector3 gravNorm = { 0.0f, 1.0f, 0.0f };  // Default gravity vector

// Sets a new gravity vector
static inline void SetGravityVector(float x, float y, float z) {
	Vector3 newGravNorm = Vec3_New(x, y, z);
	if (Vec3_IsZero(newGravNorm)) {
		printf("Warning: Gravity vector cannot be zero. Retaining default value.\n");
		return;
	}
	gravNorm = Vec3_Normalize(newGravNorm);
}

// Resets gravity vector to default
static inline void ResetGravityVector(void) {
	gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);
}

// ---- Gyro Space Transformations ----

// Local Space
// Transforms gyro inputs to Local Space using a transformation matrix
static Vector3 TransformToLocalSpace(float yaw_input, float pitch_input, float roll_input) {
	// ---- Harmonize Yaw and Roll with Balanced Sensitivity ----
	float yawSensitivity = 1.0f;       // Adjust sensitivity for Yaw if necessary
	float rollSensitivity = 1.0f;      // Adjust sensitivity for Roll if necessary
	float couplingFactor = 0.075f;     // Coupling factor for Yaw-Roll interaction

	// Scale Roll and harmonize it with Yaw
	float adjustedRoll = (roll_input * rollSensitivity) - (yaw_input * couplingFactor);

	// ---- Combine Inputs into Gyro Vector ----
	Vector3 rawGyro = Vec3_New(yaw_input * yawSensitivity - adjustedRoll, pitch_input, 0.0f);

	// ---- Define Local View Matrix ----
	Matrix4 localTransformMatrix = {
		.m = {
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 1.0f}
		}
	};

	// ---- Apply Transformation ----
	Vector3 localGyro = MultiplyMatrixVector(localTransformMatrix, rawGyro);

	// ---- Adjust for Lean Fix ----
	localGyro.z = -localGyro.z;

	// ---- Return the Transformed Vector ----
	return localGyro;
}

// Player Space
// Transforms gyro inputs to Player Space, taking into account gravity and player view orientation
static Vector3 TransformToPlayerSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm) {
	// ---- Normalize Gravity Vector ----
	if (Vec3_IsZero(gravNorm)) {
		gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);  // Default gravity vector
		printf("Warning: gravNorm was zero, defaulting to (0, 1, 0)\n");
	}
	gravNorm = Vec3_Normalize(gravNorm);

	// ---- Calculate Adjusted Inputs ----
	float yawSensitivity = 1.0f;  // Fine-tune sensitivity for Yaw if necessary
	float rollSensitivity = 1.0f; // Fine-tune sensitivity for Roll if necessary

	float adjustedYaw = (yaw_input * yawSensitivity) * gravNorm.y + pitch_input * gravNorm.z;
	float adjustedRoll = (roll_input * rollSensitivity) * gravNorm.x;

	// ---- Combine Adjusted Inputs into a Gyro Vector ----
	Vector3 adjustedGyro = Vec3_New(adjustedYaw, pitch_input, adjustedRoll);

	// ---- Apply Player View Matrix ----
	Matrix4 playerViewMatrix = {
		.m = {
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 1.0f}
		}
	};
	Vector3 playerGyro = MultiplyMatrixVector(playerViewMatrix, adjustedGyro);

	// ---- Return the Transformed Vector ----
	return playerGyro;
}

// World Space
// Transforms gyro inputs to World Space, taking into account gravity influence
static Vector3 TransformToWorldSpace(float yaw_input, float pitch_input, float roll_input, Vector3 gravNorm) {
	// ---- Normalize Gravity Vector ----
	if (Vec3_IsZero(gravNorm)) {
		gravNorm = Vec3_New(0.0f, 1.0f, 0.0f);  // Default gravity vector
		printf("Warning: gravNorm was zero, defaulting to (0, 1, 0)\n");
	}
	gravNorm = Vec3_Normalize(gravNorm);

	// ---- Map Inputs to World Space Axes ----
	float yawSensitivity = 1.0f;   // Fine-tune sensitivity for Yaw if necessary
	float rollSensitivity = 1.0f;  // Fine-tune sensitivity for Roll if necessary

	Vector3 rawGyro = Vec3_New(
		pitch_input,
		-yaw_input * yawSensitivity,
		roll_input * rollSensitivity
	);

	// ---- Calculate Perpendicular Alignment ----
	float gravDotPitch = Vec3_Dot(gravNorm, Vec3_New(1.0f, 0.0f, 0.0f));
	Vector3 pitchAxis = Vec3_Subtract(Vec3_New(1.0f, 0.0f, 0.0f), Vec3_Scale(gravNorm, gravDotPitch));
	if (!Vec3_IsZero(pitchAxis)) {
		pitchAxis = Vec3_Normalize(pitchAxis);
	}

	// ---- Calculate Transformed Values ----
	Vector3 worldGyro = Vec3_New(
		-Vec3_Dot(rawGyro, gravNorm),  // Yaw Alignment
		Vec3_Dot(rawGyro, pitchAxis), // Pitch Alignment
		rawGyro.z                     // Direct Roll Mapping
	);

	// ---- Return the Transformed Vector ----
	return worldGyro;
}