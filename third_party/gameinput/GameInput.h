/*
 * GameInput C Compatibility Wrapper
 * 
 * This header tricks the original GameInput.h into thinking we're compiling C++
 * so SDL3 can detect GameInput support, while keeping our project as C.
 * 
 * We temporarily define __cplusplus to bypass the C++ requirement check,
 * then include minimal definitions that SDL3 needs for detection.
 */

#pragma once

#ifndef GAMEINPUT_C_WRAPPER_H
#define GAMEINPUT_C_WRAPPER_H

// Save the current state of __cplusplus
#ifdef __cplusplus
#define GAMEINPUT_WAS_CPP_DEFINED
#else
// Temporarily define __cplusplus to bypass the GameInput.h C++ check
#define __cplusplus 1
#endif

// Include the real GameInput.h now that __cplusplus is defined
#include "include/GameInput.h"

// Restore the original __cplusplus state
#ifndef GAMEINPUT_WAS_CPP_DEFINED
#undef __cplusplus
#endif
#undef GAMEINPUT_WAS_CPP_DEFINED

#endif /* GAMEINPUT_C_WRAPPER_H */