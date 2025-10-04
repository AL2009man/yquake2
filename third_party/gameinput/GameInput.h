/*
 * GameInput C Wrapper - enables SDL3 GameInput detection in C projects
 */

#pragma once

#ifndef GAMEINPUT_C_WRAPPER_H
#define GAMEINPUT_C_WRAPPER_H

#ifdef __cplusplus
#define GAMEINPUT_WAS_CPP_DEFINED
#else
#define __cplusplus 1
#endif

#include "include/GameInput.h"

#ifndef GAMEINPUT_WAS_CPP_DEFINED
#undef __cplusplus
#endif
#undef GAMEINPUT_WAS_CPP_DEFINED

#endif /* GAMEINPUT_C_WRAPPER_H */