#pragma once

#include <inttypes.h>

void YUV420ToRGB(int width, int height, const uint8_t* in, uint8_t* out);