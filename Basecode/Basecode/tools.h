#pragma once

#include "settings.h"

int Int_clampAB(int value, int a, int b);

float Float_clamp01(float value);
float Float_clampAB(float value, float a, float b);
float Float_lerp(float value0, float value1, float t);
float Float_invLerp(float from, float to, float value);
float Float_remap(float src0, float src1, float dst0, float dst1, float value);
