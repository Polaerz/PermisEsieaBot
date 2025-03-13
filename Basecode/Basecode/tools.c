
#include "tools.h"

int Int_clampAB(int value, int a, int b)
{
    value = value < a ? a : value;
    value = value > b ? b : value;
    return value;
}

float Float_clamp01(float value)
{
    return fminf(fmaxf(value, 0.f), 1.f);
}

float Float_clampAB(float value, float a, float b)
{
    return fminf(fmaxf(value, a), b);
}

float Float_lerp(float value0, float value1, float t)
{
    t = Float_clamp01(t);
    return (1.f - t) * value0 + t * value1;
}

float Float_invLerp(float value0, float value1, float value)
{
    return Float_clamp01((value - value0) / (value1 - value0));
}

float Float_remap(float src0, float src1, float dst0, float dst1, float value)
{
    float t = Float_invLerp(src0, src1, value);
    return Float_lerp(dst0, dst1, t);
}
