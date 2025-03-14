#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <unistd.h>
#include <linux/joystick.h>

extern int g_joystick;

typedef struct Input
{
    float leftAxisX;
    float leftAxisY;
    float rightAxisX;
    float rightAxisY;
    bool startPressed;
    bool forwardDown;
    bool backwardDown;
    bool uTurn;
    bool speedLvlPlus;
    bool speedLvlMinus;
    
} Input;

void Input_update(Input *self);
void Input_updateControllerAxisMotion(Input *self, int axis, short value);
void Input_updateControllerButtonDown(Input *self, int button);
void Input_updateControllerButtonUp(Input *self, int button);
