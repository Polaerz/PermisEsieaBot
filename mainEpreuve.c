#include "settings.h"
#include "input.h"
#include "button.h"
#include "fps.h"
#include "led.h"
#include "motor_controller.h"
#include "ultrasonic_sensor.h"
#include "tools.h"



void selectVitessse(float vitesse, int SpeedMode)
{
    //Speed +
    if (input.SpeedMoreDown)
    {
        if(Speedmode <4)
        {
            Speedmode++;
        }
    }
    //Speed -
    if (input.SpeedLessDown)
    {
        if(Speedmode >1)
        {
            Speedmode--;
        }
    }
    switch (Speedmode)
    {
        case 1:
            vitesse = 40.f;
        case 2:
            vitesse = 50.f;
        case 3:
            vitesse = 60.f;
        case 4:
            vitesse = 70.f;
    }
}

void selectMode(int modeDown)
{
    //Speed +
    if (input.modeDown)
    {
        modeDown++;
        modeDown%4;
    }
    switch (modeDown)
    {
        case 0:
            printf("0 - Mode Normal - (demi-tour + vitesse cible)");
        case 1:
            printf("1 - Mode Normal - freinage d'urgence");
        case 2:
            printf("2 - Mode Slalom - virage assiste");
        case 3:
            printf("3 - Mode Autonome");
    }
}




int main(int argc, char *argv[])
{
    //Pour la vitesse
    float speed = 0.f;
    int speedmode = 1;
    int mode = 1;


    // Recherche un game controller
    g_joystick = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (g_joystick == -1)
    {
        printf("Could not open joystick\n");
        return EXIT_FAILURE;
    }

    // Initialise PiGPIO
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        printf("Can't start gpio lib\n");
        assert(false);
        abort();
    }

    FPS fps = { 0 };
    FPS_init(&fps, pi);

    Input input = { 0 };
    Button button = { 0 };
    Button_init(&button, pi, GPIO_BUTTON);

    LED led = { 0 };
    LED_init(&led, pi, GPIO_LED);

    
    float kp = 4.0f;
    float ki = 2.0f;
    
    MotorController motorL = { 0 };
    MotorController motorR = { 0 };
    MotorController_init(&motorL, pi, GPIO_FORWARD_L, GPIO_BACKWARD_L, GPIO_MOTOR_CONTROL_L);
    MotorController_init(&motorR, pi, GPIO_FORWARD_R, GPIO_BACKWARD_R, GPIO_MOTOR_CONTROL_R);
    MotorController_setStartPower(&motorL, 110);
    MotorController_setStartPower(&motorR, 110);
    MotorController_setController(&motorL, kp, ki);
    MotorController_setController(&motorR, kp, ki);

    //LED_blink(&led, 3, 0.2f); //fait clignoter la led
    while (true)
    {
        //appel tous les trucs et check pour des callbacks
        FPS_update(&fps);
        Input_update(&input);
        Button_update(&button);
        LED_update(&led);
        MotorController_update(&motorL);
        MotorController_update(&motorR);

        selectVitessse(speed, speedmode);
        selectMode(mode);

        if (Button_isPressed(&button) || input.startPressed) //arrête le bot
            break;

        if (input.forwardDown) //Avant
        {
            MotorController_setBackward(&motorL, false);
            MotorController_setBackward(&motorR, false);

            float deltaV = input.leftAxisX * 15.f;

            MotorController_setTargetSpeed(&motorL, speed + deltaV);
            MotorController_setTargetSpeed(&motorR, speed - deltaV);
        }
        if (input.turnDown)  //Demi-tour avec bouton Y
        {
            MotorController_setBackward(&motorL, true);
            MotorController_setBackward(&motorR, false);

            //float deltaV = input.leftAxisX * 15.f;
            //speed = 30.f; //a ajouter si trop rapide quand demi tour
            MotorController_setTargetSpeed(&motorL, speed);
            MotorController_setTargetSpeed(&motorR, speed);
        }

        else
        {
            MotorController_setTargetSpeed(&motorL, 0.f);
            MotorController_setTargetSpeed(&motorR, 0.f);
        }
    }

    MotorController_quit(&motorL);
    MotorController_quit(&motorR);
    Button_quit(&button);
    LED_quit(&led);
    FPS_quit(&fps);

    pigpio_stop(pi);

    return EXIT_SUCCESS;
}

