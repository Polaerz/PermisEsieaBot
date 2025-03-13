#include "settings.h"
#include "input.h"

#include "button.h"
#include "fps.h"
#include "led.h"
#include "motor_controller.h"
#include "ultrasonic_sensor.h"
#include "tools.h"


// TODO: décommenter le define correspondant à la partie à tester
#define MAIN_EPREUVE
//#define MAIN_PROGRAM
//#define TEST_BUTTON
//#define TEST_LED
//#define TEST_MOTOR_CONTROLLER
//#define TEST_ULTRASONIC_SENSOR

#ifdef MAIN_EPREUVE
// CODE PERSO ---------------------------------------- CODE PERSO --------------------
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
    
    //Motor
    MotorController motorL = { 0 };
    MotorController motorR = { 0 };
    MotorController_init(&motorL, pi, GPIO_FORWARD_L, GPIO_BACKWARD_L, GPIO_MOTOR_CONTROL_L);
    MotorController_init(&motorR, pi, GPIO_FORWARD_R, GPIO_BACKWARD_R, GPIO_MOTOR_CONTROL_R);
    MotorController_setStartPower(&motorL, 110);
    MotorController_setStartPower(&motorR, 110);
    MotorController_setController(&motorL, kp, ki);
    MotorController_setController(&motorR, kp, ki);

    //Ultrason
    UltrasonicSensor sensorL = { 0 };
    UltrasonicSensor_init(&sensorL, pi, GPIO_TRIG_L, GPIO_ECHO_L);
    UltrasonicSensor sensorR = { 0 };
    UltrasonicSensor_init(&sensorR, pi, GPIO_TRIG_R, GPIO_ECHO_R);

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
        UltrasonicSensor_update(&sensorL);
        UltrasonicSensor_update(&sensorR);

        selectVitessse(speed, speedmode);
        selectMode(mode);

        
        if (Button_isPressed(&button) || input.startPressed) //arrête le bot
            break;

        switch(mode)
        {
            case 0: //Mode Normal - Demi tour - Vitesse Cible

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
            case 1: //Mode Normal - Arret d urgence - Demi tour - Vitesse Cible

                if (input.forwardDown && UlrasonicSensor_getDistance(&sensorL) > 20.f) //Avant
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
                    speed = 40.f; //a ajouter si trop rapide quand demi tour
                    MotorController_setTargetSpeed(&motorL, speed);
                    MotorController_setTargetSpeed(&motorR, speed);
                }
                else
                {
                    MotorController_setTargetSpeed(&motorL, 0.f);
                    MotorController_setTargetSpeed(&motorR, 0.f);
                    LED_blink(&led,5,4);
                }

            case 2: //Mode Slalom - Virrage assiste - Vitesse Cible


            case 3: //Mode Autonome
            
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

// CODE PERSO ---------------------------------------- CODE PERSO --------------------

#elif defined MAIN_PROGRAM
//------------------------------------------------------------------------------
// Programme principal

/// @brief Programme principal.
int main(int argc, char *argv[])
{
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

    float speed = 0.f;
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

    LED_blink(&led, 3, 0.2f);
    while (true)
    {
        FPS_update(&fps);
        Input_update(&input);
        Button_update(&button);
        LED_update(&led);
        MotorController_update(&motorL);
        MotorController_update(&motorR);

        if (Button_isPressed(&button) || input.startPressed)
            break;

        if (input.forwardDown)
        {
            MotorController_setBackward(&motorL, false);
            MotorController_setBackward(&motorR, false);

            speed = 50.f;
            float deltaV = input.leftAxisX * 15.f;

            MotorController_setTargetSpeed(&motorL, speed + deltaV);
            MotorController_setTargetSpeed(&motorR, speed - deltaV);
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
#elif defined TEST_BUTTON
//------------------------------------------------------------------------------
// Bouton

/// @brief Programme de test du bouton intégré sur l'add-on board.
/// L'utilisateur doit appuyer 3 fois sur le bouton pour terminer l'exécution.
int main(int argc, char *argv[])
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        printf("ERROR pigpio_start()\n");
        assert(false); abort();
    }

    Button button;
    Button_init(&button, pi, GPIO_BUTTON);
    int count = 0;

    while (true)
    {
        Button_update(&button);
        if (Button_isPressed(&button))
        {
            count++;
            printf("Pressed %d\n", count);
        }
		
		if (count >= 3) break;
    }
    
    Button_quit(&button);
    pigpio_stop(pi);
    return EXIT_SUCCESS;
}
#elif defined TEST_LED
//------------------------------------------------------------------------------
// LED

/// @brief Programme de test de la LED intégrée sur l'add-on board.
/// La LED clignotte 5 fois (une seconde par cycle) puis le programme se termine.
/// Il est possible d'arrêter l'exécution avant en appuyant sur le bouton.
int main(int argc, char *argv[])
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        printf("ERROR pigpio_start()\n");
        assert(false); abort();
    }

    Button button;
    Button_init(&button, pi, GPIO_BUTTON);

    LED led = { 0 };
    LED_init(&led, pi, GPIO_LED);

    LED_blink(&led, 5, 1.f);
    
    while (true)
    {
        Button_update(&button);
        LED_update(&led);
		
		if ((LED_isBlinking(&led) == false) || Button_isPressed(&button)) break;
    }
    
    Button_quit(&button);
    LED_quit(&led);
    pigpio_stop(pi);
    return EXIT_SUCCESS;
}
#elif defined TEST_MOTOR_CONTROLLER
//------------------------------------------------------------------------------
// Motor controller

/// @brief Programme de test des moteurs.
/// Il faut appuyer une fois sur le bouton pour démarrer les moteurs.
/// La vitesse cible est alors de 40 pas par seconde.
/// Elle augmente de 10 pas par seconde à chaque fois que le bouton est pressé
/// jusqu'à 60.
int main(int argc, char *argv[])
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        printf("ERROR pigpio_start()\n");
        assert(false); abort();
    }

    Button button;
    Button_init(&button, pi, GPIO_BUTTON);
    
    while (true)
    {
        Button_update(&button);
        if (Button_isPressed(&button)) break;
    }

    MotorController motorL = { 0 };
    MotorController motorR = { 0 };
    MotorController_init(&motorL, pi, GPIO_FORWARD_L, GPIO_BACKWARD_L, GPIO_MOTOR_CONTROL_L);
    MotorController_init(&motorR, pi, GPIO_FORWARD_R, GPIO_BACKWARD_R, GPIO_MOTOR_CONTROL_R);
    MotorController_setStartPower(&motorL, 110);
    MotorController_setStartPower(&motorR, 110);
    MotorController_setController(&motorL, 4.f, 2.f);
    MotorController_setController(&motorR, 4.f, 2.f);
    
    float speed = 40.f;
    printf("speed = %2.1f\n", speed);
    while (true)
    {
        Button_update(&button);
        MotorController_update(&motorL);
        MotorController_update(&motorR);
        
        if (Button_isPressed(&button))
        {
            speed += 10.f;
            printf("speed = %2.1f\n", speed);
        }
        
        MotorController_setTargetSpeed(&motorL, speed);
        MotorController_setTargetSpeed(&motorR, speed);
		
		if (speed > 61.f) break;
    }
    
    Button_quit(&button);
    MotorController_quit(&motorL);
    MotorController_quit(&motorR);
    pigpio_stop(pi);
    return EXIT_SUCCESS;
}
#elif defined TEST_ULTRASONIC_SENSOR
//------------------------------------------------------------------------------
// Capteur de distance

/// @brief Programme de test d'un capteur de distance.
/// Chaque nouvelle distance mesurée par le capteur est affichée si elle est
/// inférieure à 40 cm.
/// Il faut appuyer sur le bouton pour terminer l'exécution.
int main(int argc, char *argv[])
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0)
    {
        printf("ERROR pigpio_start()\n");
        assert(false); abort();
    }

    Button button;
    Button_init(&button, pi, GPIO_BUTTON);

    UltrasonicSensor sensor = { 0 };
    UltrasonicSensor_init(&sensor, pi, GPIO_TRIG_L, GPIO_ECHO_L);
    
    while (true)
    {
        Button_update(&button);
        UltrasonicSensor_update(&sensor);

        if (Button_isPressed(&button)) break;
            
        float distance = UlrasonicSensor_getDistance(&sensor);
        if (distance <= 40.f && UltrasonicSensor_hasNewDistance(&sensor))
        {
            printf("distance = %.1f\n", distance);
        }
    }
    
    UltrasonicSensor_quit(&sensor);
    Button_quit(&button);
    pigpio_stop(pi);
    return EXIT_SUCCESS;
}
#endif
