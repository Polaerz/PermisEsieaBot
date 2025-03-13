#include "settings.h"
#include "input.h"

#include "button.h"
#include "fps.h"
#include "led.h"
#include "motor_controller.h"
#include "ultrasonic_sensor.h"
#include "tools.h"

// TODO: décommenter le define correspondant à la partie à tester
#define OUR_MAIN
//#define MAIN_PROGRAM
//#define TEST_BUTTON
//#define TEST_LED
//#define TEST_MOTOR_CONTROLLER
//#define TEST_ULTRASONIC_SENSOR

void selectMode(int mode);

#ifdef OUR_MAIN
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

    int gear = 0;
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

    UltrasonicSensor sensorL = { 0 };
    UltrasonicSensor_init(&sensorL, pi, GPIO_TRIG_L, GPIO_ECHO_L);

    UltrasonicSensor sensorR = { 0 };
    UltrasonicSensor_init(&sensorR, pi, GPIO_TRIG_R, GPIO_ECHO_R);


    int mode = 0;
    printf("Mode marche arrière\n");

    LED_blink(&led, 3, 0.2f);
    while (true)
    {
        FPS_update(&fps);
        Input_update(&input);
        Button_update(&button);
        LED_update(&led);
        MotorController_update(&motorL);
        MotorController_update(&motorR);
        UltrasonicSensor_update(&sensorL);
        UltrasonicSensor_update(&sensorR);

        if (Button_isPressed(&button) || input.startPressed)
            break;

        
        // Mode
        if (input.modePressed)
            selectMode(mode);

        switch (mode)
        {
        case 1:
            //modeMarcheArriere();
            if (input.superButtonPressed) // bouton Y
            {
                //prinf("launch c1");
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, true);

                speed = 50.f;
                //float deltaV = input.leftAxisX * 15.f;

                MotorController_setTargetSpeed(&motorL, speed);
                MotorController_setTargetSpeed(&motorR, speed);
            }else if(input.forwardDown)
            {
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, false);

                speed = 50.f;
                float deltaV = input.leftAxisX * 15.f;

                MotorController_setTargetSpeed(&motorL, speed + deltaV);
                MotorController_setTargetSpeed(&motorR, speed - deltaV);
                
            }else if(input.backwardDown)
            {
                MotorController_setBackward(&motorL, true);
                MotorController_setBackward(&motorR, true);

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
            break;
            
        case 2:
            //modeSlalom();
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
            break;    
        case 3:
            //modeFreinageUrgence();
            if (input.forwardDown && UlrasonicSensor_getDistance(&sensorL) > 20.f)
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
                for(i= 0; i<4;i++){
                    LED_blink(&led, 5, 0.4);
                }
            }
                //DEMI-TOUR SUR PLACE
                //REPART EN MARCHE ARRIERE
            }
            break;
            
        case 4:
            //modeVitesseCible();
            break;

        case 5:
            //modeCircuitAutonome();
            break;
            
        default:
            printf("error\n");
            break;
        }

        //Speed +
        if (input.speedLvlPlus)
        {
            if(gear <4)
            {
                gear++;
            }
        }
        //Speed -
        if (input.speedLvlMinus)
        {
            if(gear >1)
            {
                gear--;
            }
        }
        switch (gear)
        {
            case 1:
                speed = 40.f;
            case 2:
                speed = 50.f;
            case 3:
                speed = 60.f;
            case 4:
                speed = 70.f;
        }



    MotorController_quit(&motorL);
    MotorController_quit(&motorR);
    Button_quit(&button);
    LED_quit(&led);
    FPS_quit(&fps);
    UltrasonicSensor_quit(&sensorL);
    UltrasonicSensor_quit(&sensorR);

    pigpio_stop(pi);

    return EXIT_SUCCESS;
    }
}



void selectMode(int mode)
{
    //Speed +
    if (input.mode)
    {
        mode++;
        mode%5;
    }
    switch (mode)
    {

        case 0:
            printf("Mode Slalom\n");
            break;           
        case 1:
            printf("Mode Freinage d'urgence\n");
            break;    
        
        case 2:
            printf("Mode vitesse cible\n");
            break;
        
        case 3:
            printf("Mode autonome\n");
            break;

        case 4:
            printf("Mode marche arrière\n");
            break;
        
        default:
            printf("error\n");
            break;
    }
}









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

    printf("L cbCount = %d\n", motorL.m_cbCount);
    printf("R cbCount = %d\n", motorR.m_cbCount);
    
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
