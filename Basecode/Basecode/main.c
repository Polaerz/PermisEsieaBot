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

    int gear = 1;
    float speed = 30.f;
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

        if (Button_isPressed(&button) || input.startPressed){
            break;
        }

        
        // Mode
        if (input.modePressed){
            switch (mode){
            case 0:
                printf("Mode Slalom\n");
                mode++;
                break;           
            case 1:
                printf("Mode Freinage d'urgence\n");
                mode++;
                break;    

            case 2:
                printf("Mode vitesse cible\n");
                mode++;
                break;

            case 3:
                printf("Mode autonome\n");
                mode++;
                break;

            case 4:
                printf("Mode marche arrière\n");
                mode = 0;
                break;

            default:
                printf("error\n");
                break;
            }
        }


        switch (mode)
        {
        case 0:
            //modeMarcheArriere();
            if (input.superButtonPressed) // bouton Y
            {
                //prinf("launch c1");
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, true);

                //speed = 50.f;
                //float deltaV = input.leftAxisX * 15.f;

                MotorController_setTargetSpeed(&motorL, speed);
                MotorController_setTargetSpeed(&motorR, speed);
            }else if(input.forwardDown)
            {
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, false);

                //speed = 50.f;
                float deltaV = input.leftAxisX * 15.f;

                MotorController_setTargetSpeed(&motorL, speed + deltaV);
                MotorController_setTargetSpeed(&motorR, speed - deltaV);
                
            }else if(input.backwardDown)
            {
                MotorController_setBackward(&motorL, true);
                MotorController_setBackward(&motorR, true);

                //speed = 50.f;
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
            
        case 1:
            //modeSlalom();
            if (input.forwardDown)
            {
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, false);

                //speed = 50.f;
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
            //modeFreinageUrgence();
            if (input.forwardDown && UlrasonicSensor_getDistance(&sensorL) > 20.f)
            {
                MotorController_setBackward(&motorL, false);
                MotorController_setBackward(&motorR, false);

                //speed = 50.f;
                float deltaV = input.leftAxisX * 15.f;

                MotorController_setTargetSpeed(&motorL, speed + deltaV);
                MotorController_setTargetSpeed(&motorR, speed - deltaV);
            }
            else
            {
                MotorController_setTargetSpeed(&motorL, 0.f);
                MotorController_setTargetSpeed(&motorR, 0.f);
                for(int i= 0; i<4;i++){
                    LED_blink(&led, 5, 0.4);
                }
            }
                //DEMI-TOUR SUR PLACE
                //REPART EN MARCHE ARRIERE
            
            break;
            
        case 3:
            //modeVitesseCible();
            break;

        case 4:
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
                printf("Gear %d\n", gear);
            }
        }
        //Speed -
        if (input.speedLvlMinus)
        {
            if(gear >1)
            {
                gear--;
                printf("Gear %d\n", gear);
            }
        }
        switch (gear)
        {
            case 0:
                break;
            case 1:
                speed = 30.f;
                break;
            case 2:
                speed = 50.f;
                break;
            case 3:
                speed = 60.f;
                break;
            case 4:
                speed = 70.f;
                break;
            default :
                printf("Error\n");
                break;
        }

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
#endif