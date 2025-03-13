#include "testInput.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    // Recherche un game controller

    // Initialise PiGPIO


    Input input = { 0 };
    int gear = 0;

    float speed = 0.f;
    float kp = 4.0f;
    float ki = 2.0f;


    // Boucle principale
    for (int i = 0; i < 10; i++) {
        // Reculer
        if (input.forwardDown)
        {
            printf("Motor L : Backward = false\n");
            printf("Motor R : Backward = false\n");

            speed = 50.f;
            float deltaV = input.leftAxisX * 15.f;

            printf("Motor L : Speed = %lf\n", speed + deltaV);
            printf("Motor R : Speed = %lf\n", speed - deltaV);
        }
        // else
        // {
        //     printf("Motor L : Speed = 0.f\n");
        //     printf("Motor R : Speed = 0;f\n");
        // }

        // Demi tour
        if (input.forwardDown)
        {
            printf("Motor L : Backward = true\n");
            printf("Motor R : Backward = false\n");

            speed = 50.f;
            float deltaV = input.leftAxisX * 15.f;

            printf("Motor L : Speed = %lf\n", speed + deltaV);
            printf("Motor R : Speed = %lf\n", speed - deltaV);
        }
        // else
        // {
        //     printf("Motor L : Speed = 0.f\n");
        //     printf("Motor R : Speed = 0;f\n");
        // }

        //Speed +
        if (input.speedLvlPlus)
        {
            if (gear < 4) { gear += 1; }

            switch (gear) {
                case 1:
                    speed = 40.f;
                case 2:
                    speed = 50.f;
                case 3:
                    speed = 60.f;
                case 4:
                    speed = 70.f;
            }

            float deltaV = input.leftAxisX * 15.f;

            printf("Motor L : Speed = %lf\n", speed + deltaV);
            printf("Motor R : Speed = %lf\n", speed - deltaV);
        }

        if (input.speedLvlMinus)
        {
            if (gear > 1) { gear -= 1; }

            switch (gear) {
                case 1:
                    speed = 40.f;
                case 2:
                    speed = 50.f;
                case 3:
                    speed = 60.f;
            }

            float deltaV = input.leftAxisX * 15.f;

            printf("Motor L : Speed = %lf\n", speed + deltaV);
            printf("Motor R : Speed = %lf\n", speed - deltaV);
        }
    }

    return EXIT_SUCCESS;
}