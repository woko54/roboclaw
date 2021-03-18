/*
    Some code for testing
*/

#include <cstdio>
#include <unistd.h>

#include "Roboclaw.h"

int main()
{
    CRoboclaw* roboclaw = new CRoboclaw(0);

    roboclaw->setMicrostep(8);
    roboclaw->setSpeed("3000");
    roboclaw->setAcceleration("2000");
    roboclaw->setDeceleration("3000");

    roboclaw->setTarget("-50000");      // Move to position
    printf("Start moving to -50000\n");
    roboclaw->startMove(0);
    roboclaw->waitCompletion();
    printf("Position: %ld reached\n", roboclaw->getPosition());

    roboclaw->setTarget("0");           // Move back to home-position
    printf("Start moving to 0\n");
    roboclaw->startMove(0);
    roboclaw->waitCompletion();
    printf("Position: %ld reached\n", roboclaw->getPosition());

    printf("Waiting som time\n");
    sleep(5);                           // Wait some time

    roboclaw->setTarget("-80000");     // Move to position
    printf("Start moving to -80000\n");
    roboclaw->startMove(0);
    roboclaw->waitCompletion();
    printf("Position: %ld reached\n", roboclaw->getPosition());

    roboclaw->setDeceleration("300");   // Set a slower deceleration ramp
    roboclaw->setTarget("-200000");     // Move to position
    printf("Start moving to -200000\n");
    roboclaw->startMove(0);
    roboclaw->waitCompletion();
    printf("Position: %ld reached\n", roboclaw->getPosition());

    return 0;
}