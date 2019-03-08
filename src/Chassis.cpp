#include "Chassis.h"
#include <stdio.h>


Chassis::Chassis(int myPort) {
    printf("Initializing Chassis\n");
    printf("Chassis Drive Motor Initialized with port number [%d]\n", myPort);
    port = myPort;
    input = 0.0;
    output = 0.0;
}

Chassis::~Chassis(){
    port = 0;
    output = 0.0;
    input = 0.0;
}