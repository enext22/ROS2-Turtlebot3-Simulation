#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 0.1

struct PID
{
    float Kp;
    float Ki;
    float Kd;

    float T;

    float integral;
    float err_prev;
    float command_prev;
};

float PID_Step(struct PID *pid, float measurement, float setpoint)
{
    float err;
    float command;
    float derivative;
    float proportional;

    err = setpoint - measurement;
    
    proportional = (pid->Kp)*err;
    pid->integral += (pid->Ki)*err*(pid->T);

    derivative = (pid->Kd)*(err - pid->err_prev)/(pid->T);
    pid->err_prev = err;
    
    command = proportional + pid->integral + derivative;
    pid->command_prev = command;

    return command;
}