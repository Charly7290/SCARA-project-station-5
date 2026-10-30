#ifndef CONTROL_H
#define CONTROL_H

typedef struct {
    float u1;
    float u2;
} ControlOutput;

ControlOutput salidas(float ref_M1, float enc_M1,
                      float ref_M2, float enc_M2);

void resetPID(void);

#endif