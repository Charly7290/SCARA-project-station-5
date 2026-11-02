#ifndef CONTROL_H
#define CONTROL_H

// Controladores disponibles
typedef enum {
    CONTROL_PID,
    CONTROL_RST_POSITIONAL,
    CONTROL_RST_INCREMENTAL,
    CONTROL_RST_MASKED
} ControlMode;
typedef struct {
    float u1;
    float u2;
} ControlOutput;
//Tolerancia para usarla en todos los archivos
extern const float tolerance;

// Funciones del controlador
ControlOutput salidas(float ref_M1, float enc_M1,
                      float ref_M2, float enc_M2);

void resetPID(void);

#endif
