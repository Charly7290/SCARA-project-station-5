#include "control.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

ControlMode CONTROL_MODE = CONTROL_RST_INCREMENTAL; // Default control mode
//Opciones:
// CONTROL_PID
// CONTROL_RST_POSITIONAL
// CONTROL_RST_INCREMENTAL
// CONTROL_RST_MASKED

extern const float tolerance;
extern const float pulley_ratio_M1;
extern const float pulley_ratio_M2;

//// PID ////

// Ganancias PID del motor M1
float Kp_M1 = 3.347831f;
float Ki_M1 = 0.0f;
float Kd_M1 = 0.0f;
// Ganancias PID del motor M2
float Kp_M2 = 3.335833f;
float Ki_M2 = 0.0f;
float Kd_M2 = 0.0f;
//Tiempo de muestreo (en segundos)
float Ts = 0.02f;


///// RST POSICIONAL ////
// Ganancias RST del motor M1
float Rp1_M1 = -0.1299948270f;
float Sp0_M1 = -165.5428786966f;
float Sp1_M1 = 165.7996000000f;
float Sp2_M1 = 0.0f;
float Tp0_M1 = 0.2567005798f;
// Ganancias RST del motor M2
float Rp1_M2 = -0.1360196658f;
float Sp0_M2 = -164.1919757090f;
float Sp1_M2 = 164.4380427125f;
float Sp2_M2 = 0.0f;
float Tp0_M2 =  0.2460670034f;



// RST INCREMENTAL ////
// RST INCREMENTAL M1
float Ri1_M1 = 0.4124916761f;
float Si0_M1 = 552.4751648599f;
float Si1_M1 = -1078.3236020223f;
float Si2_M1 = 526.1051377423f;
float Ti0_M1 = 0.2567005798f;
// RST INCREMENTAL M2
float Ri1_M2 = 0.4086586913f;
float Si0_M2 = 519.3165110861f;
float Si1_M2 = -1013.1095593375f;
float Si2_M2 =  494.0391152549f ;
float Ti0_M2 = 0.2460670034f;



// RST MASKED ////
// RST ENMASCARADO COMO PID INCREMENTAL M1
float q0_M1 = 0.0f;
float q1_M1 = 0.0f;
float q2_M1 = 0.0f;
// RST ENMASCARADO COMO PID INCREMENTAL M2
float q0_M2 = 0.0f;
float q1_M2 = 0.0f;
float q2_M2 = 0.0f;

//////////////////////////////////////////////////////////////////////



// Obtener errores
float integral_M1 = 0.0f;
float previous_error_M1 = 0.0f;
float integral_M2 = 0.0f;
float previous_error_M2 = 0.0f;
#define Max_antiwindup 20.0f

//Variables para que el PID se ejecute solo en el tiempo de muestreo
static uint64_t last_pid_us = 0;
static bool first_sample = true;
static ControlOutput last_output = {0.0f, 0.0f};

//Para que empiece suavementeee
static bool first_M1 = true;
static bool first_M2 = true;

// Historial para los controladores RST
typedef struct {
    float y1;       // Posicion anterior
    float y2;       // Posicion de hace dos muestras
    float e1;       // Error anterior
    float e2;       // Error de hace dos muestras
    float u_prev;   // PWM anterior en porcentaje
    float du_prev;  // Incremento anterior del PWM
    bool initialized;
} RSTState;
static RSTState rst_M1 = {0};
static RSTState rst_M2 = {0};





//////////////////////////PID//////////////////////////

//Función PID para el motor M1
static float ControllerM1(float anguloM1_ref, float anguloM1_enc) {

    float errorangleM1 = anguloM1_ref - anguloM1_enc;
    if (fabsf(errorangleM1) <= tolerance* pulley_ratio_M1 * (float)M_PI / 180.0f){
        integral_M1 = 0.0f;
        previous_error_M1 = errorangleM1;
        first_M1 = true;
        return 0.0f;
    }
    // Aquí se calcula el PID utilizando Kp, Ki y Kd
    float derivative_M1 = 0.0f;
    if (!first_M1) {
        derivative_M1 = (errorangleM1 - previous_error_M1) / Ts;
    }
    first_M1 = false;
    float new_integral_M1 = integral_M1 + errorangleM1 * Ts;
    float u1 = Kp_M1 * errorangleM1 + Ki_M1 * new_integral_M1 + Kd_M1 * derivative_M1;


    // Antiwindup: aceptar la integral solo cuando no agrava la saturación
    if (!((u1 > Max_antiwindup && errorangleM1 > 0.0f) || (u1 < -Max_antiwindup && errorangleM1 < 0.0f))) {
        integral_M1 = new_integral_M1;
    }
    u1 = Kp_M1 * errorangleM1 + Ki_M1 * integral_M1 + Kd_M1 * derivative_M1;
    if (u1 > Max_antiwindup) {
        u1 = Max_antiwindup;
    } else if (u1 < -Max_antiwindup) {
        u1 = -Max_antiwindup;
    }

    previous_error_M1 = errorangleM1;
    return u1;
}

//Función PID para el motor M2
static float ControllerM2(float anguloM2_ref, float anguloM2_enc) {

    float errorangleM2 = anguloM2_ref - anguloM2_enc;
    if (fabsf(errorangleM2) <= tolerance* pulley_ratio_M2* (float)M_PI / 180.0f) {
        integral_M2 = 0.0f;
        previous_error_M2 = errorangleM2;
        first_M2 = true;
        return 0.0f;
    }

    // Aquí se calcula el PID utilizando Kp, Ki y Kd
    float derivative_M2 = 0.0f;
    if (!first_M2) {
        derivative_M2 = (errorangleM2 - previous_error_M2) / Ts;
    }
    first_M2 = false;
    float new_integral_M2 = integral_M2 + errorangleM2 * Ts;
    float u2 = Kp_M2 * errorangleM2 + Ki_M2 * new_integral_M2 + Kd_M2 * derivative_M2;


    // Antiwindup: aceptar la integral solo cuando no agrava la saturación
    if (!((u2 > Max_antiwindup && errorangleM2 > 0.0f) || (u2 < -Max_antiwindup && errorangleM2 < 0.0f))) {
        integral_M2 = new_integral_M2;
    }
    u2 = Kp_M2 * errorangleM2 + Ki_M2 * integral_M2 + Kd_M2 * derivative_M2;
    if (u2 > Max_antiwindup) {
        u2 = Max_antiwindup;
    } else if (u2 < -Max_antiwindup) {
        u2 = -Max_antiwindup;
    }

    previous_error_M2 = errorangleM2;
    return u2;
}

////////// RST //////////

// RST POSICIONAL R(z^-1)U = T(z^-1)Ref - S(z^-1)Y
static float RST_Posicional(
    float ref, float y, RSTState *state,
    float R1, float S0, float S1, float S2, float T0) {
    float u = T0 * ref - S0 * y - S1 * state->y1 - S2 * state->y2 - R1 * state->u_prev;
    return u;
}


// RST INCREMENTAL R(z^-1)Delta U = T(z^-1)Ref - S(z^-1)Y
static float RST_Incremental(
    float ref, float y, RSTState *state,
    float R1, float S0, float S1, float S2, float T0) {

    // T constante: T(1) = S(1)
    float du = T0 * ref - S0 * y - S1 * state->y1 - S2 * state->y2- R1 * state->du_prev;
    return state->u_prev + du;
}


// RST ENMASCARADO COMO PID INCREMENTAL Delta U = q0*e[k] + q1*e[k-1] + q2*e[k-2]
static float RST_Masked(
    float error, RSTState *state,
    float q0, float q1, float q2) {

    float du = q0 * error + q1 * state->e1 + q2 * state->e2;
    return state->u_prev + du;
}


// Ejecutar el RST seleccionado
static float ControllerRST(
    float ref, float y, RSTState *state,
    float Rp1, float Sp0, float Sp1, float Sp2, float Tp0,
    float Ri1, float Si0, float Si1, float Si2, float Ti0,
    float q0, float q1, float q2, float pulley_ratio) {

    float error = ref - y;

    // Detener el motor dentro de la tolerancia
    if (fabsf(error) <= tolerance* pulley_ratio*(float)M_PI / 180.0f) {
        *state = (RSTState){0};
        return 0.0f;
    }

    // Inicializar historial al comenzar un movimiento
    if (!state->initialized) {
        state->y1 = y;
        state->y2 = y;
        state->e1 = 0.0f;
        state->e2 = 0.0f;
        state->u_prev = 0.0f;
        state->du_prev = 0.0f;
        state->initialized = true;
    }

    float u = 0.0f;

    switch (CONTROL_MODE) {

        case CONTROL_RST_POSITIONAL:
            u = RST_Posicional(
                ref, y, state,
                Rp1, Sp0, Sp1, Sp2, Tp0);
            break;

        case CONTROL_RST_INCREMENTAL:
            u = RST_Incremental(
                ref, y, state,
                Ri1, Si0, Si1, Si2, Ti0);
            break;

        case CONTROL_RST_MASKED:
            u = RST_Masked(
                error, state,
                q0, q1, q2);
            break;

        default:
            return 0.0f;
    }

    // Saturacion de la salida: -17.6 % a +17.6 %
    if (!isfinite(u)) {
        u = 0.0f;
    }

    u = fmaxf(-Max_antiwindup,
              fminf(Max_antiwindup, u));

    // Guardar incremento realmente aplicado
    state->du_prev = u - state->u_prev;
    state->u_prev = u;

    // Actualizar historial de posicion y error
    state->y2 = state->y1;
    state->y1 = y;

    state->e2 = state->e1;
    state->e1 = error;

    return u;
}





ControlOutput salidas(float ref_M1, float enc_M1,
                      float ref_M2, float enc_M2) {
    if (Ts <= 0.0f) {
        last_output.u1 = 0.0f;
        last_output.u2 = 0.0f;
        return last_output;
    }

    uint64_t now = time_us_64();
    uint64_t period_us = (uint64_t)(Ts * 1000000.0f);

    if (first_sample || (now - last_pid_us) >= period_us) {
        last_pid_us = now;
        first_sample = false;
        if (CONTROL_MODE == CONTROL_PID) {
            // PID tradicional
            last_output.u1 = ControllerM1(ref_M1, enc_M1);
            last_output.u2 = ControllerM2(ref_M2, enc_M2);
        } else {
            // RST seleccionado: M1
            last_output.u1 = ControllerRST(
                ref_M1, enc_M1, &rst_M1,
                Rp1_M1, Sp0_M1, Sp1_M1, Sp2_M1, Tp0_M1,
                Ri1_M1, Si0_M1, Si1_M1, Si2_M1, Ti0_M1,
                q0_M1, q1_M1, q2_M1, pulley_ratio_M1
            );
            // RST seleccionado: M2
            last_output.u2 = ControllerRST(
                ref_M2, enc_M2, &rst_M2,
                Rp1_M2, Sp0_M2, Sp1_M2, Sp2_M2, Tp0_M2,
                Ri1_M2, Si0_M2, Si1_M2, Si2_M2, Ti0_M2,
                q0_M2, q1_M2, q2_M2, pulley_ratio_M2
            );
        }
    }

    return last_output;
}



//Reseteo despues de cada movimiento
void resetPID(void) {
    // PID
    integral_M1 = 0.0f;
    previous_error_M1 = 0.0f;
    integral_M2 = 0.0f;
    previous_error_M2 = 0.0f;

    first_M1 = true;
    first_M2 = true;

    // RST
    rst_M1 = (RSTState){0};
    rst_M2 = (RSTState){0};

    // Muestreo
    last_pid_us = 0;
    first_sample = true;

    last_output.u1 = 0.0f;
    last_output.u2 = 0.0f;
}