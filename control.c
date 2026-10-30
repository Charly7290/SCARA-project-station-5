#include "control.h"       // Declaración de funciones del controlador
#include "revolute.h"      // Acceso a encoder_count[]
#include "pico/stdlib.h"   // Función time_us_64()
#include <math.h>          // fabsf() y otras funciones matemáticas
extern const float tolerance;

/// VALORES A INGRESAR PARA EL CONTROLADOR PID DE LOS MOTORES M1 Y M2 ///

// Ganancias PID del motor M1
float Kp_M1 = 0.0f;
float Ki_M1 = 0.0f;
float Kd_M1 = 0.0f;

// Ganancias PID del motor M2
float Kp_M2 = 0.0f;
float Ki_M2 = 0.0f;
float Kd_M2 = 0.0f;

//Tiempo de muestreo (en segundos)
float Ts = 0.00f;

//////////////////////////////////////////////////////////////////////



// Obtener errores
float integral_M1 = 0.0f;
float previous_error_M1 = 0.0f;
float integral_M2 = 0.0f;
float previous_error_M2 = 0.0f;
#define Max_antiwindup 17.6f

//Variables para que el PID se ejecute solo en el tiempo de muestreo
static uint64_t last_pid_us = 0;
static bool first_sample = true;
static ControlOutput last_output = {0.0f, 0.0f};

//Para que empiece suavementeee
static bool first_M1 = true;
static bool first_M2 = true;



//Función PID para el motor M1
static float ControllerM1(float anguloM1_ref, float anguloM1_enc) {

    float errorangleM1 = anguloM1_ref - anguloM1_enc;
    if (fabsf(errorangleM1) <= tolerance) {
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
    if (fabsf(errorangleM2) <= tolerance) {
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

ControlOutput salidas(float ref_M1, float enc_M1, float ref_M2, float enc_M2) {
    uint64_t now = time_us_64();
    uint64_t period_us = (uint64_t)(Ts * 1000000.0f);

    if (first_sample || (now - last_pid_us) >= period_us) {

        last_pid_us = now;
        first_sample = false;

        last_output.u1 = ControllerM1(ref_M1, enc_M1);
        last_output.u2 = ControllerM2(ref_M2, enc_M2);
    }

    return last_output;
}


//Reseteo despues de cada movimiento
void resetPID(void) {
    integral_M1 = 0.0f;
    previous_error_M1 = 0.0f;

    integral_M2 = 0.0f;
    previous_error_M2 = 0.0f;

    last_pid_us = 0;
    first_sample = true;

    last_output.u1 = 0.0f;
    last_output.u2 = 0.0f;

    first_M1 = true;
    first_M2 = true;
}