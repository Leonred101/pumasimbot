/********************************************************
 *                                                      *
 *      state_machine_student.h                         *
 *                                                      *
 *      Controlador Principal de Estudiantes            *
 *      (Arquitectura Modular para Pumasimbot)          *
 *                                                      *
 *      FI-UNAM                                         *
 *                                                      *
 ********************************************************/

#ifndef STATE_MACHINE_STUDENT_H
#define STATE_MACHINE_STUDENT_H

// =============================================================================
// INCLUSIÓN DE ALGORITMOS MODULARES
// =============================================================================
#include "bug1.h"

// Descomenta la siguiente línea cuando implementes Bug Tangencial:
// #include "bug_tangente.h"


// =============================================================================
// 1. COMPORTAMIENTO REACTIVO DE ESTUDIANTES (SELECCIÓN 5 EN SIMULADOR)
// =============================================================================
AdvanceAngle reactive_students(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors){

    AdvanceAngle gen_vector;
    int obs;
    int j;
    float left_side = 0.0f;
    float right_side = 0.0f;
    int value = 0;
    static int step = 0;

    step++;
    printf("\n\n **************** Student Reactive Behavior %d *********************\n", step);

    for(j = 0; j < num_sensors / 2; j++){
        right_side += observations.sensors[j];
        printf("right side sensor[%d] %f\n", j, observations.sensors[j]);
    }

    for(j = num_sensors / 2; j < num_sensors; j++){
        left_side += observations.sensors[j];
        printf("left side sensor[%d] %f\n", j, observations.sensors[j]);
    }

    right_side /= (num_sensors / 2);
    left_side /= (num_sensors / 2);
    printf("Average right side %f\n", right_side);
    printf("Average left side %f\n", left_side);

    if(left_side < THRS) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    if(right_side < THRS) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    obs = value;
    printf("intensity %d obstacles %d dest %d\n", intensity, obs, dest);

    if (intensity == 1){
        gen_vector = generate_output(STOP, Mag_Advance, max_angle);
        printf("STOP\n");
        printf("\n **************** Reached light source ******************************\n");
    }
    else if (obs == 0){
        if (dest == 0){
            gen_vector = generate_output(RIGHTADVANCETWICE, Mag_Advance, max_angle);
            printf("TURN RIGHT TWICE\n");
        }
        else if (dest == 1){
            gen_vector = generate_output(LEFTADVANCETWICE, Mag_Advance, max_angle);
            printf("TURN LEFT TWICE\n");
        }
        else if (dest == 2){
            gen_vector = generate_output(RIGHTADVANCE, Mag_Advance, max_angle);
            printf("TURN RIGHT\n");
        }
        else if (dest == 3){
            gen_vector = generate_output(LEFTADVANCE, Mag_Advance, max_angle);
            printf("TURN LEFT\n");
        }
    }
    else if (obs == 1){
        gen_vector = generate_output(LEFT, Mag_Advance, max_angle);
        printf("TURN LEFT\n");
    }
    else if (obs == 2){
        gen_vector = generate_output(RIGHT, Mag_Advance, max_angle);
        printf("TURN RIGHT\n");
    }
    else if (obs == 3){
        gen_vector = generate_output(LEFT, Mag_Advance, max_angle);
        printf("TURN LEFT\n");
    }

    return gen_vector;
}


// =============================================================================
// 2. MÁQUINA DE ESTADOS DE ESTUDIANTES (SELECCIÓN 6 EN SIMULADOR)
// =============================================================================
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light){

    // Por defecto ejecuta la máquina de estados de Bug 1 modular:
    return state_machine_bug1(observations, dest, intensity, state, next_state, Mag_Advance, max_angle, num_sensors, angle_light);

    // Cuando desarrolles Bug Tangencial, podrás cambiar aquí a:
    // return state_machine_bug_tangente(observations, dest, intensity, state, next_state, Mag_Advance, max_angle, num_sensors, angle_light);
}

#endif // STATE_MACHINE_STUDENT_H
