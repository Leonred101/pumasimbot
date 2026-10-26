/********************************************************
 *                                                      *
 *      bug1.h                                          *
 *                                                      *
 *      Algoritmo Bug 1 basado en Odometría Relativa   *
 *      Autónoma (por conteo e integración de pasos)    *
 *      con Estado de Desatasco Anti-Esquinas (RECOVERY)*
 *                                                      *
 *      FI-UNAM                                         *
 *                                                      *
 ********************************************************/

#ifndef BUG1_H
#define BUG1_H

#include <math.h>
#include <stdbool.h>

// =============================================================================
// DEFINICIONES Y CONTROL PARA BUG 1
// =============================================================================

#define BUG1_GO_TO_GOAL          0
#define BUG1_CIRCUMNAVIGATE      1
#define BUG1_GO_TO_LEAVE_RIGHT   2
#define BUG1_TURN_180            3
#define BUG1_GO_TO_LEAVE_LEFT    4
#define BUG1_GOAL_REACHED        5
#define BUG1_UNREACHABLE         6
#define BUG1_RECOVERY            7

// Odometría relativa respecto al punto de impacto H
static float rel_x = 0.0f;
static float rel_y = 0.0f;
static float rel_theta = 0.0f;

// Variables de estado y registro de Bug 1
static float alpha_goal_H = 0.0f;
static float max_progress_L = -999999.0f;
static float best_x_L = 0.0f;
static float best_y_L = 0.0f;
static int best_step_L = 0;
static int circ_steps = 0;
static bool departed_H = false;
static int cooldown_leave = 0;
static float prev_dist_to_L = 999999.0f;
static int wall_lost_count = 0;
static float turned_angle_180 = 0.0f;

// Variables para detección y recuperación de atascos en esquinas
static float last_check_x = 0.0f;
static float last_check_y = 0.0f;
static int stuck_counter = 0;
static int recovery_substep = 0;
static int recovery_return_state = BUG1_CIRCUMNAVIGATE;

// Actualizador cinemático de odometría por pasos
static void update_odometry(int action, float advance, float angle) {
    if (action == FORWARD) {
        rel_x += advance * cos(rel_theta);
        rel_y += advance * sin(rel_theta);
    }
    else if (action == BACKWARD) {
        rel_x -= advance * cos(rel_theta);
        rel_y -= advance * sin(rel_theta);
    }
    else if (action == LEFT) {
        rel_theta += angle;
    }
    else if (action == RIGHT) {
        rel_theta -= angle;
    }
    else if (action == LEFTADVANCE) {
        rel_theta += angle;
        rel_x += advance * cos(rel_theta);
        rel_y += advance * sin(rel_theta);
    }
    else if (action == RIGHTADVANCE) {
        rel_theta -= angle;
        rel_x += advance * cos(rel_theta);
        rel_y += advance * sin(rel_theta);
    }
    else if (action == LEFTADVANCETWICE) {
        rel_theta += 2.0f * angle;
        rel_x += advance * cos(rel_theta);
        rel_y += advance * sin(rel_theta);
    }
    else if (action == RIGHTADVANCETWICE) {
        rel_theta -= 2.0f * angle;
        rel_x += advance * cos(rel_theta);
        rel_y += advance * sin(rel_theta);
    }
}

// Máquina de estados Bug 1 (Odometría Relativa y Desatasco Anti-Esquinas)
static inline AdvanceAngle state_machine_bug1(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light){

    AdvanceAngle gen_vector;
    int obs;
    int j;
    float left_side = 0.0f;
    float right_side = 0.0f;
    float min_right = 999.0f;
    float min_left = 999.0f;
    int value = 0;
    int chosen_action = STOP;
    printf("\n\n **************** Student State Machine (Bug 1) *********************\n");

    float dist_to_H = 0.0f;
    float progress = 0.0f;
    float norm_angle = 0.0f;

    // 1. Detección sensorial robusta (promedio y distancia mínima por flanco)
    int half_sensors = num_sensors / 2;
    for(j = 0; j < half_sensors; j++){
        right_side += observations.sensors[j];
        if (observations.sensors[j] < min_right) min_right = observations.sensors[j];
    }
    for(j = half_sensors; j < num_sensors; j++){
        left_side += observations.sensors[j];
        if (observations.sensors[j] < min_left) min_left = observations.sensors[j];
    }

    right_side /= half_sensors;
    left_side /= half_sensors;

    // Sensores frontales centrales
    float front_dist = (observations.sensors[half_sensors - 1] + observations.sensors[half_sensors]) / 2.0f;
    if (observations.sensors[half_sensors - 1] < front_dist) front_dist = observations.sensors[half_sensors - 1];
    if (observations.sensors[half_sensors] < front_dist) front_dist = observations.sensors[half_sensors];

    // Evaluación combinada: sensible aun con 16 sensores
    float right_eval = (min_right < right_side) ? min_right : right_side;
    float left_eval = (min_left < left_side) ? min_left : left_side;

    // Cuantización de obstáculos (THRS = 0.08)
    if (left_eval < THRS) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    if (right_eval < THRS) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    obs = value;

    if (cooldown_leave > 0) cooldown_leave--;

    // Condición de paro global al llegar a la meta
    if (intensity == 1) {
        state = BUG1_GOAL_REACHED;
    }

    // =========================================================================
    // DETECTOR DE ATASCOS EN ESQUINAS Y PAREDES
    // =========================================================================
    if (state == BUG1_CIRCUMNAVIGATE || state == BUG1_GO_TO_LEAVE_RIGHT || state == BUG1_GO_TO_LEAVE_LEFT) {
        float disp = sqrt((rel_x - last_check_x) * (rel_x - last_check_x) + (rel_y - last_check_y) * (rel_y - last_check_y));
        if (disp < 0.008f) {
            stuck_counter++;
        } else {
            stuck_counter = 0;
            last_check_x = rel_x;
            last_check_y = rel_y;
        }

        // Si lleva 4 pasos consecutivos bloqueado sin avanzar: entrar en RECOVERY
        if (stuck_counter >= 4) {
            printf("\n>>> [BUG 1] ¡ATASCO DETECTADO EN ESQUINA! (pasos bloqueado: %d). Activando estado RECOVERY...\n", stuck_counter);
            recovery_return_state = state;
            recovery_substep = 0;
            stuck_counter = 0;
            state = BUG1_RECOVERY;
        }
    }

    switch (state) {

        // =====================================================================
        // ESTADO 0: AVANZAR DIRECTO HACIA LA META (GO TO GOAL)
        // =====================================================================
        case BUG1_GO_TO_GOAL: {
            bool obstacle_blocks_path = false;
            if (front_dist < 0.075f || obs == 3) {
                obstacle_blocks_path = true;
            } else if ((dest == 0 || dest == 2) && (obs == 1)) {
                obstacle_blocks_path = true;
            } else if ((dest == 1 || dest == 3) && (obs == 2)) {
                obstacle_blocks_path = true;
            }

            if (obstacle_blocks_path && cooldown_leave == 0) {
                rel_x = 0.0f;
                rel_y = 0.0f;
                rel_theta = 0.0f;

                norm_angle = angle_light;
                if (norm_angle > PI) norm_angle -= 2.0f * PI;
                alpha_goal_H = norm_angle;

                max_progress_L = 0.0f;
                best_x_L = 0.0f;
                best_y_L = 0.0f;
                best_step_L = 0;
                circ_steps = 0;
                departed_H = false;
                wall_lost_count = 0;
                stuck_counter = 0;
                last_check_x = 0.0f;
                last_check_y = 0.0f;

                printf("\n>>> [BUG 1] Impacto H detectado (front: %.3f, obs: %d). Iniciando vuelta 360.\n", front_dist, obs);

                *next_state = BUG1_CIRCUMNAVIGATE;
                chosen_action = LEFT;
            } 
            else {
                if (front_dist < 0.040f) {
                    chosen_action = BACKWARD;
                }
                else if (dest == 0) chosen_action = RIGHTADVANCETWICE;
                else if (dest == 1) chosen_action = LEFTADVANCETWICE;
                else if (dest == 2) chosen_action = RIGHTADVANCE;
                else if (dest == 3) chosen_action = LEFTADVANCE;
                else chosen_action = FORWARD;

                *next_state = BUG1_GO_TO_GOAL;
            }
            break;
        }

        // =====================================================================
        // ESTADO 1: CIRCUNNAVEGACIÓN 360° (FOLLOW WALL POR LA DERECHA)
        // =====================================================================
        case BUG1_CIRCUMNAVIGATE: {
            circ_steps++;

            dist_to_H = sqrt(rel_x * rel_x + rel_y * rel_y);

            if (!departed_H && dist_to_H > 0.12f) {
                departed_H = true;
            }

            progress = rel_x * cos(alpha_goal_H) + rel_y * sin(alpha_goal_H);
            if (progress > max_progress_L) {
                max_progress_L = progress;
                best_x_L = rel_x;
                best_y_L = rel_y;
                best_step_L = circ_steps;
            }

            if (departed_H && dist_to_H < 0.08f && circ_steps >= 25) {
                printf("\n>>> [BUG 1] Vuelta 360 COMPLETADA en %d pasos (dist_to_H: %.3f).\n", circ_steps, dist_to_H);
                printf(">>> [BUG 1] Punto optimo L registrado en paso %d (%.3f, %.3f) con progreso %.3f.\n", best_step_L, best_x_L, best_y_L, max_progress_L);

                if (max_progress_L <= 0.005f) {
                    printf("\n>>> [BUG 1] META INALCANZABLE (ningun punto del obstaculo mejora respecto a H). Deteniendo robot.\n");
                    *next_state = BUG1_UNREACHABLE;
                    chosen_action = STOP;
                    break;
                }

                prev_dist_to_L = 999999.0f;
                wall_lost_count = 0;
                stuck_counter = 0;

                if (best_step_L <= circ_steps / 2) {
                    printf(">>> [BUG 1] Ruta mas corta hacia adelante (paso %d <= %d/2). Continuando con pared a la derecha.\n", best_step_L, circ_steps);
                    *next_state = BUG1_GO_TO_LEAVE_RIGHT;
                } else {
                    printf(">>> [BUG 1] Ruta mas corta en sentido inverso (paso %d > %d/2). Girando 180 en H para ir con pared a la izquierda!\n", best_step_L, circ_steps);
                    turned_angle_180 = 0.0f;
                    *next_state = BUG1_TURN_180;
                    chosen_action = LEFT;
                    break;
                }
            }

            // Seguimiento con pared a la derecha
            if (front_dist < 0.040f) {
                chosen_action = BACKWARD;
                wall_lost_count = 0;
            }
            else if (front_dist < 0.070f || obs == 3) {
                chosen_action = LEFT;
                wall_lost_count = 0;
            }
            else if (obs == 1) {
                wall_lost_count = 0;
                if (right_eval < 0.045f) {
                    chosen_action = LEFT;
                } else {
                    chosen_action = FORWARD;
                }
            }
            else if (obs == 0) {
                wall_lost_count++;
                if (wall_lost_count <= 2) {
                    chosen_action = FORWARD;
                } else if (wall_lost_count <= 4) {
                    chosen_action = RIGHT;
                } else {
                    chosen_action = FORWARD;
                    wall_lost_count = 0;
                }
            }
            else if (obs == 2) {
                chosen_action = RIGHT;
                wall_lost_count = 0;
            }

            if (*next_state != BUG1_GO_TO_LEAVE_RIGHT && *next_state != BUG1_TURN_180 && *next_state != BUG1_UNREACHABLE && *next_state != BUG1_RECOVERY) {
                *next_state = BUG1_CIRCUMNAVIGATE;
            }
            break;
        }

        // =====================================================================
        // ESTADO 2: RETORNO DIRECTO A L (PARED A LA DERECHA)
        // =====================================================================
        case BUG1_GO_TO_LEAVE_RIGHT: {
            float dist_to_L = sqrt((rel_x - best_x_L) * (rel_x - best_x_L) + (rel_y - best_y_L) * (rel_y - best_y_L));

            if (dist_to_L < 0.045f || (prev_dist_to_L < 0.060f && dist_to_L > prev_dist_to_L)) {
                printf("\n>>> [BUG 1] Punto optimo L alcanzado con alta precision (dist_to_L: %.3f). Desacoplandose a la meta.\n", dist_to_L);
                cooldown_leave = 4;
                *next_state = BUG1_GO_TO_GOAL;
                
                if (dest == 0 || dest == 2) chosen_action = RIGHT;
                else if (dest == 1 || dest == 3) chosen_action = LEFT;
                else chosen_action = FORWARD;
                break;
            }

            prev_dist_to_L = dist_to_L;

            if (front_dist < 0.040f) {
                chosen_action = BACKWARD;
                wall_lost_count = 0;
            }
            else if (front_dist < 0.070f || obs == 3) {
                chosen_action = LEFT;
                wall_lost_count = 0;
            }
            else if (obs == 1) {
                wall_lost_count = 0;
                if (right_eval < 0.045f) {
                    chosen_action = LEFT;
                } else {
                    chosen_action = FORWARD;
                }
            }
            else if (obs == 0) {
                wall_lost_count++;
                if (wall_lost_count <= 2) {
                    chosen_action = FORWARD;
                } else if (wall_lost_count <= 4) {
                    chosen_action = RIGHT;
                } else {
                    chosen_action = FORWARD;
                    wall_lost_count = 0;
                }
            }
            else if (obs == 2) {
                chosen_action = RIGHT;
                wall_lost_count = 0;
            }

            if (*next_state != BUG1_RECOVERY) {
                *next_state = BUG1_GO_TO_LEAVE_RIGHT;
            }
            break;
        }

        // =====================================================================
        // ESTADO 3: GIRO DE 180° EN EL PUNTO H PARA TOMAR LA RUTA MÁS CORTA
        // =====================================================================
        case BUG1_TURN_180: {
            turned_angle_180 += max_angle;
            chosen_action = LEFT;

            printf(">>> [BUG 1] Ejecutando giro 180 en H (rotado: %.2f / %.2f rad)...\n", turned_angle_180, PI);

            if (turned_angle_180 >= PI - 0.05f) {
                printf(">>> [BUG 1] Giro de 180 completado en H. Iniciando retorno hacia L con pared a la izquierda!\n");
                turned_angle_180 = 0.0f;
                prev_dist_to_L = 999999.0f;
                wall_lost_count = 0;
                stuck_counter = 0;
                *next_state = BUG1_GO_TO_LEAVE_LEFT;
            } else {
                *next_state = BUG1_TURN_180;
            }
            break;
        }

        // =====================================================================
        // ESTADO 4: RETORNO INVERSO A L (PARED A LA IZQUIERDA)
        // =====================================================================
        case BUG1_GO_TO_LEAVE_LEFT: {
            float dist_to_L = sqrt((rel_x - best_x_L) * (rel_x - best_x_L) + (rel_y - best_y_L) * (rel_y - best_y_L));

            if (dist_to_L < 0.045f || (prev_dist_to_L < 0.060f && dist_to_L > prev_dist_to_L)) {
                printf("\n>>> [BUG 1] Punto optimo L alcanzado en sentido inverso (dist_to_L: %.3f). Desacoplandose a la meta.\n", dist_to_L);
                cooldown_leave = 4;
                *next_state = BUG1_GO_TO_GOAL;

                if (dest == 0 || dest == 2) chosen_action = RIGHT;
                else if (dest == 1 || dest == 3) chosen_action = LEFT;
                else chosen_action = FORWARD;
                break;
            }

            prev_dist_to_L = dist_to_L;

            if (front_dist < 0.040f) {
                chosen_action = BACKWARD;
                wall_lost_count = 0;
            }
            else if (front_dist < 0.070f || obs == 3) {
                chosen_action = RIGHT;
                wall_lost_count = 0;
            }
            else if (obs == 2) {
                wall_lost_count = 0;
                if (left_eval < 0.045f) {
                    chosen_action = RIGHT;
                } else {
                    chosen_action = FORWARD;
                }
            }
            else if (obs == 0) {
                wall_lost_count++;
                if (wall_lost_count <= 2) {
                    chosen_action = FORWARD;
                } else if (wall_lost_count <= 4) {
                    chosen_action = LEFT;
                } else {
                    chosen_action = FORWARD;
                    wall_lost_count = 0;
                }
            }
            else if (obs == 1) {
                chosen_action = LEFT;
                wall_lost_count = 0;
            }

            if (*next_state != BUG1_RECOVERY) {
                *next_state = BUG1_GO_TO_LEAVE_LEFT;
            }
            break;
        }

        // =====================================================================
        // ESTADO 5: META ALCANZADA
        // =====================================================================
        case BUG1_GOAL_REACHED:
            printf("\n>>> [BUG 1] ********** META ALCANZADA **********\n");
            chosen_action = STOP;
            *next_state = BUG1_GOAL_REACHED;
            break;

        // =====================================================================
        // ESTADO 6: META INALCANZABLE (DETECCIÓN FORMAL DE BUG 1)
        // =====================================================================
        case BUG1_UNREACHABLE:
            printf("\n>>> [BUG 1] ********** META INALCANZABLE (STOP) **********\n");
            chosen_action = STOP;
            *next_state = BUG1_UNREACHABLE;
            break;

        // =====================================================================
        // ESTADO 7: RECUPERACIÓN / DESATASCO ACTIVO DE ESQUINAS (RECOVERY)
        // =====================================================================
        case BUG1_RECOVERY: {
            recovery_substep++;

            // Fase 1: Retroceder 2 pasos para separarse del vértice o pared
            if (recovery_substep <= 2) {
                printf(">>> [BUG 1] RECOVERY: Retrocediendo para desatorar esquina (subpaso %d/2)...\n", recovery_substep);
                chosen_action = BACKWARD;
                *next_state = BUG1_RECOVERY;
            }
            // Fase 2: Rotar hacia espacio abierto (alejándose del obstáculo)
            else if (recovery_substep <= 4) {
                if (recovery_return_state == BUG1_GO_TO_LEAVE_LEFT) {
                    printf(">>> [BUG 1] RECOVERY: Rotando a la derecha para despejar esquina izquierda...\n");
                    chosen_action = RIGHT;
                } else {
                    printf(">>> [BUG 1] RECOVERY: Rotando a la izquierda para despejar esquina derecha...\n");
                    chosen_action = LEFT;
                }
                *next_state = BUG1_RECOVERY;
            }
            // Fase 3: Avanzar un paso despegado del obstáculo
            else if (recovery_substep == 5) {
                printf(">>> [BUG 1] RECOVERY: Avanzando despegado de la esquina...\n");
                chosen_action = FORWARD;
                *next_state = BUG1_RECOVERY;
            }
            // Fase 4: Desatasco completado, retornar al curso normal
            else {
                printf(">>> [BUG 1] RECOVERY: Despegue completado exitosamente. Retomando curso.\n");
                last_check_x = rel_x;
                last_check_y = rel_y;
                stuck_counter = 0;
                wall_lost_count = 0;
                *next_state = recovery_return_state;
                chosen_action = FORWARD;
            }
            break;
        }

        default:
            chosen_action = FORWARD;
            *next_state = BUG1_GO_TO_GOAL;
            break;
    }

    update_odometry(chosen_action, Mag_Advance, max_angle);
    gen_vector = generate_output(chosen_action, Mag_Advance, max_angle);

    return gen_vector;
}

#endif // BUG1_H
