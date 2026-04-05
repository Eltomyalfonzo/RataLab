// ================================================================
//  Navegacion.cpp
// ================================================================
#include "Navegacion.h"
#include "config.h"
#include "globals.h"
#include "Motors.h"
#include "Ultrasonido.h"

// =============================================================
//  Tarea ultrasonidos — modo autónomo
// =============================================================
void tareaUltra(void*) {
    TickType_t t = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&t, pdMS_TO_TICKS(80));
        if (!sistemaIniciado || modoTremouse) continue;

        DatosSensores d;
        d.centro    = ultrasonido(TRIG_C, ECHO_C); vTaskDelay(pdMS_TO_TICKS(20));
        d.derecha   = ultrasonido(TRIG_R, ECHO_R); vTaskDelay(pdMS_TO_TICKS(20));
        d.izquierda = ultrasonido(TRIG_L, ECHO_L);
        xQueueOverwrite(queueSensores, &d);
    }
}

// =============================================================
//  Tarea navegación autónoma reactiva
// =============================================================
void tareaNav(void*) {
    TickType_t t = xTaskGetTickCount();
    DatosSensores s;
    while (true) {
        vTaskDelayUntil(&t, pdMS_TO_TICKS(150));
        if (!sistemaIniciado || !modoAutonomo || modoTremouse) continue;
        if (xQueuePeek(queueSensores, &s, 0) != pdTRUE) continue;

        if (s.centro < DIST_GIRO) {
            motorStop(); vTaskDelay(pdMS_TO_TICKS(150));
            if      (s.izquierda > DIST_GIRO) { mvIzquierda(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO)); }
            else if (s.derecha   > DIST_GIRO) { mvDerecha(VEL_AUTO);   vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO)); }
            else                               { mvIzquierda(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO * 2)); }
            motorStop(); vTaskDelay(pdMS_TO_TICKS(200));
        } else if (s.derecha > DIST_GIRO) {
            mvAdelante(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(400));
            motorStop(); vTaskDelay(pdMS_TO_TICKS(100));
            mvDerecha(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO));
            motorStop(); vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            if      (s.derecha < DIST_MINIMA)    mvAdelante(VEL_AUTO - 20);
            else if (s.derecha > DIST_PARED + 5) mvAdelante(VEL_AUTO + 10);
            else                                 mvAdelante(VEL_AUTO);
        }
    }
}
