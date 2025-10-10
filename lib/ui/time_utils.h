#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <time.h>

/**
 * @brief Converte un timestamp GMT/UTC in un tempo locale CET/CEST (Europa Centrale).
 * * Questa funzione esegue l'aritmetica del tempo direttamente, includendo le regole
 * DST europee (ultima domenica di Marzo/Ottobre), ed è completamente thread-safe.
 *
 * @param gmt_timestamp Il timestamp in secondi (time_t) GMT/UTC.
 * @param timeinfo Puntatore a una struct tm dove salvare il tempo convertito.
 * @return 0 in caso di successo, -1 in caso di errore.
 */
int convert_gmt_to_cet(time_t gmt_timestamp, struct tm *timeinfo);

#endif // TIME_UTILS_H