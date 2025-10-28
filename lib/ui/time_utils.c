#include <time.h>
#include <stdio.h>
#include <stdbool.h>

// Le regole per l'Europa Centrale (CET/CEST) sono:
// DST INIZIA: ultima Domenica di Marzo alle 02:00:00 (si passa a 03:00:00)
// DST FINISCE: ultima Domenica di Ottobre alle 03:00:00 (si torna a 02:00:00)
#define CET_OFFSET_SECONDS  (1 * 3600) // Offset CET: UTC + 1 ora
#define DST_OFFSET_SECONDS  (1 * 3600) // Offset DST: 1 ora aggiuntiva

// --- Funzione Ausiliaria: Calcola l'ultima Domenica del Mese ---
/**
 * @brief Calcola il giorno in cui cade l'ultima domenica del mese specificato nell'anno specificato.
 * @param anno L'anno (es. 2025).
 * @param mese Il mese (1=Gennaio, 12=Dicembre).
 * @return Il giorno del mese (1-31) in cui cade l'ultima domenica.
 */
int get_last_sunday(int anno, int mese) {
    struct tm t = {0};
    time_t time_temp;
    
    // Inizia impostando il 1° giorno del mese successivo (o Gennaio se è Dicembre)
    t.tm_year = anno - 1900;
    t.tm_mon = mese % 12; // Use modulo 12 to handle December (12 -> 0 for January of next year)
    if (mese == 12) t.tm_year++;
    t.tm_mday = 1;
    t.tm_hour = 12; // Per evitare problemi di fuso orario durante mktime

    // Ottieni il timestamp del primo giorno del mese successivo
    time_temp = mktime(&t);

    // Torna indietro di 24 ore (al 31/12 o all'ultimo giorno del mese corrente)
    time_temp -= (24 * 3600); 

    // Converti questo ultimo giorno in struct tm
    gmtime_r(&time_temp, &t); 
    
    // Ora t contiene l'ultimo giorno del mese. 
    // Torniamo indietro fino alla domenica (tm_wday = 0)
    int diff_giorni = t.tm_wday; // tm_wday è 0 (Dom) a 6 (Sab)
    
    // La data dell'ultima domenica è il giorno corrente meno il tm_wday
    return t.tm_mday - diff_giorni;
}

// --- Funzione Ausiliaria: Verifica se un tempo è in DST ---
/**
 * @brief Verifica se la data e ora specificate (UTC) ricadono nel periodo di DST europeo.
 * * @param t Puntatore alla struct tm che rappresenta l'ora in CET (UTC + 1 ora).
 * @return true se DST è attiva (CEST), false altrimenti.
 */
bool is_cest_active(const struct tm *t) {
    int mese = t->tm_mon + 1; // tm_mon è 0-11
    int giorno = t->tm_mday;
    int ora = t->tm_hour;
    int anno = t->tm_year + 1900;

    // DST non è MAI attiva da Novembre (11) a Febbraio (2)
    if (mese < 3 || mese > 10) {
        return false;
    }
    // DST è SEMPRE attiva da Aprile (4) a Settembre (9)
    if (mese > 3 && mese < 10) {
        return true;
    }

    // Caso 1: Marzo (Inizio DST)
    if (mese == 3) {
        int last_sunday_march = get_last_sunday(anno, 3);
        
        if (giorno < last_sunday_march) {
            return false; // Prima della Domenica di inizio
        } else if (giorno > last_sunday_march) {
            return true; // Dopo la Domenica di inizio
        } else {
            // È l'ultima Domenica di Marzo. DST inizia alle 02:00 CET (01:00 UTC).
            // A questo punto, 't' è ancora in GMT/UTC, quindi l'inizio è alle 01:00 UTC.
            return (ora >= 1); 
        }
    }

    // Caso 2: Ottobre (Fine DST)
    if (mese == 10) {
        int last_sunday_october = get_last_sunday(anno, 10);
        
        if (giorno < last_sunday_october) {
            return true; // Prima della Domenica di fine
        } else if (giorno > last_sunday_october) {
            return false; // Dopo la Domenica di fine
        } else {
            // È l'ultima Domenica di Ottobre. DST finisce alle 03:00 CEST (01:00 UTC).
            // L'orologio torna indietro da 03:00 CEST a 02:00 CET.
            // A questo punto, 't' è ancora in GMT/UTC, quindi la fine è alle 01:00 UTC.
            return (ora < 1); 
        }
    }

    return false; // Non dovrebbe succedere
}

// --- Funzione Principale Richiesta ---

/**
 * @brief Converte un timestamp GMT/UTC in un tempo locale CET/CEST.
 * * @param gmt_timestamp Il timestamp in secondi (time_t) GMT/UTC.
 * @param timeinfo Puntatore a una struct tm dove salvare il tempo convertito.
 * @return 0 in caso di successo.
 */
int convert_gmt_to_cet(time_t gmt_timestamp, struct tm *timeinfo) {
    if (timeinfo == NULL) {
        return -1;
    }

    // Create a tm struct representing the input UTC time
    struct tm utc_tm;
    gmtime_r(&gmt_timestamp, &utc_tm);

    // Check if DST is active for that UTC time
    bool is_dst_active = is_cest_active(&utc_tm);

    // Manually apply the correct offset to get the final local timestamp
    time_t local_timestamp = gmt_timestamp + CET_OFFSET_SECONDS;
    if (is_dst_active) {
        local_timestamp += DST_OFFSET_SECONDS;
    }

    // Convert the final, correct local timestamp into the tm struct for display
    gmtime_r(&local_timestamp, timeinfo);
    timeinfo->tm_isdst = is_dst_active; // Manually set the DST flag

    return 0;
}