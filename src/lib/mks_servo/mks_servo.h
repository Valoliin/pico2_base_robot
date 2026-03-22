#ifndef MKS_SERVO_H
#define MKS_SERVO_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Initialisation
    void mks_init();

    // Calcul de la trame (Remplit le buffer)
    void mks_set_speed(uint8_t *buffer, uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc);

    // Calcul du Checksum
    uint8_t mks_get_checksum(uint8_t *buffer, uint8_t size);

    // Envoi physique
    void mks_send(uint8_t *buffer, uint8_t len);

    // Lecture du retour
    bool mks_read_ack();

    // Active (1) ou Désactive (0) le moteur via le bus
    void mks_set_enable(uint8_t *buffer, uint8_t addr, uint8_t enable);

    // Arrêt d'urgence (Coupe net, attention si la vitesse est > 1000 RPM)
    void mks_set_emergency_stop(uint8_t *buffer, uint8_t addr);

    // Arrêt doux ou immédiat en mode vitesse (acc=0 -> immédiat, acc>0 -> décélération douce)
    void mks_set_stop(uint8_t *buffer, uint8_t addr, uint8_t acc);

    // Prépare la trame de calibration
    void mks_set_calibrate(uint8_t *buffer, uint8_t addr);

    // Lit le statut spécifique de la calibration (0=En cours, 1=Succès, 2=Échec, 255=Timeout)
    uint8_t mks_read_calib_status();

    // Initialise un paquet long (52 octets) rempli de zéros avec l'en-tête FC
    void mks_init_long_packet(uint8_t *long_buffer);

    // Injecte une commande standard dans un des 5 slots du paquet long
    void mks_add_to_long_packet(uint8_t *long_buffer, uint8_t slot, uint8_t *standard_frame, uint8_t frame_len);

#ifdef __cplusplus
}
#endif

#endif