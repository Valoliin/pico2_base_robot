#ifndef MKS_SERVO_H
#define MKS_SERVO_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

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
#endif