#ifndef MKS_SERVO_H
#define MKS_SERVO_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

// Initialisation
void mks_init();

// 1. Calcul de la trame (Remplit le buffer)
void mks_set_speed(uint8_t *buffer, uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc);

// 2. Calcul du Checksum
uint8_t mks_get_checksum(uint8_t *buffer, uint8_t size);

// 3. Envoi physique
void mks_send(uint8_t *buffer, uint8_t len);

// 4. Lecture du retour
bool mks_read_ack();

#endif