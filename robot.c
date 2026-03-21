#include "mks_servo.h"
#include <stdio.h>
int main()
{
    stdio_init_all();

    // 1. Initialisation matérielle
    mks_init();

    // Pause de sécurité au démarrage (3 secondes pour le moteur)
    sleep_ms(3000);

    // On crée un tableau vide de 7 cases pour stocker notre trame
    uint8_t ma_trame[7];

    while (1)
    {
        // ÉTAPE 1 : On prépare les données (Vitesse 30, Dir 1, Acc 16)
        mks_set_speed(ma_trame, 1, 1, 30, 16);

        // ÉTAPE 2 : On calcule le CRC et on le place manuellement au bout
        // On calcule sur les 6 premiers octets
        ma_trame[6] = mks_get_checksum(ma_trame, 6);

        // ÉTAPE 3 : SI on veut envoyer, on appelle mks_send
        // (Tu pourrais décider de NE PAS l'envoyer si une condition n'est pas remplie)
        mks_send(ma_trame, 7);

        // ÉTAPE 4 : On vérifie si le moteur est content
        if (mks_read_ack())
        {
            // Success
        }

        sleep_ms(5000);
        // ÉTAPE 1 : On prépare les données (Vitesse 30, Dir 1, Acc 16)
        mks_set_speed(ma_trame, 1, 0, 30, 16);

        // ÉTAPE 2 : On calcule le CRC et on le place manuellement au bout
        // On calcule sur les 6 premiers octets
        ma_trame[6] = mks_get_checksum(ma_trame, 6);

        // ÉTAPE 3 : SI on veut envoyer, on appelle mks_send
        // (Tu pourrais décider de NE PAS l'envoyer si une condition n'est pas remplie)
        mks_send(ma_trame, 7);

        // ÉTAPE 4 : On vérifie si le moteur est content
        if (mks_read_ack())
        {
            // Success
        }

        sleep_ms(5000);
    }
}