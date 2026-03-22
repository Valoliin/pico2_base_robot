#include "lib/mks_servo/mks_servo.h"
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
    uint8_t cal[7];
    uint8_t trame_groupee[52];
    uint8_t cmd_m1[7];
    uint8_t cmd_m2[7];
    uint8_t cmd_m3[7];

    // --- LANCEMENT DE LA CALIBRATION ---
    mks_set_calibrate(cal, 1);
    cal[4] = mks_get_checksum(cal, 4);
    mks_send(cal, 5);
    sleep_ms(2000);

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

        // 1. On prépare la "feuille blanche" de 52 octets (remplie de zéros)
        mks_init_long_packet(trame_groupee);

        // 2. On prépare l'action de tes 3 moteurs (sans calculer le CRC ici)
        mks_set_speed(cmd_m1, 1, 0, 50, 16); // Moteur ID 1 : Avance
        mks_set_speed(cmd_m2, 2, 0, 50, 16); // Moteur ID 2 : Recule
        mks_set_speed(cmd_m3, 3, 0, 15, 10); // Moteur ID 3 : Avance doucement

        // 3. On les insère dans les 3 premiers "slots" (0, 1 et 2)
        mks_add_to_long_packet(trame_groupee, 0, cmd_m1, 6); // Slot 0 = Octets 1 à 10
        mks_add_to_long_packet(trame_groupee, 1, cmd_m2, 6); // Slot 1 = Octets 11 à 20
        mks_add_to_long_packet(trame_groupee, 2, cmd_m3, 6); // Slot 2 = Octets 21 à 30

        // Les slots 3 et 4 resteront automatiquement remplis de 0x00 !

        // 4. Le grand Checksum final (sur les 51 premiers octets)
        trame_groupee[51] = mks_get_checksum(trame_groupee, 51);

        // 5. Envoi groupé instantané !
        mks_send(trame_groupee, 52);

        sleep_ms(2000);
    }
}