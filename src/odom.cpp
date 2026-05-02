/**
 * PROTOCOLE DE CALIBRATION DE L'ODOMÉTRIE (PAA5100 120°)
 *
 * L'objectif est de déterminer les constantes C1, C2, C3 qui convertissent les "ticks"
 * bruts des capteurs en mètres.
 *
 * * 1. PRÉPARATION :
 * - Dans `src/robot.cpp`, activer le mode de calibration :
 *   `#define CALIBRATION_MODE 1`
 * - Compiler et flasher le code sur le Pico.
 * - Lancer l'agent micro-ROS et écouter les topics des capteurs :
 *   `ros2 topic echo /pico/capteur1`
 *   `ros2 topic echo /pico/capteur2`
 *   `ros2 topic echo /pico/capteur3`
 *
 * * 2. CALIBRATION DES CONSTANTES DE CONVERSION (C1, C2, C3) :
 * - Placer le robot sur une surface avec une bonne texture, le long d'une règle.
 * - Remettre à zéro les compteurs de ticks en publiant sur le topic de reset :
 *   `ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty`
 * - Pousser le robot bien droit sur une distance connue, par exemple 500 mm (0.5 m).
 *   Essayez de le pousser dans une direction qui maximise la lecture sur l'axe 'y'
 *   de chaque capteur (mouvement tangentiel).
 * - Relever la valeur finale des ticks accumulés pour chaque capteur (par exemple, `y` sur `/pico/capteur1`).
 *   Soit `ticks_capteur1`, `ticks_capteur2`, `ticks_capteur3`.
 *
 * - Calculer les nouvelles constantes avec la formule directe :
 *   `C1 = distance_reelle_en_metres / ticks_capteur1` (ex: 0.5 / 25000)
 *   `C2 = distance_reelle_en_metres / ticks_capteur2`
 *   `C3 = distance_reelle_en_metres / ticks_capteur3`
 *
 * - Inscrire ces nouvelles valeurs pour C1, C2, C3 dans ce fichier.
 *
 * * 3. CALIBRATION DE LA ROTATION (R_ROBOT) :
 * - Dans `src/robot.cpp`, repasser en mode normal : `#define CALIBRATION_MODE 0`
 * - Compiler et flasher.
 * - Faire pivoter le robot sur lui-même (ex: 10 tours complets, soit 62.83 radians).
 * - Observer la valeur `z` (theta) sur le topic `/pico/odom_simple`.
 * - Ajuster `R_ROBOT` avec la formule :
 *   `R_nouveau = R_actuel * (Angle_Mesuré / Angle_Théorique)`
 *
 * * 4. FINALISATION :
 * - Une fois les constantes calibrées, le robot est prêt.
 */
#include <math.h>
#include <stdint.h>
#include "odom.hpp"

// ==============================================================================
// CONFIGURATION DE L'ODOMÉTRIE
// ==============================================================================

// Flag pour basculer entre tes tests et le match avec ROS2
#define MODE_STANDALONE 1 // 1: Calcule X, Y, Theta globaux | 0: Renvoie juste les Deltas locaux

// Rayon géométrique du robot (distance entre le centre et les capteurs)
// À AJUSTER : Met la valeur exacte de ta CAO (ex: 0.125 pour 12.5 cm)
const float R_ROBOT = 0.145f;

// Constantes trigonométriques pré-calculées pour la configuration 120°
const float K1 = 1.0f / 3.0f;
const float K2 = 0.5f;
const float K3 = sqrtf(3.0f) / 2.0f;
const float KW = 1.0f / (3.0f * R_ROBOT);

// Inversion flags for sensor readings.
// Set to 'true' if the sensor's reported displacement (dx or dy) needs to be
// inverted to match the robot's coordinate system.
bool invert_dx1 = false;
bool invert_dy1 = false;
bool invert_dx2 = false;
bool invert_dy2 = false;
bool invert_dx3 = false;
bool invert_dy3 = false;
// ==============================================================================
// VARIABLES GLOBALES
// ==============================================================================

// Pose globale du robot sur la table (Unité : Mètres et Radians)
float global_X = 0.0f;
float global_Y = 0.0f;
float global_Theta = 0.0f;

// Vecteur de déplacement local à envoyer au Pi 5 (si MODE_STANDALONE == 0)
float delta_Xr = 0.0f;
float delta_Yr = 0.0f;
float delta_Theta = 0.0f;

// ==============================================================================
// FONCTION PRINCIPALE DE MISE À JOUR
// ==============================================================================

// Fonction à appeler à chaque cycle (ex: tous les 10ms) après lecture SPI des capteurs
void updateOdometry(int16_t dx1, int16_t dy1, int16_t dx2, int16_t dy2, int16_t dx3, int16_t dy3)
{
    // 1. Conversion des ticks bruts en distances réelles
    float mx1 = C1 * (float)(invert_dx1 ? -dx1 : dx1);
    float my1 = C1 * (float)(invert_dy1 ? -dy1 : dy1);
    float mx2 = C2 * (float)(invert_dx2 ? -dx2 : dx2);
    float my2 = C2 * (float)(invert_dy2 ? -dy2 : dy2);
    float mx3 = C3 * (float)(invert_dx3 ? -dx3 : dx3);
    float my3 = C3 * (float)(invert_dy3 ? -dy3 : dy3);

    // 2. Application de la matrice (Système 120°)
    delta_Xr = K1 * (mx1 - (K2 * mx2) - (K3 * my2) - (K2 * mx3) + (K3 * my3));
    delta_Yr = K1 * (my1 + (K3 * mx2) - (K2 * my2) - (K3 * mx3) - (K2 * my3)); // Correction de la formule ici
    delta_Theta = KW * (my1 + my2 + my3);

    // 4. Intégration Globale (Runge-Kutta 2)
    float theta_mid = global_Theta + (delta_Theta / 2.0f);

    global_X += (delta_Xr * cosf(theta_mid)) - (delta_Yr * sinf(theta_mid));
    global_Y += (delta_Xr * sinf(theta_mid)) + (delta_Yr * cosf(theta_mid));
    global_Theta += delta_Theta;

    // Normalisation de Theta
    while (global_Theta > (float)M_PI)
        global_Theta -= 2.0f * (float)M_PI;
    while (global_Theta <= -(float)M_PI)
        global_Theta += 2.0f * (float)M_PI;
}
