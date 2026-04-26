/**
 * PROTOCOLE DE CALIBRATION DE L'ODOMÉTRIE (PAA5100 120°)
 * * 1. PRÉPARATION :
 * - S'assurer que les constantes C1, C2, C3 sont à 0.001f (valeur par défaut).
 * - Lancer l'agent micro-ROS et écouter le topic :
 * 'ros2 topic echo /debug_sensors --field data'
 * * 2. CALIBRATION DES TRANSLATIONS (Coefficients C1, C2, C3) :
 * - Placer le robot sur une règle de 500 mm (0.5 m).
 * - Pousser le robot bien droit sur 500 mm.
 * - Relever la valeur affichée pour chaque capteur (data[1], data[3], data[5]).
 * - Calculer le nouveau coefficient pour chaque capteur :
 * C_nouveau = C_actuel * (0.50 / Valeur_Lue)
 * - Note : Si data[0, 2, 4] (axes radiaux) ne sont pas à 0, le capteur est physiquement mal aligné.
 * * 3. CALIBRATION DE LA ROTATION (R_ROBOT) :
 * - Une fois C1, C2, C3 réglés, faire pivoter le robot sur lui-même (10 tours).
 * - Observer 'global_Theta' sur le topic /odom.
 * - Si l'angle final n'est pas 0 (ou 62.83 rad), ajuster R_ROBOT :
 * R_nouveau = R_actuel * (Angle_Mesuré / Angle_Théorique)
 */
#include <math.h>
#include <stdint.h>

// ==============================================================================
// CONFIGURATION DE L'ODOMÉTRIE
// ==============================================================================

// Flag pour basculer entre tes tests et le match avec ROS2
#define MODE_STANDALONE 1 // 1: Calcule X, Y, Theta globaux | 0: Renvoie juste les Deltas locaux

// Rayon géométrique du robot (distance entre le centre et les capteurs)
// À AJUSTER : Met la valeur exacte de ta CAO (ex: 0.125 pour 12.5 cm)
const float R_ROBOT = 0.125f;

// Constantes trigonométriques pré-calculées pour la configuration 120°
const float K1 = 1.0f / 3.0f;
const float K2 = 0.5f;
const float K3 = sqrtf(3.0f) / 2.0f;
const float KW = 1.0f / (3.0f * R_ROBOT);

// Facteurs de conversion (Gains) : Millimètres par "tick" du capteur PAA15100.
// À AJUSTER lors de tes tests sur la table pour compenser les micro-défauts d'impression 3D.
// Si le capteur 1 compte trop de distance, diminue légèrement C1.
float C1 = 0.001f; // Exemple : 1 tick = 1 mm (à calibrer !)
float C2 = 0.001f;
float C3 = 0.001f;

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
    float mx1 = C1 * (float)dx1;
    float my1 = C1 * (float)dy1;
    float mx2 = C2 * (float)dx2;
    float my2 = C2 * (float)dy2;
    float mx3 = C3 * (float)dx3;
    float my3 = C3 * (float)dy3;

    // 2. Application de la matrice (Système 120°)
    delta_Xr = K1 * (mx1 - (K2 * mx2) - (K3 * my2) - (K2 * mx3) + (K3 * my3));
    delta_Yr = K1 * (my1 + (K3 * mx2) - (K2 * my2) - (K3 * mx3) - (K2 * my3));
    delta_Theta = KW * (my1 + my2 + my3);

    // 3. Intégration Globale (Runge-Kutta 2)
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
