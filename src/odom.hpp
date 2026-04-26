#ifndef ODOM_HPP
#define ODOM_HPP

#include <stdint.h>

// Variables partagées
extern float global_X;
extern float global_Y;
extern float global_Theta;
extern float delta_Xr;
extern float delta_Yr;
extern float delta_Theta;

// Fonction mathématique
void updateOdometry(int16_t dx1, int16_t dy1, int16_t dx2, int16_t dy2, int16_t dx3, int16_t dy3);

#endif // ODOM_HPP