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

// Inversion flags for sensor readings (defined in odom.cpp)
extern bool invert_dx1;
extern bool invert_dy1;
extern bool invert_dx2;
extern bool invert_dy2;
extern bool invert_dx3;
extern bool invert_dy3;

// Conversion constants.
// Defined here as inline to be available across translation units without linker errors.
inline const float C1 = 0.000039693255f;
inline const float C2 = 0.000039846037f;
inline const float C3 = 0.000036990438f;

// Fonction mathématique
void updateOdometry(int16_t dx1, int16_t dy1, int16_t dx2, int16_t dy2, int16_t dx3, int16_t dy3);

#endif // ODOM_HPP