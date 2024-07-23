#ifndef INFRALOC_UTILS_H
#define INFRALOC_UTILS_H

#include "mymath.hpp"

#include <array>

#ifndef I_SERIAL
#define I_SERIAL Serial
#endif

void printArray(const std::array<number_t, 16> &arr, const int k);

void printArray(const std::array<number_t, 16> &arr, const number_t k);

#endif // INFRALOC_UTILS_H
