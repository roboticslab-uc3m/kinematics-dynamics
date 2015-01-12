// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RavePart2.hpp"

// ----------------------------------------------------------------------------

void teo::RavePart2::setEncRaw(const int Index, const double Position) {
    // printf("[SharedArea] setEncRaw.\n");
    encRawMutex.wait();
    encRaw[Index] = Position;
    encRawMutex.post();
}

// ----------------------------------------------------------------------------

double teo::RavePart2::getEncRaw(const int Index) {
    // printf("[SharedArea] getEncRaw.\n");
    double Position;
    encRawMutex.wait();
    Position = encRaw[Index];
    encRawMutex.post();
    return Position;
}

// ----------------------------------------------------------------------------

double teo::RavePart2::getEncExposed(const int Index) {
    double RawPosition;
    encRawMutex.wait();
    RawPosition = encRaw[Index];
    encRawMutex.post();
    // printf("[SharedArea] get.\n");
    return RawPosition / encRawExposed[Index];
}

// ----------------------------------------------------------------------------

