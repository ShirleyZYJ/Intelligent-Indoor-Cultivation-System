#ifndef DATA_MATRIX_ENCODER_H_
#define DATA_MATRIX_ENCODER_H_
#include <iostream>
#include "dataMatrix.h"
#include "ReedSolomonEncoder.h"

void byteEncoder(uchar* Matrix, uchar byteHead, uchar byteValue);

// if MATRIX_SIZE = 10, message is an array containing 10 numbers
// if MATRIX_SIZE = 12, message is an array containing 8 characters 
// from ' ' to 'Z' (ASCII code 32 ~ 90)
void dataMatrixEncoder(uchar* dataMatrix, char* message);

#endif
