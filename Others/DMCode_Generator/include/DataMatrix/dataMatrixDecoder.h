#ifndef DATA_MATRIX_DECODER_H_
#define DATA_MATRIX_DECODER_H_
#include <iostream>
#include "dataMatrix.h"
#include "ReedSolomonDecoder.h"

uchar byteDecoder(uchar dataMatrix_Expanded[], uchar byteHead);
long long dataMatrixDecoder(uchar* dataMatrix_Detected);

#endif
