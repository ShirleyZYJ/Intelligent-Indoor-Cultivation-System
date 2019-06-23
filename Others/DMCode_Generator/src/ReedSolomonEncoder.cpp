#include <iostream>
#include "ReedSolomonEncoder.h"

//#define DEBUG_RS_ENCODE

using namespace std;

void ReedSolomonEncoder(uchar* code, uchar code_Size, uchar message_Size) {

    // size of code: code_Size
    // code[0]~code[message_Size-1]: message
    // code[message_Size]~code[code_Size-1]: correction
    uchar correction_Size = code_Size - message_Size;
    uchar correction[correction_Size];
    uchar poly_Correction[correction_Size * correction_Size];
    for(uchar i = 0; i < correction_Size; i++ ) {
        //sum of each polynomial
        uchar sum = 0;
        for(uchar j = 0; j < message_Size; j++ )
            sum = GfAdd(sum, GfMult( code[j], GfValueOf[(code_Size - 1 - j) * (i + 1) ] ));
        correction[i] = sum;
        //polynomial matrix
        int currentRow = i * correction_Size;
        for(uchar j = 0; j < correction_Size; j++ ) {
            poly_Correction[currentRow + j] = GfValueOf[j * (i + 1)];
        }
    }

    GausEliminate(poly_Correction, correction, correction_Size);

    for(uchar i = 0; i < correction_Size; i++)
        code[message_Size + i] = correction[correction_Size - 1 - i];

#ifdef DEBUG_RS_ENCODE
    cout  << "The generated " << (int)code_Size << "-bit Reed-solomon codes: " << endl;
    cout << "\t";
    for(uchar i = 0; i < code_Size; i++)
        cout << (int)code[i] << " ";
    cout << endl << endl;
#endif
}
