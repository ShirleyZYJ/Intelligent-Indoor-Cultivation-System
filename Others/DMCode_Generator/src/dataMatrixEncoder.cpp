#include <iostream>
#include "dataMatrixEncoder.h" // dataMatrix.h is included here
// MATRIX_SIZE_10 and MATRIX_SIZE_12 are both defined in dataMatrix.h
// BLACK_BOARDER and WITH_WHITE_BLACK are both defined in dataMatrix.h

// MESSAGE_SIZE is also defined in dataMatrix.h
// MESSAGE_SIZE = 5 when MATRIX_SIZE_10 =1
// MESSAGE_SIZE = 7 when MATRIX_SIZE_12 =1

//#define DEBUG_ENCODE

using std::cout;
using std::endl;
// Function:Encode a single byte
// Input variable:
//     Matrix: the matrix to encode
//     byteHead: the upper-left corner of each byte
//     byteValue: the value of each byte
void byteEncoder(uchar* Matrix, uchar byteHead, uchar byteValue) {
    for(char y = 2; y >= 0; y--)
        for(char x = 2; x >= 0; x--) {
            if(y == 0 && x == 2)
                continue;
            Matrix[byteHead + (MATRIX_SIZE + 2) * y + x] = byteValue % 2;
            byteValue /= 2;
        }
}

// Function:Encode a 2D code
// Input variable:
//     dataMatrix: the matrix to encode
//     message: the data information to be contained in Data Matrix
void dataMatrixEncoder(uchar* dataMatrix, char* message) {

    uchar code[CODE_SIZE] = {};
    // translate the data from char to integer
#if MATRIX_SIZE_10 // translate every 2 chars to a two-digit number
    for(int i = 0; i < MESSAGE_SIZE; i++) {
        code[i]=10*(int)(message[2*i]-'0')+(int)(message[2*i+1]-'0')+130;
    }
#elif MATRIX_SIZE_12
    for(int i = 0; i < MESSAGE_SIZE; i++) {
        code[i]=(uchar)message[i]+1;
    }
#endif

#ifdef DEBUG_ENCODE
    cout << "The message is: " << endl;
    for(int i = 0; i < MESSAGE_SIZE; i++) {
            cout << (int)code[i] << " ";
    }
    cout << endl;
#endif
    // encode the message by Reed-Solomon error correction
    // fill the last 7 bit of code[12] with the correction codes
    ReedSolomonEncoder(code, CODE_SIZE, MESSAGE_SIZE);
#ifdef DEBUG_ENCODE
    cout << "The RS-encoded code is: " << endl;
    for(int i = 0; i < CODE_SIZE; i++) {
            cout << (int)code[i] << " ";
    }
    cout << endl;
#endif
    // create an expanded matrix for data storage (explained in the report)
    uchar dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 2)] = {};

    // Eecode each byte
    for(uchar index = 0; index < CODE_SIZE; index++) {
        byteEncoder(dataMatrix_Expanded, dataHead[index], code[index]);
    }

#if MATRIX_SIZE_10
    // Depart the patial data and redundancies on the edges

    // Depart message code 1
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 2 + MATRIX_SIZE];

    // Depart message code 3
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 3] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 1];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 4] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 2];

    // Depart message code 4
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 3];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 6] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 4];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 7] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 5];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 3];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 6] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 4];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 7] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 5];

    // Depart Redundancy 2
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + (MATRIX_SIZE + 1)];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + (MATRIX_SIZE + 1)];

#elif MATRIX_SIZE_12
    // Complete the patial data on the edges

    // Complete message code 1
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 8 + MATRIX_SIZE];

    // Complete message code 3
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 6];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 3] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 7];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 4] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 8];

    // Complete message code 4
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 9];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 6] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 10];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 7] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 11];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 9];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 6] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 10];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 7] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 11];

    // Complete message code 7
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 9 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 10 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + MATRIX_SIZE];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 10 + (MATRIX_SIZE + 1)];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + (MATRIX_SIZE + 1)];

    // Complete message code 8
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 10] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 1];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 2];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 2 + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 1];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 2];

#endif

#if BLACK_BOARDER
#if WITH_WHITE_BLANK
    // draw the surrounding white blank
    for(int i = 0; i < MATRIX_SIZE + 4; i++) {
        dataMatrix[i] = 0;
        dataMatrix[(MATRIX_SIZE + 4) * (MATRIX_SIZE + 3) + i] = 0;
        dataMatrix[(MATRIX_SIZE + 4) * i] = 0;
        dataMatrix[(MATRIX_SIZE + 4) * i + (MATRIX_SIZE + 3)] = 0;
    }
    // draw the feature of data matrix
    for(int i = 1; i < MATRIX_SIZE + 3; i++) {
        dataMatrix[(MATRIX_SIZE + 4) * i+1] = 1;
        dataMatrix[(MATRIX_SIZE + 4) * (MATRIX_SIZE + 2) + i] = 1;
        dataMatrix[(MATRIX_SIZE + 4)+i] = i % 2;
        dataMatrix[(MATRIX_SIZE + 4) * i + (MATRIX_SIZE + 2)] = (i + 1) % 2;
    }
    // Move the valid part of the expanded matrix into the white blank
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix[(MATRIX_SIZE + 4) * (y + 2) + (x + 2)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * y + x];

#ifdef DEBUG_ENCODE
    cout << "The generated data matrix with black boarders and white blank: " << endl;
    for(int y = 0; y < MATRIX_SIZE + 4; y++) {
        for(int x = 0; x < MATRIX_SIZE + 4; x++)
            cout << (int)dataMatrix[(MATRIX_SIZE + 4) * y  + x] << " ";
        cout << endl;
    }
    cout << endl;
#endif

#else // without white blank between 2D code and black boarder
    // draw the feature of data matrix
    for(int i = 0; i < MATRIX_SIZE + 2; i++) {
        dataMatrix[(MATRIX_SIZE + 2) * i] = 1;
        dataMatrix[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + i] = 1;
        dataMatrix[i] = (i + 1) % 2;
        dataMatrix[(MATRIX_SIZE + 2) * i + (MATRIX_SIZE + 1)] = i % 2;
    }
    // Move the valid part of the expanded matrix (M*M) into the black boarder
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix[(MATRIX_SIZE + 2) * (y + 1) + (x + 1)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * y + x];

#ifdef DEBUG_ENCODE
    cout << "The generated data matrix with black boarders: " << endl;
    for(int y = 0; y < MATRIX_SIZE + 2; y++) {
        for(int x = 0; x < MATRIX_SIZE + 2; x++)
            cout << (int)dataMatrix[(MATRIX_SIZE + 2) * y  + x] << " ";
        cout << endl;
    }
    cout << endl;
#endif

#endif
#else // without data matrix feature boarder, not encouraged
      // don't care about this part
    // Move the valid part of the expanded matrix (M*M) into a M*M matirx
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix[MATRIX_SIZE * y + x ] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * y + x];

#ifdef DEBUG_ENCODE
    cout << "The generated data matrix with black boarders: " << endl;
    for(int y = 0; y < MATRIX_SIZE; y++) {
        for(int x = 0; x < MATRIX_SIZE; x++)
            cout << (int)dataMatrix[MATRIX_SIZE * y  + x] << " ";
        cout << endl;
    }
    cout << endl;
#endif

#endif
}
