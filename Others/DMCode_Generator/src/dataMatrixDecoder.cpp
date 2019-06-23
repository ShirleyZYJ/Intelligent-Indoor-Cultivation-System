#include <iostream>
#include <string>
#include "dataMatrixDecoder.h"
using std::cout;
using std::endl;

#define ERROR_TOLARANCE 10 // %
#define DEBUG_DECODE_RESULT
//#define DEBUG_DECODE_DETAIL

uchar byteDecoder(uchar dataMatrix_Expanded[], uchar byteHead) {
    uchar byteValue = 0;
    for(uchar y = 0; y < 3; y++)
        for(uchar x = 0; x < 3; x++)
            byteValue += binary_Weight[y * 3 + x] * dataMatrix_Expanded[byteHead + (MATRIX_SIZE + 2) * y + x];
    return byteValue;
}

long long dataMatrixDecoder(uchar* dataMatrix_Detected) {

    uchar dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 2)] = {};
    uchar code[CODE_SIZE] = {};

#if BLACK_BOARDER
    int err_cnt;
#if WITH_WHITE_BLANK

    // detect the surrounding white blank
    err_cnt=0;
    for(int i = 0; i < MATRIX_SIZE + 4; i++) {
        char up_is_black = dataMatrix_Detected[i];
        char down_is_black = dataMatrix_Detected[(MATRIX_SIZE + 4) * (MATRIX_SIZE + 3) + i];
        char left_is_black = dataMatrix_Detected[(MATRIX_SIZE + 4) * i];
        char right_is_black = dataMatrix_Detected[(MATRIX_SIZE + 4) * i + (MATRIX_SIZE + 3)];
        err_cnt += up_is_black+down_is_black+left_is_black+right_is_black;
    }
    if(100*err_cnt>ERROR_TOLARANCE*(MATRIX_SIZE+4)*4)
        return -1;
    // detect the feature of data matrix
    err_cnt=0;
    for(int i = 1; i < MATRIX_SIZE + 3; i++) {
        bool left_is_black = dataMatrix_Detected[(MATRIX_SIZE + 4) * i + 1];
        bool down_is_black = dataMatrix_Detected[(MATRIX_SIZE + 4) * (MATRIX_SIZE + 2) + i];
        bool up_is_feature = (dataMatrix_Detected[(MATRIX_SIZE + 4) + i] == i % 2);
        bool right_is_feature = (dataMatrix_Detected[(MATRIX_SIZE + 4) * i + (MATRIX_SIZE + 2)] == (i + 1) % 2);
        err_cnt += !up_is_feature+!down_is_black+!left_is_black+!right_is_feature;
    }
    if(100*err_cnt>ERROR_TOLARANCE*(MATRIX_SIZE+2)*4)
        return -2;
    // Move the detected data matrix (M*M) to the expanded matrix ((M+2)*(M+2))
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix_Expanded[(MATRIX_SIZE + 2)*y + x] = dataMatrix_Detected[(MATRIX_SIZE + 4) * (y + 2) + (x + 2)];

#else
    // detect the feature of data matrix
    err_cnt=0;
    for(int i = 0; i < MATRIX_SIZE + 2; i++) {
        bool left_is_black = dataMatrix_Detected[(MATRIX_SIZE + 2) * i];
        bool down_is_black = dataMatrix_Detected[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + i];
        bool up_is_feature = (dataMatrix_Detected[i] == (i + 1) % 2);
        bool right_is_feature = (dataMatrix_Detected[(MATRIX_SIZE + 2) * i + (MATRIX_SIZE + 1)] == i % 2);
        err_cnt += !up_is_feature+!down_is_black+!left_is_black+!right_is_feature;
    }
    if(100*err_cnt>ERROR_TOLARANCE*(MATRIX_SIZE+2)*4)
        return -3;
    // Move the detected data matrix (M*M) to the expanded matrix ((M+2)*(M+2))
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix_Expanded[(MATRIX_SIZE + 2)*y + x] = dataMatrix_Detected[(MATRIX_SIZE + 2) * (y + 1) + (x + 1)];

#ifdef DEBUG_DECODE_DETAIL
    cout << "The detected data matrix with black boarders: " << endl;
    for(int y = 0; y < MATRIX_SIZE + 2; y++) {
        for(int x = 0; x < MATRIX_SIZE + 2; x++)
            cout << (int)dataMatrix_Expanded[(MATRIX_SIZE + 2) * y  + x] << " ";
        cout << endl;
    }
    cout << endl;
#endif

#endif
#else
    // Move the detected data matrix (M*M) to the expanded matrix ((M+2)*(M+2))
    for(uchar y = 0; y < MATRIX_SIZE; y++)
        for(uchar x = 0; x < MATRIX_SIZE; x++)
            dataMatrix_Expanded[(MATRIX_SIZE + 2)*y + x] = dataMatrix_Detected[MATRIX_SIZE * y + x];
#endif

#if MATRIX_SIZE_10
    // Complete the patial data and redundancies on the edges

    // Complete message code 1
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 2 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + 0];

    // Complete message code 3
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 2];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 3];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 4];

    // Complete message code 4
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 3] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 5];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 4] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 6];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 7];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 3] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 5];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 4] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 6];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 5] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 7];

    // Complete Redundancy 2
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + (MATRIX_SIZE + 1)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 1];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + (MATRIX_SIZE + 1)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 1];

#elif MATRIX_SIZE_12
    // Complete the patial data on the edges

    // Complete message code 1
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 8 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 4 + 0];

    // Complete message code 3
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 6] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 2];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 7] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 3];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 8] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 4];

    // Complete message code 4
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 9] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 5];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 10] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 6];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 7];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 9] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 5];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 10] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 6];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 11] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 7];

    // Complete message code 7
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 9 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 5 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 10 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + MATRIX_SIZE] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 0];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 10 + (MATRIX_SIZE + 1)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 6 + 1];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + (MATRIX_SIZE + 1)] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 7 + 1];

    // Complete message code 8
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 11 + 2];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 10];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * MATRIX_SIZE + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 0 + 11];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 0] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 1 + 11];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 1] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 2 + 11];
    dataMatrix_Expanded[(MATRIX_SIZE + 2) * (MATRIX_SIZE + 1) + 2] = dataMatrix_Expanded[(MATRIX_SIZE + 2) * 3 + 11];

#endif
    // Decode each byte
    for(uchar index = 0; index < CODE_SIZE; index++) {
        code[index] = byteDecoder(dataMatrix_Expanded, dataHead[index]);
        //cout << (int)data[index] << " ";
    }

#ifdef DEBUG_DECODE_DETAIL
    cout << "The completed data matrix with black boarders: " << endl;
    for(int y = 0; y < MATRIX_SIZE + 2; y++) {
        for(int x = 0; x < MATRIX_SIZE + 2; x++)
            cout << (int)dataMatrix_Expanded[(MATRIX_SIZE + 2) * y  + x] << " ";
        cout << endl;
    }
    cout << endl;
#endif

    bool success_Flag;

    success_Flag = ReedSolomonDecoder(code, CODE_SIZE, MESSAGE_SIZE);

    if(success_Flag == false)
        return -4;

    long long data = 0;


#if MATRIX_SIZE_10
    if(code[0]<130||code[0]>150) // the highest 2 bits should be smaller than 21
        return -5;
    else
        data = code[0]-130;
    for(int i = 1 ; i < MESSAGE_SIZE; i++) {
        if(code[i] < 130 || code[i] > 229) {
            return -5;
        } else {
            data = 100 * data + code[i]-130;
        }
    }
#elif MATRIX_SIZE_12
    for(int i = 0 ; i < MESSAGE_SIZE; i++) {
        if(code[i] < (int)'0'+1 || code[i] > (int)'9'+1) {
            return -5;
        } else {
            data = 10 * data + code[i]-1-(int)'0';
        }
    }
#endif


#ifdef DEBUG_DECODE_RESULT
    static int counter=1;
    cout<<counter++<<": ";
    cout<< data<<endl;
#endif

    return data;

}
