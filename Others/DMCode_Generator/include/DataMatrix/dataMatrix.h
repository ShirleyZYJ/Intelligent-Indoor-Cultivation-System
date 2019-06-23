#ifndef DATA_MATRIX_H_
#define DATA_MATRIX_H_

#define MATRIX_SIZE 10    // define the size of data matrix

#define MATRIX_SIZE_10 1 // set to 1 if size is 10, otherwise 0
#define MATRIX_SIZE_12 0 // set to 1 if size is 12, otherwise 0
                         // Notice: only one of them can be 1 or 0
#define BLACK_BOARDER 1 // set to 1 if a black board surrounding data matrix, otherwise 0
                        // Don't set it to 0, otherwise the 2D code has no Data Matrix Feature Boarder
#define WITH_WHITE_BLANK 1   // set to 1 if there is blank between the board and data matrix, otherwise 0

typedef unsigned char uchar;

const uchar binary_Weight[9] = {128, 64, 0, 32, 16, 8, 4, 2, 1};

// compile the following when size is 10
#if MATRIX_SIZE_10
const uchar CODE_SIZE = 12;
const uchar MESSAGE_SIZE = 5;
// the up-left coordinate of each code
const uchar dataHead[CODE_SIZE] = {
    // the up-left coordinate of each message code
    (MATRIX_SIZE + 2) * 0 + 8,
    (MATRIX_SIZE + 2) * 0 + 0,
    (MATRIX_SIZE + 2) * 8 + 0,
    (MATRIX_SIZE + 2) * 9 + 3,
    (MATRIX_SIZE + 2) * 1 + 3,
    // the up-left coordinate of each redundancy code
    (MATRIX_SIZE + 2) * 3 + 1,
    (MATRIX_SIZE + 2) * 3 + 9,
    (MATRIX_SIZE + 2) * 6 + 2,
    (MATRIX_SIZE + 2) * 4 + 4,
    (MATRIX_SIZE + 2) * 2 + 6,
    (MATRIX_SIZE + 2) * 5 + 7,
    (MATRIX_SIZE + 2) * 7 + 5
};


// compile the following when size is 12
#elif MATRIX_SIZE_12
const uchar CODE_SIZE = 18;
const uchar MESSAGE_SIZE = 8;
// the up-left coordinate of each code
const uchar dataHead[CODE_SIZE] = {
    // the up-left coordinate of each message code
    (MATRIX_SIZE + 2) * 6 + 10,
    (MATRIX_SIZE + 2) * 0 + 0,
    (MATRIX_SIZE + 2) * 10 + 6,
    (MATRIX_SIZE + 2) * 11 + 9,
    (MATRIX_SIZE + 2) * 1 + 3,
    (MATRIX_SIZE + 2) * 3 + 1,
    (MATRIX_SIZE + 2) * 9 + 11,
    (MATRIX_SIZE + 2) * 11 + 0,
    // the up-left coordinate of each redundancy code
    (MATRIX_SIZE + 2) * 8 + 0,
    (MATRIX_SIZE + 2) * 6 + 2,
    (MATRIX_SIZE + 2) * 4 + 4,
    (MATRIX_SIZE + 2) * 2 + 6,
    (MATRIX_SIZE + 2) * 0 + 8,
    (MATRIX_SIZE + 2) * 3 + 9,
    (MATRIX_SIZE + 2) * 5 + 7,
    (MATRIX_SIZE + 2) * 7 + 5,
    (MATRIX_SIZE + 2) * 9 + 3,
    (MATRIX_SIZE + 2) * 8 + 8
};
#endif

#endif
