#include <iostream>
#include <sstream>
#include <vector>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;
//using namespace cv;

int MM = 8;
int NN = 255;
const int alphaTo[] = {
    1,   2,   4,   8,  16,  32,  64, 128,  45,  90, 180,  69, 138,  57, 114, 228,
    229, 231, 227, 235, 251, 219, 155,  27,  54, 108, 216, 157,  23,  46,  92, 184,
    93, 186,  89, 178,  73, 146,   9,  18,  36,  72, 144,  13,  26,  52, 104, 208,
    141,  55, 110, 220, 149,   7,  14,  28,  56, 112, 224, 237, 247, 195, 171, 123,
    246, 193, 175, 115, 230, 225, 239, 243, 203, 187,  91, 182,  65, 130,  41,  82,
    164, 101, 202, 185,  95, 190,  81, 162, 105, 210, 137,  63, 126, 252, 213, 135,
    35,  70, 140,  53, 106, 212, 133,  39,  78, 156,  21,  42,  84, 168, 125, 250,
    217, 159,  19,  38,  76, 152,  29,  58, 116, 232, 253, 215, 131,  43,  86, 172,
    117, 234, 249, 223, 147,  11,  22,  44,  88, 176,  77, 154,  25,  50, 100, 200,
    189,  87, 174, 113, 226, 233, 255, 211, 139,  59, 118, 236, 245, 199, 163, 107,
    214, 129,  47,  94, 188,  85, 170, 121, 242, 201, 191,  83, 166,  97, 194, 169,
    127, 254, 209, 143,  51, 102, 204, 181,  71, 142,  49,  98, 196, 165, 103, 206,
    177,  79, 158,  17,  34,  68, 136,  61, 122, 244, 197, 167,  99, 198, 161, 111,
    222, 145,  15,  30,  60, 120, 240, 205, 183,  67, 134,  33,  66, 132,  37,  74,
    148,   5,  10,  20,  40,  80, 160, 109, 218, 153,  31,  62, 124, 248, 221, 151,
    3,   6,  12,  24,  48,  96, 192, 173, 119, 238, 241, 207, 179,  75, 150,   0
};

const int expOf[] = {
    255,   0,   1, 240,   2, 225, 241,  53,   3,  38, 226, 133, 242,  43,  54, 210,
    4, 195,  39, 114, 227, 106, 134,  28, 243, 140,  44,  23,  55, 118, 211, 234,
    5, 219, 196,  96,  40, 222, 115, 103, 228,  78, 107, 125, 135,   8,  29, 162,
    244, 186, 141, 180,  45,  99,  24,  49,  56,  13, 119, 153, 212, 199, 235,  91,
    6,  76, 220, 217, 197,  11,  97, 184,  41,  36, 223, 253, 116, 138, 104, 193,
    229,  86,  79, 171, 108, 165, 126, 145, 136,  34,   9,  74,  30,  32, 163,  84,
    245, 173, 187, 204, 142,  81, 181, 190,  46,  88, 100, 159,  25, 231,  50, 207,
    57, 147,  14,  67, 120, 128, 154, 248, 213, 167, 200,  63, 236, 110,  92, 176,
    7, 161,  77, 124, 221, 102, 218,  95, 198,  90,  12, 152,  98,  48, 185, 179,
    42, 209,  37, 132, 224,  52, 254, 239, 117, 233, 139,  22, 105,  27, 194, 113,
    230, 206,  87, 158,  80, 189, 172, 203, 109, 175, 166,  62, 127, 247, 146,  66,
    137, 192,  35, 252,  10, 183,  75, 216,  31,  83,  33,  73, 164, 144,  85, 170,
    246,  65, 174,  61, 188, 202, 205, 157, 143, 169,  82,  72, 182, 215, 191, 251,
    47, 178,  89, 151, 101,  94, 160, 123,  26, 112, 232,  21,  51, 238, 208, 131,
    58,  69, 148,  18,  15,  16,  68,  17, 121, 149, 129,  19, 155,  59, 249,  70,
    214, 250, 168,  71, 201, 156,  64,  60, 237, 130, 111,  20,  93, 122, 177, 150
};
int GfAdd(int a, int b);
int GfMult(int a, int b);
int GfMult2(int a, int b);
int GfDiv(int a, int b);
int GfDiv2(int a, int b);

int main() {

    //double t;

    // encode
    int total = 18;
    int data = 8;
    int error = total - data;
    int* codes = new int[total];
    int* errors = new int[error];
    int* errors2 = new int[error];
    int* polys = new int[error * error];
    int* polys2 = new int[error * error];

    int i, j, k, d;
    codes[0] = 66;
    codes[1] = 67;
    codes[2] = 68;
    codes[3] = 69;
    codes[4] = 70;
    codes[5] = 142;
    codes[6] = 129;
    codes[7] = 56;


    //t = (double)getTickCount();


    for( i = 0; i < error; i++ ) {
        //sum of each polynomial
        int sum = 0;
        for( j = 0; j < data; j++ )
            sum = GfAdd( sum, GfMult2( codes[j], (total - 1 - j) * (i + 1) ) );
        errors[i] = sum;
        errors2[i] = sum;
        //polynomial matrix
        int index = i * error;
        for( j = 0; j < error; j++ ) {
            polys[index + j] = alphaTo[j * (i + 1)];
            polys2[index + j]=polys[index + j];
        }
    }

    //gaussion elimination
    for( i = 0; i < error; i++ ) {
        //diagonal postion
        d = i * error + i;
        int diagonal = polys[d];
        //diagonal --> 1
        int index = i * error;
        for( j = 0; j < error; j++ )
            polys[index + j] = GfDiv( polys[index + j], diagonal );
        errors[i] = GfDiv( errors[i], diagonal );
        //otherrows - thisrow
        for( k = 0; k < error; k++ ) {
            if( k != i ) { //another row
                int index2 = k * error;
                int coefficient = polys[index2 + i];
                //each column( polynomial[k] = polynomial[k] - polynomial[i]*coefficient )
                for( int m = 0; m < error; m++ ) {
                    polys[index2 + m] = GfAdd( polys[index2 + m], GfMult(coefficient, polys[index + m]) );
                }
                errors[k] = GfAdd( errors[k], GfMult(coefficient, errors[i]) );
            }
        }
    }

    for(i = 0; i < error; i++)
        cout << errors[i] << " ";
    cout << endl;
    //errors --> codes

    for(i = 0; i < error; i++)
        codes[data + i] = errors[error - 1 - i];

    // the codes pass the channel
    int* codes_RX = new int[total];
    for(i = 0; i < total; i++)
        codes_RX[total-1-i] = codes[i];

    for(i = 0; i < total; i++)
        cout << codes_RX[i] << " ";
    cout << endl;

    codes_RX[17] = 0;
    //codes_RX[16] = 66;
    //codes_RX[15] = 0;
    //codes_RX[14] = 0;
    //codes_RX[13] = 9;
    //codes_RX[12]=0;
    //codes_RX[11]=0;

    for(i = 0; i < total; i++)
        cout << codes_RX[i] << " ";
    cout << endl;

    delete[] codes;
    delete[] errors;
    delete[] polys;

    // decode
    // generate syndromes
    int* syndromes = new int[error];

    for(i = 0; i < error; i++) {
        syndromes[i] = 0;
        for(j = 0; j < total; j++)
            syndromes[i] = GfAdd(syndromes[i], GfMult2(codes_RX[j], (i + 1) * j));
    }
    for(i = 0; i < error; i++)
        cout << syndromes[i] << " ";
    cout << endl;

    // obtain error position polynomials sigma(x)
    int errorMax;
    errorMax = error/2 ;
    int* polynomial = new int[errorMax * errorMax];
    int* right = new int[errorMax];
    for(i = 0; i < errorMax; i++) {
        right[i] = syndromes[errorMax + i];
        //right[i] = errors2[i];
        for(j = 0; j < errorMax; j++)
            polynomial[errorMax * i + j] = syndromes[errorMax - 1 + i - j];
            //polynomial[errorMax * i + j] = polys2[errorMax * i + j];
    }
    for(i = 0; i < errorMax; i++) {
        for(j = 0; j < errorMax; j++)
            cout << polynomial[errorMax * i + j] << " ";
        cout << " " << right[i] << endl;
    }

    //gaussion elimination
/*    for( i = 0; i < errorMax; i++ ) {
        //diagonal postion
        int diag = polynomial[i * errorMax + i];
        //diagonal --> 1
        for( j = 0; j < errorMax; j++ )
            polynomial[errorMax * i + j] = GfDiv( polynomial[errorMax * i + j], diag );
        right[i] = GfDiv( right[i], diag );
        //otherrows - thisrow
        for( k = 0; k < errorMax; k++ ) {
            if( k != i ) { //another row
                int coeff = polynomial[k * errorMax + i];
                //each column( polynomial[k] = polynomial[k] - polynomial[i]*coefficient )
                for( int mm = 0; mm < errorMax; mm++ ) {
                    polynomial[k * errorMax + mm] = GfAdd( polynomial[k * errorMax + mm], GfMult(coeff, polynomial[k * errorMax + mm]) );
                }
                right[k] = GfAdd( right[k], GfMult(coeff, right[i]) );
            }
        }
    }
*/
    for( i = 0; i < errorMax; i++ ) {
        //diagonal postion
        d = i * errorMax + i;
        int diagonal = polynomial[d];
        //diagonal --> 1
        int index = i * errorMax;
        for( j = 0; j < errorMax; j++ )
            polynomial[index + j] = GfDiv( polynomial[index + j], diagonal );
        right[i] = GfDiv( right[i], diagonal );
        //otherrows - thisrow
        for( k = 0; k < errorMax; k++ ) {
            if( k != i ) { //another row
                int index2 = k * errorMax;
                int coefficient = polynomial[index2 + i];
                //each column( polynomial[k] = polynomial[k] - polynomial[i]*coefficient )
                for( int m = 0; m < errorMax; m++ ) {
                    polynomial[index2 + m] = GfAdd( polynomial[index2 + m], GfMult(coefficient, polynomial[index + m]) );
                }
                right[k] = GfAdd( right[k], GfMult(coefficient, right[i]) );
            }
        }
    }

    for(i = 0; i < errorMax; i++)
        cout << right[i] << " ";
    cout << endl;
    // determine the positions of error
    vector<int> errorPosition;
    int sigma;
    for(i = 0; i < total; i++) {
        sigma = 1;
        for(j = 0; j < errorMax; j++)
            sigma = GfAdd(sigma, GfDiv2(right[j], i * (j + 1)));
        cout << sigma << " ";
        if(sigma == 0)
            errorPosition.insert(errorPosition.end(),i);
    }

    cout << endl;

    int errorNum=errorPosition.size();

    cout<<errorNum<<endl;

    vector<int>::iterator iter;
    for(iter = errorPosition.begin(); iter != errorPosition.end(); iter++)
        cout << *iter << " ";
    cout << endl;

    // determine the error values and modify them
    int* poly3 = new int[errorNum * errorNum];
    int* right3 = new int[errorNum];
    
    for( i = 0; i < errorNum; i++ ) {
        //sum of each polynomial
        int sum = 0;
        for( j = 0; j < total; j++ )
            sum = GfAdd( sum, GfMult2( codes_RX[j], j * (i + 1) ) );
        for(iter = errorPosition.begin(); iter != errorPosition.end(); iter++)
            sum = GfAdd( sum, GfMult2( codes_RX[*iter], (*iter) * (i + 1) ));
        right3[i] = sum;
        //polynomial matrix
        int index = i * errorNum;
        for( j = 0; j < errorNum; j++ ) {
            poly3[index + j] = alphaTo[errorPosition[j] * (i + 1)];
        }
    }

    for(i = 0; i < errorNum; i++) {
        for(j = 0; j < errorNum; j++)
            cout << poly3[errorNum * i + j] << " ";
        cout << " " << right3[i] << endl;
    }

    //gaussion elimination
    for( i = 0; i < errorNum; i++ ) {
        //diagonal postion
        d = i * errorNum + i;
        int diagonal = poly3[d];
        //diagonal --> 1
        int index = i * errorNum;
        for( j = 0; j < errorNum; j++ )
            poly3[index + j] = GfDiv( poly3[index + j], diagonal );
        right3[i] = GfDiv( right3[i], diagonal );
        //otherrows - thisrow
        for( k = 0; k < errorNum; k++ ) {
            if( k != i ) { //another row
                int index2 = k * errorNum;
                int coefficient = poly3[index2 + i];
                //each column( polynomial[k] = polynomial[k] - polynomial[i]*coefficient )
                for( int m = 0; m < errorNum; m++ ) {
                    poly3[index2 + m] = GfAdd( poly3[index2 + m], GfMult(coefficient, poly3[index + m]) );
                }
                right3[k] = GfAdd( right3[k], GfMult(coefficient, right3[i]) );
            }
        }
    }

    for(i = 0; i < errorNum; i++)
        cout << right3[i] << " ";
    cout << endl;

    //t = 1000 * ((double)getTickCount() - t) / getTickFrequency();
    //cout  << t << " milliseconds." << endl;

    return 0;
}
int GfAdd(int a, int b) {
    return  a ^ b;
}
//a * b
int GfMult(int a, int b) {
    return (a == 0 || b == 0) ? 0 : alphaTo[(expOf[a] + expOf[b]) % NN];
}
// a * alpha^b
int GfMult2(int a, int b) {
    return a == 0 ? 0 : alphaTo[(expOf[a] + b) % NN];
}
// a/b
int GfDiv(int a, int b) {
    if( a == 0 )return 0;
    if( a == b )return 1;
    return expOf[a] > expOf[b] ?
           alphaTo[ expOf[a] - expOf[b] ] : alphaTo[ NN + expOf[a] - expOf[b] ] ;
}
// a/alhpa^b
int GfDiv2(int a, int b) {
    if( a == 0 )return 0;
    if( expOf[a] == b )return 1;
    return expOf[a] > b ?
           alphaTo[ expOf[a] - b] : alphaTo[ NN + expOf[a] - b] ;
}
