#include "ReedSolomonDecoder.h"
#include <vector>
//#define DEBUG_RS
using namespace std;

// Reed-Solomon Decoder
bool ReedSolomonDecoder(uchar *code, uchar code_Size, uchar message_Size) {

    uchar correction_Size = code_Size - message_Size;
    uchar code_Reverse[code_Size];

    // Step 0: Reverse the received codes
#ifdef DEBUG_RS
    cout << "Step 0:" << endl;
    cout << "\t" << "The received " << (int)code_Size << "-bit codes: " << endl;
    cout << "\t";
    for(uchar i = 0; i < code_Size; i++)
        cout << (int)code[i] << " ";
    cout << endl << endl;
#endif

    for(uchar i = 0; i < code_Size; i++)
        code_Reverse[i] = code[code_Size - 1 - i];

#ifdef DEBUG_RS
    cout << "\t" << "The Reversed " << (int)code_Size << "-bit codes: " << endl;
    cout << "\t";
    for(uchar i = 0; i < code_Size; i++)
        cout << (int)code_Reverse[i] << " ";
    cout << endl << endl;
#endif

    // Step 1: Generate syndromes
    uchar syndrome[correction_Size];
    uchar error_Flag = 0; // set to 1 if error detected

    for(uchar i = 0; i < correction_Size; i++) {
        syndrome[i] = 0;
        for(uchar j = 0; j < code_Size; j++)
            syndrome[i] = GfAdd(syndrome[i], GfMult(code_Reverse[j], GfValueOf[(i + 1) * j]));
        error_Flag = syndrome[i] ? 1 : 0;
    }

#ifdef DEBUG_RS
    cout << "Step 1:" << endl;
    cout << "\t" << "The generated " << (int)correction_Size << "-bit syndromes: " << endl;
    cout << "\t";
    for(uchar i = 0; i < correction_Size; i++)
        cout << (int)syndrome[i] << " ";
    cout << endl << endl;
#endif

    if(!error_Flag) {
#ifdef DEBUG_RS
        cout << "No error detected!" << endl;
#endif
        return true;
    }

    // Step 2: Obtain error position polynomials Sigma(x)
    uchar error_Max = correction_Size / 2 ;
    uchar poly_Sigma[error_Max * error_Max];
    uchar sigma[error_Max];

    for(uchar i = 0; i < error_Max; i++) {
        sigma[i] = syndrome[error_Max + i];
        for(uchar j = 0; j < error_Max; j++)
            poly_Sigma[error_Max * i + j] = syndrome[error_Max - 1 + i - j];
    }
    GausEliminate(poly_Sigma, sigma, error_Max);

#ifdef DEBUG_RS
    cout << "Step 2:" << endl;
    cout << "\t" << "The generated " << (int)error_Max << "-bit sums of sigma: " << endl;
    cout << "\t";
    for(uchar i = 0; i < error_Max; i++)
        cout << (int)sigma[i] << " ";
    cout << endl << endl;
#endif

    // Step 3: Determine the positions of error
    vector<uchar> errorPosition;

    for(uchar i = 0; i < code_Size; i++) {
        uchar sigma_Sum = 1;
        for(uchar j = 0; j < error_Max; j++)
            sigma_Sum = GfAdd(sigma_Sum, GfDiv(sigma[j], GfValueOf[i * (j + 1)]));
        if(sigma_Sum == 0)
            errorPosition.insert(errorPosition.end(), i);
    }

    uchar error_Num = errorPosition.size();

    if(!error_Num) {
#ifdef DEBUG_RS
        cout << "Sorry, more than " << int(error_Max) << " errors cannot be corrected!" << endl;
#endif
        return false;
    }

#ifdef DEBUG_RS
    cout << "Step 3:" << endl;
    cout << "\t" << "The number of errors: " << (int)error_Num << endl;
    cout << "\t" << "The positions of errors: " ;
    for(uchar i = 0; i < error_Num; i++)
        cout << code_Size - 1 - (int)errorPosition[error_Num - 1 - i] << " ";
    cout << endl << endl;
#endif
    // Step 4: determine the error values and modify them

    uchar poly_Error[error_Num * error_Num];
    uchar error[error_Num];

    for(uchar i = 0; i < error_Num; i++ ) {
        //sum of each polynomial
        uchar sum = 0;
        for(uchar j = 0; j < code_Size; j++ )
            sum = GfAdd( sum, GfMult( code_Reverse[j], GfValueOf[j * (i + 1)] ) );
        for(uchar j = 0 ; j < error_Num; j++)
            sum = GfAdd( sum, GfMult( code_Reverse[errorPosition[j]], GfValueOf[errorPosition[j] * (i + 1)] ));
        error[i] = sum;
        //polynomial matrix
        uchar current_Row = i * error_Num;
        for( uchar j = 0; j < error_Num; j++ ) {
            poly_Error[current_Row + j] = GfValueOf[errorPosition[j] * (i + 1)];
        }
    }

    GausEliminate(poly_Error, error, error_Num);

#ifdef DEBUG_RS
    cout << "Step 4:" << endl;
    cout << "\t" << "The correct values of errors:" << endl;
    for(uchar i = 0; i < error_Num; i++)
        cout << "\t" << code_Size - 1 - (int)errorPosition[error_Num - 1 - i] << ": " << (int)error[error_Num - 1 - i] << " ";
    cout << endl << endl;
#endif

    for(uchar i = 0; i < error_Num; i++)
        code_Reverse[errorPosition[i]] = error[i];

#ifdef DEBUG_RS
    cout << "\t" << "The recovered " << (int)code_Size << "-bit codes (reversed): " << endl;
    cout << "\t";
    for(uchar i = 0; i < code_Size; i++)
        cout << (int)code_Reverse[i] << " ";
    cout << endl << endl;
#endif

    for(uchar i = 0; i < code_Size; i++)
        code[i] = code_Reverse[code_Size - 1 - i];

#ifdef DEBUG_RS
    cout << "\t" << "The recovered " << (int)code_Size << "-bit codes: " << endl;
    cout << "\t";
    for(uchar i = 0; i < code_Size; i++)
        cout << (int)code[i] << " ";
    cout << endl << endl;
#endif

    return true;
}
