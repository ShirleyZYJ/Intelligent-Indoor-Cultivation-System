#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "dataMatrixDecoder.h"
#include "dataMatrixEncoder.h"
#include "DrawCode2D.h"
using namespace std;
using namespace cv;

const int NrPerPage = 20;
const int page = 20;
const int id_Nr = NrPerPage * page; // the number of id's
const int id_Size = 10; // 10 bits for 10*10 Data Matrix, 
                         // 8  bits for 12*12 Data Matrix
const int code2D_Size = MATRIX_SIZE+2*BLACK_BOARDER+2*WITH_WHITE_BLANK;

int main() {
    
    // Step 1: generate id_Nr 10-bit codes  
    char id[id_Nr*id_Size] = {};
    // Step 2: first 2 bits: characteristic code (cannot be larger than 20!)
    for(int i=0;i<id_Nr;++i) {
        id[id_Size*i+0]='1';
        id[id_Size*i+1]='0';
    }
    // Step 3: 3rd-10th bits: x- and y-coordinates
    for(int i=0;i<id_Nr;++i) {
        id[id_Size*i+2]=char(i/NrPerPage/1000+int('0'));
        id[id_Size*i+3]=char(i/NrPerPage/100%10+int('0'));
        id[id_Size*i+4]=char(i/NrPerPage/10%10+int('0'));
        id[id_Size*i+5]=char(i/NrPerPage%10+int('0'));
        id[id_Size*i+6]=char(i%NrPerPage/1000+int('0'));
        id[id_Size*i+7]=char(i%NrPerPage/100%10+int('0'));
        id[id_Size*i+8]=char(i%NrPerPage/10%10+int('0'));
        id[id_Size*i+9]=char(i%NrPerPage%10+int('0'));
    }
    // display all the id's
    for(int i=0;i<id_Nr;++i){
        for(int j=0;j<id_Size;++j)
            cout<<id[id_Size*i+j];
        cout<<endl;
    }

    uchar code2D[id_Nr*code2D_Size * code2D_Size];
    // Step 4: encode each 2D code
    for(int i=0;i<id_Nr;++i)
        dataMatrixEncoder(code2D+code2D_Size * code2D_Size*i, id+id_Size*i);
    // Step 5: draw codes in 20 pages, each page has 20 codes
    for(int page; page<20;++page)
        DrawCode2D_5X4(code2D+code2D_Size * code2D_Size*20*page,code2D_Size,page);

    waitKey( 0 );

    return 0;
}
