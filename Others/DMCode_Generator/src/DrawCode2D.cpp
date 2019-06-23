#include <iostream>
#include "DrawCode2D.h"
#include <string>

using namespace cv;

// Function: draw a unit of the 2D code 
// Input variable:
//     x: x-coordinate of the upper-left corner
//     y: y-coordinate of the upper-left corner
//     size: width of the unit
//     color: color of the unit
//            black(0) when color = 1
//            white(255) when color = 0
void DrawSqure(Mat img, unsigned char x, unsigned char y, int size, unsigned char color) {
    rectangle( img,
               Point(size * x, size * y),
               Point( size * (x + 1) - 1, size * (y + 1) - 1),
               Scalar( color ? 0 : 255 ),
               -1,
               1 );
}

void DrawCode2D(const unsigned char* matrix, int dimension) {

// Windows names
    char Code2D_window[] = "Data Matrix Code";
    int w = 20;
    int margin = 6;
    int boarder_size = dimension + 2;
    int total_size = boarder_size + 2 * margin;
    Mat Code2D_image = Mat::zeros(total_size * w, total_size * w, CV_8UC1 );
    rectangle(Code2D_image, Point(0, 0), Point(total_size * w, total_size * w), Scalar(255), -1, 1);

    // draw black boarder
    rectangle(Code2D_image, Point(margin * w, margin * w),
              Point((total_size - margin)* w, (total_size - margin)* w),
              Scalar(0), -1, 1);
    rectangle(Code2D_image, Point((margin + 1) * w, (margin + 1) * w),
              Point((total_size - margin - 1)* w - 1, (total_size - margin - 1)* w - 1),
              Scalar(255), -1, 1);
    // draw data matrix code
    for(int y = 0; y < dimension; y++)
        for(int x = 0; x < dimension; x++)
            DrawSqure(Code2D_image, x + margin + 1, y + margin + 1, w, matrix[dimension * y + x]);
    imshow( Code2D_window, Code2D_image );
    moveWindow( Code2D_window, 0, 0 );
}

void DrawCode2D_5X2(const unsigned char* matrix, int dimension, int page) {

// Windows names
    char Code2D_window[] = "Data Matrix Code";
    int w = 10; // pixel numbers of a single unit's edge
    int margin = 6; // unit numbers between every two codes
    int boarder_size = dimension + 2; // unit numbers of one code's black boarder 
    int x_size = boarder_size*2+margin*7; 
    int y_size = boarder_size*5 + margin*6;
    Mat Code2D_image = Mat::zeros(y_size * w, x_size * w, CV_8UC1 );
    rectangle(Code2D_image, Point(0, 0), Point(x_size * w, y_size * w), Scalar(255), -1, 1); // make the whole page white
    int code_headX[10]={};
    int code_headY[10]={};
    for(int i =0;i<10;++i){
        code_headX[i]=margin*3+(boarder_size+margin)*(i/5);
        code_headY[i]=margin+(boarder_size+margin)*(i%5);
    }
    // draw horizontal lines
    for(int i=0;i<5;i++)
        line(Code2D_image,
             Point(0,(code_headY[i]-margin/2)*w), 
             Point(x_size*w,(code_headY[i]-margin/2)*w),
             Scalar(0),2,1);
    line(Code2D_image,
         Point(0,(code_headY[4]+boarder_size+margin/2)*w), 
         Point(x_size*w,(code_headY[4]+boarder_size+margin/2)*w),
         Scalar(0),2,1);
    // draw vertical lines
    line(Code2D_image,
         Point((code_headX[0]-margin/2)*w,0), 
         Point((code_headX[0]-margin/2)*w,y_size*w),
         Scalar(0),2,1);
    line(Code2D_image,
         Point((code_headX[5]-margin/2)*w,0), 
         Point((code_headX[5]-margin/2)*w,y_size*w),
         Scalar(0),2,1);
    line(Code2D_image,
         Point((code_headX[5]+boarder_size+margin/2)*w,0), 
         Point((code_headX[5]+boarder_size+margin/2)*w,y_size*w),
         Scalar(0),2,1);

    for(int i =0;i<10;++i) {
        // draw black boarder
        rectangle(Code2D_image, Point(code_headX[i] * w, code_headY[i] * w),
              Point((code_headX[i]+boarder_size)* w-1, (code_headY[i]+boarder_size)* w-1),
              Scalar(0), -1, 1);
        rectangle(Code2D_image, Point((code_headX[i] + 1) * w, (code_headY[i] + 1) * w),
              Point((code_headX[i]+boarder_size-1) * w - 1, (code_headY[i]+boarder_size-1)* w - 1),
              Scalar(255), -1, 1);
        // draw data matrix code
        for(int y = 0; y < dimension; y++)
            for(int x = 0; x < dimension; x++)
                DrawSqure(Code2D_image, code_headX[i] + x+1, code_headY[i] + y+1, w, matrix[dimension*dimension*i+dimension * y + x]);
    }
    std::string image_name = "images/Code2D_" + std::to_string(page) + ".jpg";
    imwrite( image_name, Code2D_image );
    //imshow( Code2D_window, Code2D_image );
    //moveWindow( Code2D_window, 0, 0 );
}

// Function: draw 5X4 = 20 2D-codes in one page
// Input variable:
//     matrix: head address of the first 2D code
//     dimension: dimension of a 2D code
//     page: page number of the current page
void DrawCode2D_5X4(const unsigned char* matrix, int dimension, int page) {

    char Code2D_window[] = "Data Matrix Code"; // Windows names
    int w = 10; // pixel numbers of a single unit's edge
    int margin = 4; // unit numbers between every two codes
    int boarder_size = dimension + 2; // unit numbers of one code's black boarder
    int x_size = boarder_size*4+margin*5;
    int y_size = boarder_size*5 + margin*6;
    Mat Code2D_image = Mat::zeros(y_size * w, x_size * w, CV_8UC1 );
    rectangle(Code2D_image, Point(0, 0), Point(x_size * w, y_size * w), Scalar(255), -1, 1); // make the whole page white
    int code_headX[20]={}; // store the x coordinate each code's upper left corner
    int code_headY[20]={}; // store the y coordinate each code's upper left corner
    for(int i =0;i<20;++i){
        code_headX[i]=margin+(boarder_size+margin)*(i/5);
        code_headY[i]=margin+(boarder_size+margin)*(i%5);
    }
    // Draw lines outside every codes to make sticking work easy
    // NOTICE: To meet the requirement of a printing shop from taobao, 
    //         I remove the lines between codes but only leave the marks
    //         near the page edges.
    // draw horizontal lines
    for(int i=0;i<5;i++){
        line(Code2D_image,
             Point(0,(code_headY[i]-margin/2)*w), 
             Point(margin/2*w,(code_headY[i]-margin/2)*w),
             Scalar(0),2,1);
        line(Code2D_image,
             Point((x_size-margin/2)*w,(code_headY[i]-margin/2)*w), 
             Point(x_size*w,(code_headY[i]-margin/2)*w),
             Scalar(0),2,1);
    }
    line(Code2D_image,
         Point(0,(code_headY[4]+boarder_size+margin/2)*w), 
         Point(margin/2*w,(code_headY[4]+boarder_size+margin/2)*w),
         Scalar(0),2,1);
    line(Code2D_image,
         Point((x_size-margin/2)*w,(code_headY[4]+boarder_size+margin/2)*w), 
         Point(x_size*w,(code_headY[4]+boarder_size+margin/2)*w),
         Scalar(0),2,1);
    // draw vertical lines
    for(int i=0; i<4; i++) {
        line(Code2D_image,
            Point((code_headX[5*i]-margin/2)*w,0), 
            Point((code_headX[5*i]-margin/2)*w,margin/2*w),
            Scalar(0),2,1);
        line(Code2D_image,
            Point((code_headX[5*i]-margin/2)*w,(y_size-margin/2)*w), 
            Point((code_headX[5*i]-margin/2)*w,y_size*w),
            Scalar(0),2,1);
    }
    line(Code2D_image,
         Point((code_headX[15]+boarder_size+margin/2)*w,0), 
         Point((code_headX[15]+boarder_size+margin/2)*w,margin/2*w),
         Scalar(0),2,1);
    line(Code2D_image,
         Point((code_headX[15]+boarder_size+margin/2)*w,(y_size-margin/2)*w), 
         Point((code_headX[15]+boarder_size+margin/2)*w,y_size*w),
         Scalar(0),2,1);

    // draw the 2D codes one by one based on the upper-left corner coordinate
    for(int i =0;i<20;++i) {
        // draw black boarder
        rectangle(Code2D_image, Point(code_headX[i] * w, code_headY[i] * w),
              Point((code_headX[i]+boarder_size)* w-1, (code_headY[i]+boarder_size)* w-1),
              Scalar(0), -1, 1);
        rectangle(Code2D_image, Point((code_headX[i] + 1) * w, (code_headY[i] + 1) * w),
              Point((code_headX[i]+boarder_size-1) * w - 1, (code_headY[i]+boarder_size-1)* w - 1),
              Scalar(255), -1, 1);
        // draw each bit of a 2D code
        for(int y = 0; y < dimension; y++)
            for(int x = 0; x < dimension; x++)
                DrawSqure(Code2D_image, code_headX[i] + x+1, code_headY[i] + y+1, w, matrix[dimension*dimension*i+dimension * y + x]);
    }
    // give the path and a name to the page
    std::string image_name = "images/Code2D_" + std::to_string(page) + ".jpg";
    // save the page
    imwrite( image_name, Code2D_image );
    //imshow( Code2D_window, Code2D_image );
    //moveWindow( Code2D_window, 0, 0 );
}