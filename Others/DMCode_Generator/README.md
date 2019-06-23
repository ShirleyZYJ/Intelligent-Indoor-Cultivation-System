# README #

This block is used to encode 10-digit or 8-digit id to 10X10 or 12X12 (excluding the feature boarder) Data Matrix code and generate the codes saved as .jpg format. In current version, there are 20 codes having the same x-coordinate but increasing y-coordinate (0000-0019)in one A4 page. Different pages have the increasing y-coordinates (0000,0001,0002,0003,...).

1) This block also includes dataMatrix.h (directly included in dataMatrixEncoder.h). However, this head file has different path from that in Data Matrix detector. ALthough it is troublesome, you MUST remember to keep the two dataMatrix.h in the two blocks completely identical when you want to change the 2D code's size, otherwise the generated Data Matrix Codes may not be detected by the detector.

2) There are also two copies of ReedSolomon.h in the two blocks, respectively. They should be surely identical,too. Just don't bother to modify them, unless you are going to do further development.  

3) In the source file DrawCode2D.cpp, there are three functions for drawing Data Matrix codes, which draw 1, 10 and 20 codes in one page respectively. Since drawing 20 codes makes the best use of a page, it is applied in CodeGenerator.cpp. Please ignore the other two functions if you have no special requirement. Besides, you can take the function DrawCode2D_5X4() as a prototype if you want to do further development.

4) In the source file CodeGenerator.cpp, don't change the value of NrPerPage if you still apply the function DrawCode2D_5X4(). The value of page can be changed depending on how many A4 pages (or in other words, how many x-coordinates) you need.

5) Currently, only 10X10 (excluding the feature boarder) Data Matrix code is used, so the code generator doesn't support generating 12X12 codes. For this reason, keep the value of id_Size to be 10. If you want to develop 12X12 code generation, only modify the contents of Step 2 and Step 3 and change is_Size to 8. 

6) If you stick the Data Matrix codes to different objects, like floor, storage rack and goods, you can change the content of Step 2.

7) The Data Matrix Decoder and Reed Solomon Decoder are also contained in this block. They are used to test the Encoding in the early developing phase. Just ignore them. They haven't been updated for some time. 

8) The generated images (A4 pages) will be stored in the folder ./image. You can change the code at around line 189 in DrawCode2D.cpp if you want to change the storage path or even the image name.
