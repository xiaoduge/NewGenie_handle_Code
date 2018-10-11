#ifndef _STM32_BMP_H_
#define _STM32_BMP_H_

#include "DtypeStm32.h"

#pragma   pack(1)


typedef struct BITMAPFILEHEADER  
{   
    u16 bfType;   
    u32 bfSize;   
    u16 bfReserved1;   
    u16 bfReserved2;   
    u32 bfOffBits;   
}BITMAPFILEHEADER;   
  
typedef struct BITMAPINFOHEADER  
{   
    u32 biSize;   
    u32 biWidth;   
    u32 biHeight;   
    u16 biPlanes;   
    u16 biBitCount;   
    u32 biCompression;   
    u32 biSizeImage;   
    u32 biXPelsPerMeter;   
    u32 biYPelsPerMeter;   
    u32 biClrUsed;   
    u32 biClrImportant;   
}BITMAPINFODEADER;  

// pallete
typedef  struct RGBQUAD
{
    u8 rgbBlue ;   
    u8 rgbGreen ;   
    u8 rgbRed ;     
    u8 rgbReserved ;
}RGBQUAD;

typedef  struct BmpSt
{
    BITMAPFILEHEADER head;
    BITMAPINFODEADER info;
    RGBQUAD rgb[1];
}BmpSt;

typedef void (*draw_pixel_callback)(u16 x,u16 y,u16 color);   

void DrawBitmap(const uint8_t *bmpData,int x,int y,draw_pixel_callback cb);

#pragma pack()

#endif


