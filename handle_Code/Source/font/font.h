#ifndef __FONT_H
#define __FONT_H		

typedef struct  typFNT_GB33                           // 汉字字模数据结构 
{
       unsigned char  Index1[2];               // 汉字内码索引	
       unsigned char   Msk1[66];               // 点阵码数据 
}FNT_GB33;

typedef struct typFNT_GB16                 // 汉字字模数据结构 
{
       unsigned char  Index[3];               // 汉字内码索引	
       unsigned char   Msk[32];               // 点阵码数据 
}FNT_GB16;

typedef struct typFNT_GB32                 // 汉字字模数据结构 
{
       unsigned char  Index[3];               // 汉字内码索引	
       unsigned char   Msk[128];              // 点阵码数据 
}FNT_GB32;

typedef struct  typFNT_GB24                  // 汉字字模数据结构 
{
       unsigned char  Index[3];               // 汉字内码索引	
       unsigned char   Msk[72];               // 点阵码数据 
}FNT_GB24;

typedef struct typFNT_GB48                 // 汉字字模数据结构 
{
       unsigned char  Index2[3];            // 汉字内码索引	
       unsigned char   Msk2[288];           // 点阵码数据 
}FNT_GB48;

typedef struct  typFNT_GB49                // 汉字字模数据结构 
{
       unsigned char  Index[3];               // 汉字内码索引	
       unsigned char   Msk[144];               // 点阵码数据 
}FNT_GB49;

typedef struct  typFNT_GB40                 // 汉字字模数据结构 
{
       unsigned char  Index[3];               // 汉字内码索引	
       unsigned char   Msk[120];               // 点阵码数据 
}FNT_GB40;



//extern unsigned char asc2_1608[1520];

extern unsigned char const ascii_8x16[1536];	
extern unsigned short const ASCII_12x24[]; 
extern unsigned char const Font24x48[];
extern unsigned char const Font16x33[];
extern unsigned char const ascii_24x48[];

extern struct  typFNT_GB33 const codeGB_33[];
extern struct  typFNT_GB32 const codeGB_32[] ;
extern struct  typFNT_GB24 const codeGB_24[];
extern struct  typFNT_GB48 const codeGB_48[];
extern struct  typFNT_GB49 const codeGB_49[];
extern struct  typFNT_GB16 const codeGB_16[];


int GetcodeGB_33_LENGTH(void);
int GetcodeGB_32_LENGTH(void);
int GetcodeGB_48_LENGTH(void);
//int GetcodeGB_49_LENGTH(void);
int GetcodeGB_40_LENGTH(void);
int GetcodeGB_24_LENGTH(void);
int GetcodeGB_16_LENGTH(void);


#endif  

