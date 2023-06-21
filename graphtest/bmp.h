#ifndef BMPLINB_INCLUDEGUARD
#define BMPLINB_INCLUDEGUARD

/*****************************************************************************/
/*                                                                           */
/*     bmp.h: bmp �t�@�C�������̃��C�u�����̂��߂̃w�b�_�t�@�C��             */
/*                                                                           */
/*             Kazutoshi Ando (Shizuoka Univ.)                               */
/*                                                                           */
/*                  Ver. 2004.08.18                                          */
/*                  Ver. 2004.08.17                                          */
/*                  Ver. 2003.11.04                                          */
/*                                                                           */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define HEADERSIZE   54               /* �w�b�_�̃T�C�Y 54 = 14 + 40         */
#define PALLETSIZE 1024               /* �p���b�g�̃T�C�Y                    */
#define MAXWIDTH   1024               /* ��(pixel)�̏��                     */
#define MAXHEIGHT  1024               /* ����(pixel) �̏��                  */

/* x �� y �̌����̂��߂� �}�N���֐� */
#define SWAP(x,y) {typeof(x) temp; temp=x; x=y; y=temp;}

typedef struct {                      /* 1�s�N�Z��������̐ԗΐ̊e�P�x     */
	unsigned char r;
	unsigned char g;
	unsigned char b;
} color;

typedef struct {
	long height;
	long width;
	color data[MAXHEIGHT][MAXWIDTH];
} img;

void ReadBmp(const char* filename, img* imgp);
void WriteBmp(const char* filename, img* tp);
void PrintBmpInfo(const char* filename);
void HMirror(img* sp, img* tp);
void VMirror(img* sp, img* tp);
void Rotate90(int a, img* sp, img* tp);
void Shrink(int a, img* sp, img* tp);
void Mosaic(int a, img* sp, img* tp);
void Gray(img* sp, img* tp);
void Diminish(img* sp, img* tp, unsigned char x);

#endif