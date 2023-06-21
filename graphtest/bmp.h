#ifndef BMPLINB_INCLUDEGUARD
#define BMPLINB_INCLUDEGUARD

/*****************************************************************************/
/*                                                                           */
/*     bmp.h: bmp ファイル処理のライブラリのためのヘッダファイル             */
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
#define HEADERSIZE   54               /* ヘッダのサイズ 54 = 14 + 40         */
#define PALLETSIZE 1024               /* パレットのサイズ                    */
#define MAXWIDTH   1024               /* 幅(pixel)の上限                     */
#define MAXHEIGHT  1024               /* 高さ(pixel) の上限                  */

/* x と y の交換のための マクロ関数 */
#define SWAP(x,y) {typeof(x) temp; temp=x; x=y; y=temp;}

typedef struct {                      /* 1ピクセルあたりの赤緑青の各輝度     */
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