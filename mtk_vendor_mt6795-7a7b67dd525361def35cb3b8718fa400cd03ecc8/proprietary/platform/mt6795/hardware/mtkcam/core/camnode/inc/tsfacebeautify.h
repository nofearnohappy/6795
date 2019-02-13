#include <stdio.h> 
#include "tsoffscreen.h"
#include "tcomdef.h"


#ifdef __cplusplus
extern "C" {
#endif

void tShzFaceBeautify(LPTSOFFSCREEN pFaceImg, 
						   LPTSOFFSCREEN pDstImage, 
						   TRECT rcFace,
						   TInt32 lBlurLevel,
						   TInt32 lWhitenLevel,
						   TBool bCaptureProcess,
						   TInt32 ColorId);

int tShzFaceBeautify_init();

void tShzFaceBeautify_uninit();

extern unsigned char ts0011[256];
extern unsigned char ts0012[256];
extern unsigned char ts0021[256];
extern unsigned char ts0022[256];
extern unsigned char ts0031[256];
extern unsigned char ts0032[256];
extern unsigned char ts0041[256];
extern unsigned char ts0042[256];
extern unsigned char ts0051[256];
extern unsigned char ts0052[256];
extern unsigned char ts0061[256];
extern unsigned char ts0062[256];
extern unsigned char ts0071[256];
extern unsigned char ts0072[256];
extern unsigned char ts0081[256];
extern unsigned char ts0082[256];

#ifdef __cplusplus
};
#endif
