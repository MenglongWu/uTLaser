#ifndef _GL_TYPE_H_
#define _GL_TYPE_H_


// typedef unsigned char uint8_t;
// typedef unsigned short uint16_t;
// typedef unsigned long uint32_t;

// typedef signed char int8_t;
// typedef signed short int16_t;
// typedef signed long int32_t;


#define far
#define near
#define pascal
#define NEAR                near
#define FAR                 far
#ifndef CONST
#define CONST               const
#endif


typedef long                LONG;
typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef FLOAT               *PFLOAT;
typedef BOOL near           *PBOOL;
typedef BOOL far            *LPBOOL;
typedef BYTE near           *PBYTE;
typedef BYTE far            *LPBYTE;
typedef int near            *PINT;
typedef int far             *LPINT;
typedef WORD near           *PWORD;
typedef WORD far            *LPWORD;
typedef long far            *LPLONG;
typedef DWORD near          *PDWORD;
typedef DWORD far           *LPDWORD;
typedef void far            *LPVOID;
typedef CONST void far      *LPCVOID;

typedef int                 INT;
typedef unsigned int        UINT;
typedef unsigned int        *PUINT;


//
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif


#define MAKEWORD(a, b)      ((WORD)(((BYTE)((DWORD_PTR)(a) & 0xff)) | ((WORD)((BYTE)((DWORD_PTR)(b) & 0xff))) << 8))
#define MAKELONG(a, b)      ((LONG)(((WORD)((DWORD_PTR)(a) & 0xffff)) | ((DWORD)((WORD)((DWORD_PTR)(b) & 0xffff))) << 16))
#define LOWORD(l)           ((WORD)((DWORD_PTR)(l) & 0xffff))
#define HIWORD(l)           ((WORD)((DWORD_PTR)(l) >> 16))
#define LOBYTE(w)           ((BYTE)((DWORD_PTR)(w) & 0xff))
#define HIBYTE(w)           ((BYTE)((DWORD_PTR)(w) >> 8))


typedef DWORD   COLORREF;
typedef DWORD   *LPCOLORREF;


#define HFILE_ERROR ((HFILE)-1)

typedef struct tagRECT
{
    LONG    left;
    LONG    top;
    LONG    right;
    LONG    bottom;
} RECT, *PRECT, NEAR *NPRECT, FAR *LPRECT;

typedef const RECT FAR* LPCRECT;

typedef struct _RECTL       /* rcl */
{
    LONG    left;
    LONG    top;
    LONG    right;
    LONG    bottom;
} RECTL, *PRECTL, *LPRECTL;

typedef const RECTL FAR* LPCRECTL;

typedef struct tagPOINT
{
    LONG  x;
    LONG  y;
} POINT, *PPOINT, NEAR *NPPOINT, FAR *LPPOINT;

typedef struct _POINTL      /* ptl  */
{
    LONG  x;
    LONG  y;
} POINTL, *PPOINTL;

typedef struct tagSIZE
{
    LONG        cx;
    LONG        cy;
} SIZE, *PSIZE, *LPSIZE;

typedef SIZE               SIZEL;
typedef SIZE               *PSIZEL, *LPSIZEL;


//color
#define 	COL_AliceBlue	0xF0F8FF
#define 	COL_AntiqueWhite	0xFAEBD7
#define 	COL_Aqua	0x00FFFF
#define 	COL_Aquamarine	0x7FFFD4
#define 	COL_Azure	0xF0FFFF
#define 	COL_Beige	0xF5F5DC
#define 	COL_Bisque	0xFFE4C4
#define 	COL_Black	0x000000
#define 	COL_BlanchedAlmond	0xFFEBCD
#define 	COL_Blue	0x0000FF
#define 	COL_BlueViolet	0x8A2BE2
#define 	COL_Brown	0xA52A2A
#define 	COL_BurlyWood	0xDEB887
#define 	COL_CadetBlue	0x5F9EA0
#define 	COL_Chartreuse	0x7FFF00
#define 	COL_Chocolate	0xD2691E
#define 	COL_Coral	0xFF7F50
#define 	COL_CornflowerBlue	0x6495ED
#define 	COL_Cornsilk	0xFFF8DC
#define 	COL_Crimson	0xDC143C
#define 	COL_Cyan	0x00FFFF
#define 	COL_DarkBlue	0x00008B
#define 	COL_DarkCyan	0x008B8B
#define 	COL_DarkGoldenrod	0xB8860B
#define 	COL_DarkGray	0xA9A9A9
#define 	COL_DarkGreen	0x006400
#define 	COL_DarkKhaki	0xBDB76B
#define 	COL_DarkMagenta	0x8B008B
#define 	COL_DarkOliveGreen	0x556B2F
#define 	COL_DarkOrange	0xFF8C00
#define 	COL_DarkOrchid	0x9932CC
#define 	COL_DarkRed	0x8B0000
#define 	COL_DarkSalmon	0xE9967A
#define 	COL_DarkSeaGreen	0x8FBC8B
#define 	COL_DarkSlateBlue	0x483D8B
#define 	COL_DarkSlateGray	0x2F4F4F
#define 	COL_DarkTurquoise	0x00CED1
#define 	COL_DarkViolet	0x9400D3
#define 	COL_DeepPink	0xFF1493
#define 	COL_DeepSkyBlue	0x00BFFF
#define 	COL_DimGray	0x696969
#define 	COL_DodgerBlue	0x1E90FF
#define 	COL_Firebrick	0xB22222
#define 	COL_FloralWhite	0xFFFAF0
#define 	COL_ForestGreen	0x228B22
#define 	COL_Fuchsia	0xFF00FF
#define 	COL_Gainsboro	0xDCDCDC
#define 	COL_GhostWhite	0xF8F8FF
#define 	COL_Gold	0xFFD700
#define 	COL_Goldenrod	0xDAA520
#define 	COL_Gray	0x808080
#define 	COL_Green	0x008000
#define 	COL_GreenYellow	0xADFF2F
#define 	COL_Honeydew	0xF0FFF0
#define 	COL_HotPink	0xFF69B4
#define 	COL_IndianRed	0xCD5C5C
#define 	COL_Indigo	0x4B0082
#define 	COL_Ivory	0xFFFFF0
#define 	COL_Khaki	0xF0E68C
#define 	COL_Lavender	0xE6E6FA
#define 	COL_LavenderBlush	0xFFF0F5
#define 	COL_LawnGreen	0x7CFC00
#define 	COL_LemonChiffon	0xFFFACD
#define 	COL_LightBlue	0xADD8E6
#define 	COL_LightCoral	0xF08080
#define 	COL_LightCyan	0xE0FFFF
#define 	COL_LightGoldenrodYellow	0xFAFAD2
#define 	COL_LightGray	0xD3D3D3
#define 	COL_LightGreen	0x90EE90
#define 	COL_LightPink	0xFFB6C1
#define 	COL_LightSalmon	0xFFA07A
#define 	COL_LightSeaGreen	0x20B2AA
#define 	COL_LightSkyBlue	0x87CEFA
#define 	COL_LightSlateGray	0x778899
#define 	COL_LightSteelBlue	0xB0C4DE
#define 	COL_LightYellow	0xFFFFE0
#define 	COL_Lime	0x00FF00
#define 	COL_LimeGreen	0x32CD32
#define 	COL_Linen	0xFAF0E6
#define 	COL_Magenta	0xFF00FF
#define 	COL_Maroon	0x800000
#define 	COL_MediumAquamarine	0x66CDAA
#define 	COL_MediumBlue	0x0000CD
#define 	COL_MediumOrchid	0xBA55D3
#define 	COL_MediumPurple	0x9370DB
#define 	COL_MediumSeaGreen	0x3CB371
#define 	COL_MediumSlateBlue	0x7B68EE
#define 	COL_MediumSpringGreen	0x00FA9A
#define 	COL_MediumTurquoise	0x48D1CC
#define 	COL_MediumVioletRed	0xC71585
#define 	COL_MidnightBlue	0x191970
#define 	COL_MintCream	0xF5FFFA
#define 	COL_MistyRose	0xFFE4E1
#define 	COL_Moccasin	0xFFE4B5
#define 	COL_NavajoWhite	0xFFDEAD
#define 	COL_Navy	0x000080
#define 	COL_OldLace	0xFDF5E6
#define 	COL_Olive	0x808000
#define 	COL_OliveDrab	0x6B8E23
#define 	COL_Orange	0xFFA500
#define 	COL_OrangeRed	0xFF4500
#define 	COL_Orchid	0xDA70D6
#define 	COL_PaleGoldenrod	0xEEE8AA
#define 	COL_PaleGreen	0x98FB98
#define 	COL_PaleTurquoise	0xAFEEEE
#define 	COL_PaleVioletRed	0xDB7093
#define 	COL_PapayaWhip	0xFFEFD5
#define 	COL_PeachPuff	0xFFDAB9
#define 	COL_Peru	0xCD853F
#define 	COL_Pink	0xFFC0CB
#define 	COL_Plum	0xDDA0DD
#define 	COL_PowderBlue	0xB0E0E6
#define 	COL_Purple	0x800080
#define 	COL_Red	0xFF0000
#define 	COL_RosyBrown	0xBC8F8F
#define 	COL_RoyalBlue	0x4169E1
#define 	COL_SaddleBrown	0x8B4513
#define 	COL_Salmon	0xFA8072
#define 	COL_SandyBrown	0xF4A460
#define 	COL_SeaGreen	0x2E8B57
#define 	COL_SeaShell	0xFFF5EE
#define 	COL_Sienna	0xA0522D
#define 	COL_Silver	0xC0C0C0
#define 	COL_SkyBlue	0x87CEEB
#define 	COL_SlateBlue	0x6A5ACD
#define 	COL_SlateGray	0x708090
#define 	COL_Snow	0xFFFAFA
#define 	COL_SpringGreen	0x00FF7F
#define 	COL_SteelBlue	0x4682B4
#define 	COL_Tan	0xD2B48C
#define 	COL_Teal	0x008080
#define 	COL_Thistle	0xD8BFD8
#define 	COL_Tomato	0xFF6347
#define 	COL_Transparent	0x00FFFFFF
#define 	COL_Turquoise	0x40E0D0
#define 	COL_Violet	0xEE82EE
#define 	COL_Wheat	0xF5DEB3
#define 	COL_White	0xFFFFFF
#define 	COL_WhiteSmoke	0xF5F5F5
#define 	COL_Yellow	0xFFFF00
#define 	COL_YellowGreen	0x9ACD32


#endif

