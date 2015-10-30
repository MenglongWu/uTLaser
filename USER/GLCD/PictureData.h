/*--  运用取模软件进行取模 --*/
/*--  比如进行取模：宽度x高度=320x240  --*/
#ifndef __TEMP_H 
#define __TEMP_H

extern uint8_t	Num_fuhao[224] ;
extern uint8_t	Num_0[224] ;
extern uint8_t	Num_1[224]  ;
extern uint8_t	Num_2[224] ;
extern uint8_t	Num_3[224] ;
extern uint8_t	Num_4[224] ;
extern uint8_t	Num_5[224] ;
extern uint8_t	Num_6[224] ;
extern uint8_t	Num_7[224] ;
extern uint8_t	Num_8[224] ;
extern uint8_t	Num_9[224] ;


extern uint8_t	Num_00[58] ;
extern uint8_t	Num_01[58] ;
extern uint8_t	Num_02[58] ;
extern uint8_t	Num_03[58] ;
extern uint8_t	Num_04[58] ;
extern uint8_t	Num_05[58] ;
extern uint8_t	Num_06[58] ;
extern uint8_t	Num_07[58] ;
extern uint8_t	Num_08[58] ;
extern uint8_t	Num_09[58] ;

extern uint8_t	Num_0dian[58] ;
extern uint8_t	Num_0H[58] ;
extern uint8_t	Num_0z[58] ;
extern uint8_t	Num_0K[58] ;
extern uint8_t	Num_0n[58] ;
extern uint8_t	Num_0m[58] ;
extern uint8_t	Num_0C[58] ;
extern uint8_t	Num_0W[58] ;

extern uint8_t	Num_0O[58] ;
extern uint8_t	Num_0f[58] ;



//logo 243x57
//时钟 77x44
//电池 56x44
//红光 62x44
extern const unsigned char gImage_logo[27702];
#ifdef _DEBUG_
 //const unsigned char gImage_logo[27702];
 #define gImage_battery_0_4 gImage_logo
 #define gImage_battery_1_4 gImage_logo
 #define gImage_battery_2_4 gImage_logo
 #define gImage_battery_3_4 gImage_logo
 #define gImage_battery_4_4 gImage_logo
 #define gImage_battery_ac gImage_logo
 #define gImage_redlight_off gImage_logo
 #define gImage_redlight_on gImage_logo
 #define gImage_t_10min gImage_logo
 #define gImage_t_off gImage_logo
 #define gImage_dbm   gImage_logo
#else
//电池 56x44
extern const unsigned char gImage_battery_0_4[4928];
extern const unsigned char gImage_battery_1_4[4928];
extern const unsigned char gImage_battery_2_4[4928];
extern const unsigned char gImage_battery_3_4[4928];
extern const unsigned char gImage_battery_4_4[4928];

extern const unsigned char gImage_battery_ac[4928] ;
extern const unsigned char gImage_redlight_off[5456];
extern const unsigned char gImage_redlight_on[5456];
extern const unsigned char gImage_t_10min[6776];

extern const unsigned char gImage_t_off[6776];
extern const unsigned char gImage_dbm[4158];
#endif
#endif

