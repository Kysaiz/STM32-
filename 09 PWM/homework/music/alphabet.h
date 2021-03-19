#ifndef __ALPHABET_H
#define __ALPHABET_H

//#define L1              262
//#define L1_SHARP        277
//#define L2              294
//#define L2_SHARP        311
//#define L3              330
//#define L4              349
//#define L4_SHARP        370
//#define L5              392
//#define L5_SHARP        415
//#define L6              440
//#define L6_SHARP        466
//#define L7              494

//#define M1           523
//#define M1_SHARP     554
//#define M2           584
//#define M2_SHARP     622
//#define M3           659
//#define M4           690
//#define M4_SHARP     740
//#define M5           790
//#define M5_SHARP     831
//#define M6           880
//#define M6_SHARP     932
//#define M7           988

//#define H1            1046
//#define H1_SHARP      1109
//#define H2            1175
//#define H2_SHARP      1245
//#define H3            1318
//#define H4            1397
//#define H4_SHARP      1480
//#define H5            1568
//#define H5_SHARP      1661
//#define H6            1760
//#define H6_SHARP      1865
//#define H7            1976



//C调
#define L1              131
#define L2              147
#define L3              165
#define L4              175
#define L5              196
#define L6              221
#define L7              248

#define M1           262
#define M2           294
#define M3           330
#define M4           350
#define M5           393
#define M6           441
#define M7           495

#define H1            525
#define H2            598
#define H3            661
#define H4            700
#define H5            786
#define H6            886
#define H7            990


//中低高音的Do Re Mi Fa So La Si
//                            中音区                   低音区                高音区
static unsigned int TONE[] = {0,M1,M2,M3,M4,M5,M6,M7,  L1,L2,L3,L4,L5,L6,L7, H1,H2,H3,H4,H5,H6,H7};


#endif

