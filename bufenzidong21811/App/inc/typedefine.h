#ifndef __TYPEDEFINE_H__
#define __TYPEDEFINE_H__


typedef unsigned	char		BOOLEAN;
typedef unsigned	char		INT8U;                    /* Unsigned  8 bit quantity                           */
typedef signed		char		INT8S;                    /* Signed    8 bit quantity                           */
typedef unsigned	short		INT16U;                   /* Unsigned 16 bit quantity                           */
typedef signed		short		INT16S;                   /* Signed   16 bit quantity                           */
typedef unsigned	int		    INT32U;                   /* Unsigned 32 bit quantity                           */
typedef signed		int		    INT32S;                   /* Signed   32 bit quantity                           */
typedef unsigned	long		LONGU;                    /* Unsigned 32 bit quantity                           */
typedef signed		long		LONGS;                    /* Unsigned 32 bit quantity                           */
typedef unsigned	long long	INT64U;                   /* Unsigned 64 bit quantity                           */
typedef signed		long long	INT64S;                   /* Signed   64 bit quantity                           */
typedef float				    FP32;                     /* Single precision floating point                    */
typedef double				    FP64;                     /* Double precision floating point                    */

typedef unsigned	char*		PINT8U;                   /* Unsigned  8 bit quantity                           */
typedef unsigned	short*		PINT16U;                  /* Unsigned 16 bit quantity                           */
typedef unsigned	int*		PINT32U;                  /* Unsigned 32 bit quantity                           */
typedef unsigned	long*		PLONGU;                   /* Unsigned 32 bit quantity                           */


//typedef enum {
//    FALSE = 0,
//    TRUE = !FALSE,
//} BOOL, bool;



//-------------定义多种数据类型的联合，以支持不同数据类型-----------------

//4字节数据类型
typedef union
{
	unsigned    char        U8[4];
	signed      char        S8[4];
	unsigned    short       U16[2];
	signed      short       S16[2];
	unsigned    int         U32;
	signed      int         S32;
	float			        F32;
} Byte4MultipleDate_Type;


//2字节数据类型，数据类型转换时用
typedef union
{
	unsigned    char        U8[2];
	signed      char        S8[2];
	unsigned    short       U16;
	signed      short       S16;
} Byte2MultipleDate_Type;


#endif

