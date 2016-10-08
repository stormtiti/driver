#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#ifndef APPLICFG_H
#define APPLICFG_H

#include <cstdint>
#include <sys/types.h>
/* Integers */

//#define DBG_PRINTF
//#define DGB_PRINTF_MOONSCMD
//#define DBG_PRINTF_DIFFWHEEL
//#define DGB_PRINTF_ODOMETER
//#define DGB_PRINTF_ODOMETER_COUNT
//#define TEST_DELTA_COUNT //test spd 0.5 linear test
#define INTEGER8 int8_t
#define INTEGER16 int16_t
#define INTEGER24 int32_t
#define INTEGER32 int32_t
#define INTEGER40 int64_t
#define INTEGER48 int64_t
#define INTEGER56 int64_t
#define INTEGER64 int64_t

/* Unsigned integers */

#define UNS8   u_int8_t
#define UNS16  u_int16_t
#define UNS32  u_int32_t
#define UNS24  u_int32_t
#define UNS40  u_int64_t
#define UNS48  u_int64_t
#define UNS56  u_int64_t
#define UNS64  u_int64_t

/* Reals */
/*
#define REAL32	float
#define REAL64 double
*/
typedef union {
	struct{
		UNS8 data0;
		UNS8 data1;
		UNS8 data2;
		UNS8 data3;
	}field;
	UNS32 u32Data;
}HEX_DATA32;

typedef union {
	struct{
		UNS8 data0;
		UNS8 data1;
	}field;
	UNS16 u16Data;
}HEX_DATA16;

#endif // APPLICFG_H

#ifdef __cplusplus
}
#endif
