#ifndef _TYPES_
#define _TYPES_

#include <stdint.h>


#undef bool
#undef true
#undef false

typedef _Bool bool;
#define false 0
#define true (!false)


#ifndef NULL 
#define NULL ((void*)0)
#endif


/* Some definitions for registers memory mapping. All just resolve to volatile. This way, allowed usage can be seen in the typedefs. HW_WO: Write only, HW_RO: Read only, HW_RW: Read/Write, HW_RS: Reserved (do not access), HW_UU: Unused (padding etc.) */

typedef volatile uint32_t HW_WO;
typedef volatile uint32_t HW_RO;
typedef volatile uint32_t HW_RW;
typedef volatile uint32_t HW_RS;
typedef volatile uint32_t HW_UU;


#endif
