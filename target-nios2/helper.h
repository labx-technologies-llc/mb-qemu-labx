#include "def-helper.h"

DEF_HELPER_1(raise_exception, void, i32)

#if !defined(CONFIG_USER_ONLY)
DEF_HELPER_1(mmu_read, i32, i32)
DEF_HELPER_2(mmu_write, void, i32, i32)
#endif

DEF_HELPER_2(divs, i32, i32, i32)
DEF_HELPER_2(divu, i32, i32, i32)

DEF_HELPER_4(memalign, void, i32, i32, i32, i32)

#include "def-helper.h"

