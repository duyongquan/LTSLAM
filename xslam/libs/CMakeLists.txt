set(AVX_SOURCES_HDRS vlfeat/mathop_avx.h)
set(AVX_SOURCES_SRCS vlfeat/mathop_avx.c)
set(AVX_SOURCES ${AVX_SOURCES_HDRS} ${AVX_SOURCES_SRCS})

set(SSE2_SOURCES_HDRS vlfeat/imopv_sse2.h vlfeat/mathop_sse2.h)
set(SSE2_SOURCES_SRCS vlfeat/imopv_sse2.c lfeat/mathop_sse2.c)

set(SSE2_SOURCES ${SSE2_SOURCES_HDRS} ${SSE2_SOURCES_SRCS})

file(GLOB_RECURSE VLFEAT_LIBRARY_HDRS "vlfeat/*.h")
file(GLOB_RECURSE VLFEAT_LIBRARY_SRCS "vlfeat/*.c")

list(REMOVE_ITEM VLFEAT_LIBRARY_HDRS ${AVX_SOURCES_HDRS})
list(REMOVE_ITEM VLFEAT_LIBRARY_HDRS ${VLFEAT_LIBRARY_SRCS})

set_source_files_properties(${AVX_SOURCES} PROPERTIES COMPILE_FLAGS "-mavx")
set_source_files_properties(${SSE2_SOURCES} PROPERTIES COMPILE_FLAGS "-msse2")

add_library(vlfeat STATIC ${VLFEAT_LIBRARY_HDRS} ${VLFEAT_LIBRARY_SRCS})