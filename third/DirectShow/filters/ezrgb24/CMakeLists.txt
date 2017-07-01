SET(SOURCE
	ezprop.cpp
	ezrgb24.cpp
	ezprop.rc
)

SET(HEADERS
	ezprop.h
	ezrgb24.h
	ezuids.h
	iez.h
	resource.h
	ezrgb24.def
)

include_directories(.)
include_directories(../../baseclasses)

add_definitions( -DEZRGB24_EXPORTS
	-D_UNICODE
	-DUNICODE
)

add_library(ezrgb24 SHARED ${SOURCE} ${HEADERS} )

target_link_libraries(ezrgb24 strmbase winmm.lib)