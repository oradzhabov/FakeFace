SET(SOURCE
	Dll.cpp
	Routines.cpp
	VirtualCamDevice.cpp
)

SET(HEADERS
	VirtualCamDevice.h
	VirtualCamDevice.def
)


include_directories(../../third/DirectShow/baseclasses)




add_library(VirtualCamDevice SHARED ${SOURCE} ${HEADERS})

add_definitions(-D_USRDLL
	-DVIRTUALCAMDEVICE_EXPORTS
	-D_UNICODE
	-DUNICODE
)

#
# Copy resources
#
add_custom_command(TARGET VirtualCamDevice PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/data/device $<TARGET_FILE_DIR:VirtualCamDevice>)


target_link_libraries(VirtualCamDevice strmbase )

# install (TARGETS FaceAnalyser DESTINATION lib)
# install (FILES ${HEADERS} DESTINATION include/OpenFace)
