#TBB library
include_directories(${TBB_ROOT_DIR}/include)

include_directories(${BOOST_INCLUDE_DIR})

SET(SOURCE
	src/CCNF_patch_expert.cpp
	src/LandmarkDetectionValidator.cpp
	src/LandmarkDetectorFunc.cpp
	src/LandmarkDetectorModel.cpp
	src/LandmarkDetectorUtils.cpp
	src/LandmarkDetectorParameters.cpp
	src/Patch_experts.cpp
	src/PAW.cpp
	src/PDM.cpp
	src/SVR_patch_expert.cpp
)

SET(HEADERS
	include/CCNF_patch_expert.h	
	include/LandmarkCoreIncludes.h
	include/LandmarkDetectionValidator.h
	include/LandmarkDetectorFunc.h
	include/LandmarkDetectorModel.h
	include/LandmarkDetectorParameters.h
	include/LandmarkDetectorUtils.h
	include/Patch_experts.h	
	include/PAW.h
	include/PDM.h
	include/SVR_patch_expert.h		
)

include_directories(./include)
include_directories(${LandmarkDetector_SOURCE_DIR}/include)


add_definitions(-DUSE_PPL
	-DDLIB_HAVE_SSE2
	-DDLIB_HAVE_SSE3
	-DDLIB_HAVE_SSE4
	-D_UNICODE
	-DUNICODE
)


add_library( LandmarkDetector ${SOURCE} ${HEADERS} )

# install (TARGETS LandmarkDetector DESTINATION lib)
# install (FILES ${HEADERS} DESTINATION include/OpenFace)
