include_directories(${dlib_INCLUDE_DIR})
include_directories(${BOOST_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
include_directories(${FaceAnalyser_SOURCE_DIR}/include)

SET(SOURCE
	FakeFace.cpp
	IHeadEstimator.cpp
	main.cpp
	routine.cpp
	dlib/EstimatorSourceData.cpp
	dlib/HeadPoseEstimator.cpp
	OpenFace/OFEstimator.cpp
)

SET(HEADERS
	FakeFace.h
	IHeadEstimator.h
	SampleGrabber0.h
	main.h
	dlib/EstimatorSourceData.h
	dlib/HeadPoseEstimator.h
	OpenFace/OFEstimator.h
)

# include_directories(./include)
# include_directories(${FACEANALYSER_SOURCE_DIR}/include)
# include_directories(../LandmarkDetector/include)

include_directories(${dlib_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../../third/DirectShow/baseclasses)
include_directories(../../third/LandmarkDetector/include)
include_directories(../../third/FaceAnalyser/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS}/boost)

link_directories(${Boost_LIBRARY_DIRS})

#
# Flag WIN32 makes app as Windows instead of Console
#
add_executable(VirtualCamManager WIN32 ${SOURCE} ${HEADERS})

add_definitions(-DHEAD_POSE_ESTIMATOR_DEBUG
	-DUSE_PPL
	-DDLIB_HAVE_SSE2
	-DDLIB_HAVE_SSE3
	-DDLIB_HAVE_SSE4
	-D_UNICODE
	-DUNICODE
)

target_link_libraries(VirtualCamManager strmiids.lib ${dlib_LIBRARIES} ${OpenCV_LIBRARIES} LandmarkDetector FaceAnalyser )

#
# Copy resources
#
add_custom_command(TARGET VirtualCamManager PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/data/manager $<TARGET_FILE_DIR:VirtualCamManager>)

# install (TARGETS FaceAnalyser DESTINATION lib)
# install (FILES ${HEADERS} DESTINATION include/OpenFace)
