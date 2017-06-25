cmake_minimum_required(VERSION 2.8)
project(FakeFace)


find_package(dlib REQUIRED)

MESSAGE("dlib information:") 
MESSAGE(STATUS "  Using dlib-${dlib_VERSION}")
MESSAGE(STATUS "  dlib_INCLUDE_DIRS: ${dlib_INCLUDE_DIRS}") 
MESSAGE(STATUS "  dlib_LIBRARIES: ${dlib_LIBRARIES}") 
include_directories(${dlib_INCLUDE_DIRS})


find_package( OpenCV REQUIRED COMPONENTS core photo video objdetect calib3d features2d flann highgui imgproc imgcodecs)

MESSAGE("OpenCV information:") 
MESSAGE(STATUS "  OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}") 
MESSAGE(STATUS "  OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}") 
MESSAGE(STATUS "  OpenCV_LIBRARY_DIRS: ${OpenCV_LIBRARY_DIRS}") 
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Boost 1.5.9 REQUIRED COMPONENTS filesystem system)
MESSAGE("Boost information:") 
MESSAGE(STATUS "  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE(STATUS "  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE(STATUS "  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS}/boost)
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})	



include_directories(${LandmarkDetector_SOURCE_DIR}/include)
include_directories(${FaceAnalyser_SOURCE_DIR}/include)


add_subdirectory(third/LandmarkDetector)
add_subdirectory(third/FaceAnalyser)
add_subdirectory(projects/VirtualCamManager)
add_subdirectory(projects/VirtualCamDevice)

add_dependencies (FaceAnalyser LandmarkDetector)
add_dependencies (VirtualCamManager FaceAnalyser)

# MESSAGE("Download shape predictor resource") 
# set(MY_URL "http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
# set(MY_DOWNLOAD_PATH "d.d")
# set(MY_EXTRACTED_FILE "path/to/extracted/file")

# if (NOT EXISTS "${MY_DOWNLOAD_PATH}")
#     file(DOWNLOAD "${MY_URL}" "${MY_DOWNLOAD_PATH}" INACTIVITY_TIMEOUT 5 SHOW_PROGRESS)
# endif()


# target_link_libraries(VirtualCamManager FaceAnalyser LandmarkDetector ${dlib_LIBRARIES})
# target_link_libraries(main FaceAnalyser LandmarkDetector ${dlib_LIBRARIES})