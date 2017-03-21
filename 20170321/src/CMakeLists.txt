set(OpenCV_DIR "/home/geo/opencv-2.4.13/build")
find_package(OpenCV REQUIRED)
add_library(base base.cpp)
target_link_libraries(base
${OpenCV_LIBS})

add_executable(main transform.cpp)
target_link_libraries(main 
	base
	${OpenCV_LIBS})
