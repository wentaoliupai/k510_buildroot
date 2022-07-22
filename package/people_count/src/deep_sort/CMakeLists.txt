
file(GLOB SOURCE
    "main.cc"
    "v4l2.c"
    "cv2_utils.cc"
    "object_detect.cc"
	"deep_sort.cpp"
    "drm/k510_drm.c"
    "drm/buf_mgt.cc"
    "definition/*.cpp"
    "matching/*.cpp"
    "3rd_party/*.cpp"
    "3rd_party/munkres/*.cpp"
    "counting/*.cpp"
)

set(bin people_count)
set(input ../data/imx219_0.conf ../data/imx219_1.conf ../data/video_320x320_240.conf ../data/people_count.sh)
set(data ../data/000001.jpg)

add_definitions(-DFULL_SCREEN)

include_directories ("./")
include_directories ("drm")
include_directories ("3rd_party")
include_directories ("definition")
include_directories ("matching")
include_directories ("counting")

include_directories(${DEMO_ROOT}/riscv64/include)
include_directories(${DEMO_ROOT}/riscv64/opencv_4.5.1/include/opencv4)
include_directories(${DEMO_ROOT}/riscv64/video_zt)

link_directories(${DEMO_ROOT}/riscv64/lib/)
link_directories(${DEMO_ROOT}/riscv64/opencv_4.5.1/lib)
link_directories(${DEMO_ROOT}/riscv64/video_zt)

add_executable(${bin} ${SOURCE})
target_link_libraries(${bin} mediactl drm pthread venc live555)
target_link_libraries(${bin} nncase.runtime nncase.rt_modules.k510)
target_link_libraries(${bin} opencv_core opencv_imgcodecs opencv_imgproc opencv_videoio)

install(TARGETS ${bin} DESTINATION exe)
install(PROGRAMS ${input} DESTINATION exe)
install(PROGRAMS ${data} DESTINATION exe)

