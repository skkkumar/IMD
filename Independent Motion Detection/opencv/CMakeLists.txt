
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME tracker)
PROJECT(${PROJECTNAME})
# include_directories(${Gurls++_INCLUDE_DIRS})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
# find_package(Gurls)
find_package(ICUBcontrib REQUIRED)
find_package(Gurls QUIET)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUBCONTRIB_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(OpenCV)

# include(iCubOptions)
# include(iCubHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
#   add_definitions(-DGURLS_AVAILABLE=1)
  add_definitions(${Gurls_DEFINITIONS})

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
)
include_directories(${Gurls_INCLUDE_DIRS})
# add_executable(${PROJECTNAME} ${source} ${header})


add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${Gurls_LIBRARIES})

# install(TARGETS ${PROJECTNAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)
install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin)