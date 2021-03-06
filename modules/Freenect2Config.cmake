

FIND_LIBRARY(freenect2_LIBRARY freenect2
        PATHS @CMAKE_INSTALL_PREFIX@/lib
        NO_DEFAULT_PATH
        )
SET(freenect2_LIBRARIES ${freenect2_LIBRARY} @LIBFREENECT2_THREADING_LIBRARIES@)
FIND_PATH(freenect2_INCLUDE_DIR @PROJECT_NAME@/libfreenect2.hpp
        PATHS @CMAKE_INSTALL_PREFIX@/include
        NO_DEFAULT_PATH
        )
SET(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR})
SET(freenect2_VERSION @PROJECT_VER@)