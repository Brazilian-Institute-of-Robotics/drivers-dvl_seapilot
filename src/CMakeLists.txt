rock_library(dvl_seapilot
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base dvl_teledyne)

rock_executable(dvl_seapilot_read Main.cpp
    DEPS dvl_seapilot)

