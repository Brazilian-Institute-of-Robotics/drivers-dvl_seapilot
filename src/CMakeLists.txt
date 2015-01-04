rock_library(dvl_seapilot
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(dvl_seapilot_bin Main.cpp
    DEPS dvl_seapilot)

