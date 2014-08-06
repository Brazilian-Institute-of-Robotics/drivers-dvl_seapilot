rock_library(dvl_rti
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(dvl_rti_bin Main.cpp
    DEPS dvl_rti)

