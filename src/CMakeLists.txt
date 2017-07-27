add_definitions(
#    -ggdb3
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${LCMTYPES_INCLUDE_DIRS}
    )

add_executable(hr-gps-to-local main.c)

target_link_libraries (hr-gps-to-local ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(hr-gps-to-local
    glib-2.0
    lcm
    bot2-core
    lcmtypes_hr-lcmtypes
    lcmtypes-gps18
    lcmtypes_ros2lcm_bridge
    bot2-param-client
    bot2-frames
    hr-common)

pods_install_executables(hr-gps-to-local)
