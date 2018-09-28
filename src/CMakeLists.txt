add_definitions(
#    -ggdb3
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${LCMTYPES_INCLUDE_DIRS}
    )

add_executable(gps-to-local main.c)

target_link_libraries (gps-to-local ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(gps-to-local
    glib-2.0
    lcm
    bot2-core
    lcmtypes_hr-lcmtypes
    lcmtypes-gps18
    bot2-param-client
    bot2-frames
    common-utils)

pods_install_executables(gps-to-local)
