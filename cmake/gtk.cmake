find_package(PkgConfig REQUIRED)
find_package(GTK2 2.6 REQUIRED gtk)
pkg_check_modules(GTKMM gtkmm-2.4)

include_directories(SYSTEM ${GTK2_INCLUDE_DIRS} ${GTKMM_INCLUDE_DIRS})
