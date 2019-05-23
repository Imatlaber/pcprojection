macro(install_had_spy)
    file(WRITE "${CMAKE_BINARY_DIR}/had-spy.sh" "java -cp $HAD_BUILD/lcm/share/java/*:$HAD_BUILD/libbot/share/java/*:$HAD_BUILD/modules/share/java/* lcm.spy.Spy $*")
    file(INSTALL "${CMAKE_BINARY_DIR}/had-spy.sh"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
        FILE_PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE)
endmacro()
