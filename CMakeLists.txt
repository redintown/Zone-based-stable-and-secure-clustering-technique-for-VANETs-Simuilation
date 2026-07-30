build_exec(
    EXECNAME zone-secure-vanet

    EXECUTABLE_DIRECTORY_PATH ${CMAKE_CURRENT_SOURCE_DIR}

    SOURCE_FILES
        vanet.cc
        socket_client.cc
        node_manager.cc
        mobility_manager.cc
        wifi_manager.cc
        neighbour_manager.cc
        zone_manager.cc
        hello_manager.cc
        hello_message.cc
        rm_manager.cc
        cluster_manager.cc
        maintenance_manager.cc
        security_manager.cc
        statistics_manager.cc

    LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libinternet}
        ${libmobility}
        ${libwifi}
        ${libapplications}
)
