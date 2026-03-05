add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_map_chunker
        run_map_chunker.cc
)
target_link_libraries(run_map_chunker
        ${PROJECT_NAME}.libs
        ${third_party_libs}
        ${catkin_LIBRARIES}
)

add_executable(run_global_pcd_chunker
        run_map_chunker.cc
)
target_compile_definitions(run_global_pcd_chunker PRIVATE LM_SINGLE_INPUT_MODE=1)
target_link_libraries(run_global_pcd_chunker
        ${PROJECT_NAME}.libs
        ${third_party_libs}
        ${catkin_LIBRARIES}
)

add_executable(test_ui
        test_ui.cc
)

target_link_libraries(test_ui
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online run_map_chunker run_global_pcd_chunker
        DESTINATION lib/${PROJECT_NAME})
