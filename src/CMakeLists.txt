add_library(${PROJECT_NAME}.libs SHARED
        common/nav_state.cc
        common/constant.h
        common/options.cc
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc
        map_runtime/dynamic_map_manager.cc

        bridges/localization_bridge.cc
        bridges/localization_bridge_ros1.cc
        bridges/sensor_conversion.cc

        adapters/laser_mapping_adapter.cc
        adapters/lidar_loc_adapter.cc
        adapters/pgo_adapter.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization_builder.cc
        core/localization/localization_runtime_factory.cc
        core/localization/localization.cpp
        pipelines/motion_pipeline.cc

        utils/timer.cc
        utils/pointcloud_utils.cc

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)
include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

target_link_libraries(${PROJECT_NAME}.libs
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}.libs ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils
        DESTINATION lib)

add_subdirectory(app)
