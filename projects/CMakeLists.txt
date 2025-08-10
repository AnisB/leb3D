# Convert grid to leb3d
bacasable_exe(convert_grid_to_leb3d "projects" "convert_grid_to_leb3d.cpp;" "${DEMO_SDK_INCLUDES};")
target_link_libraries(convert_grid_to_leb3d "demo" "${D3D12_LIBRARIES}")
set_target_properties(convert_grid_to_leb3d PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "${PROJECT_SOURCE_DIR}")

# Render volume
bacasable_exe(render_volume "projects" "render_volume.cpp;" "${DEMO_SDK_INCLUDES}")
target_link_libraries(render_volume "demo" "${D3D12_LIBRARIES}")
copy_dir_next_to_binary(render_volume "${PROJECT_3RD_BINARY}/D3D12" "D3D12")
set_target_properties(render_volume PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "${PROJECT_SOURCE_DIR}")
