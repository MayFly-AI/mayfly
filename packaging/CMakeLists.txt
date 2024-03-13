project(packaging)

add_custom_target(deb DEPENDS packaging-cmds mayfly)

add_custom_command(OUTPUT packaging-cmds
		   COMMAND ${PROJECT_SOURCE_DIR}/make_deb.sh "${PROJECT_SOURCE_DIR}" "${CMAKE_BINARY_DIR}"
		  )

# not actually generated so set it to symbolic
set_source_files_properties(packaging-cmds PROPERTIES SYMBOLIC "true")
