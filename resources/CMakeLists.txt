
add_custom_command(OUTPUT resources
               COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_LIST_DIR}/images" "${CMAKE_BINARY_DIR}/images"
               DEPENDS "${CMAKE_CURRENT_LIST_DIR}/images"
               COMMENT "Currently copying the widget files to the correct directories"
               )

               
# create a target that includes some_file, this gives us a name that we can use later
add_custom_target(
  WIDGETRESOURCES
  DEPENDS resources
)
