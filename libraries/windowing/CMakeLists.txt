add_library(windowing
src/BiopsyViewer.cpp
src/FAIViewer.cpp
src/MedicalViewer.cpp
src/WindowBody.cpp
src/WindowManager.cpp
)

target_include_directories(windowing
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(windowing PUBLIC

)