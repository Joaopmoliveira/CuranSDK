add_executable(test_pytorch_initial main.cpp)

target_link_libraries(test_pytorch_initial  
${TORCH_LIBRARIES}
OpenMP::OpenMP_CXX
LAPACK::LAPACK
BLAS::BLAS
gflags::gflags
glog::glog
)

target_include_directories(test_pytorch_initial PUBLIC
${TORCH_INCLUDE_DIRS})