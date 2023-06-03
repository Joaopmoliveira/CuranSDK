cmake_minimum_required(VERSION 3.11)

project(FRILibrary)

add_library (FRILibrary STATIC 

#src/base
src/base/friClientApplication.cpp
src/base/friClientData.h

#src/client_lbr
src/client_lbr/friLBRClient.cpp
src/client_lbr/friLBRCommand.cpp
src/client_lbr/friLBRState.cpp

#src/client_trafo
src/client_trafo/friTransformationClient.cpp

#src/connection
src/connection/friUdpConnection.cpp

#src/nanopb-0.2.8
src/nanopb-0.2.8/pb_decode.c
src/nanopb-0.2.8/pb_encode.c

#src/protobuf
src/protobuf/friCommandMessageEncoder.cpp
src/protobuf/friMonitoringMessageDecoder.cpp
src/protobuf/pb_frimessages_callbacks.c

#src/protobuf_gen
src/protobuf_gen/FRIMessages.pb.c
)

target_include_directories(FRILibrary PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
