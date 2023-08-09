add_executable(Compiler 
src/compiler.cpp
)

target_link_libraries(Compiler nlohmann_json::nlohmann_json)

add_executable(WatchDog 
src/watchdog.cpp
)

add_executable(Sensors 
src/sensors.cpp
)

add_executable(Client 
src/client.cpp
)

target_include_directories(Sensors PUBLIC 
${Boost_INCLUDE_DIRS}
)
target_include_directories(Client PUBLIC 
${Boost_INCLUDE_DIRS}
)

if(UNIX AND NOT APPLE)
    target_link_libraries(WatchDog PUBLIC 
    Threads::Threads
    asio)

    target_link_libraries(Sensors PUBLIC 
    Threads::Threads
    asio
    rt
    atomic
    )

    target_link_libraries(Client PUBLIC 
    Threads::Threads
    asio
    rt
    atomic)

else()

    target_link_libraries(Sensors PUBLIC 
    asio
    )

    target_link_libraries(Client PUBLIC 
    asio
    )

    target_link_libraries(WatchDog PUBLIC 
    asio)
endif()