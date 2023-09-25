---
layout: "page"
permalink : "/3d_rendering/"
---

### 3D Rendering

One very interesting tools which is always usefull while using robots and other sensors which can interact in real time is to be able to render 3D scenes. As always we first see how we can link our executable to the 3D rendering library.

```cmake
find_package(vsg)
find_package(vsgXchange)

add_executable(myexecutable main.cpp)
target_compile_definitions(myexecutable PRIVATE vsgXchange_FOUND)
target_compile_definitions(myexecutable PRIVATE CURAN_COPIED_RESOURCE_PATH="${post_build_resource_path}")
target_link_libraries(myexecutable
PUBLIC
vsg::vsg
vsgXchange::vsgXchange
renderable
)
```

# Empty Scene

To render a 3D scene we have a couple of requirments in our lab. We must have the flexibility do add objects at runtime, we must be able to delete objects from a scene, add objects whilst the program is running. This is how you can create a basic empty scene. 

```cpp
#include "rendering/SequencialLinks.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        window.run();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```
This source code will create an empty window which we can rotate and move.
![empty_world](assets/images/empty_world.png)

The wired floor is automatically added to the scene to facilitate the visualization by inexperienced personell viewing our demos (simmilar to the background of blender). The code blocks once the window.run() method is called. Now assume that we add a function in a thread that goes to do something like waiting for a connection to be established and once this connection is established we want to add a box to the scene. This is how you would achieve this behavior.

# Add asyncronous box

```cpp
#include "rendering/SequencialLinks.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>
#include <chrono>

int mimic_waiting_for_connection(curan::renderable::Window& window){
    std::this_thread::sleep_for(std::chrono::seconds(10)); //mimic waiting for a connection that takes 10 seconds
    curan::renderable::Box::Info create_info;
    create_info.geomInfo.dx = vsg::vec3(0.5,0.0,0.0);
    create_info.geomInfo.dy = vsg::vec3(0.0,0.5,0.0);
    create_info.geomInfo.dz = vsg::vec3(0.0,0.0,0.5);
    create_info.geomInfo.position = vsg::vec3(0.0,0.0,0.0);
    create_info.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    create_info.builder = vsg::Builder::create();
    vsg::ref_ptr<curan::renderable::Renderable> box = curan::renderable::Box::make(create_info);
    window << box;
};

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};
        std::thread connect_thread(mimic_waiting_for_connection(window));
        window.run();
        connect_thread.join();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

```

This code adds objects to our scene asyncrounously, which means that we never block the entire scene while waiting for our object. The scene is now 
![world_with_box](assets/images/world_with_box.png)

For a reference of objects that you can add to the scene look and the classes available inside the renderer library. There are two special objects which need a bit more attention. 

# SequencialLinks

How to create a robot is one of them. So in Curan you can create an Object called Sequencial Links. The API we use follows the Denavit-Harterberg convention. To define the robot in a file you need to write a json file describing where the meshes are, whats their relative transformation with respect to the previous link. Here is the json format of the LBR Med where you specify where the meshes are relative to the json file.

```json
[
    {
        "path": "BaseModified.obj",
        "d_offset" : 0.0,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : 0
    },
    {
        "path": "Link1Modified.obj",
        "d_offset" : 0.34,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : -90
    },
    {
        "path": "Link2Modified.obj",
        "d_offset" : 0.0,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : 90
    },
    {
        "path": "Link3Modified.obj",
        "d_offset" : 0.4,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : 90
    },
    {
        "path": "Link4Modified.obj",
        "d_offset" : 0.0,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : -90
    },
    {
        "path": "Link5Modified.obj",
        "d_offset" : 0.4,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : -90
    },
    {
        "path": "Link6Modified.obj",
        "d_offset" : 0.0,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : 90
    },
    {
        "path": "Link7Modified.obj",
        "d_offset" : 0,
        "theta" : 0 ,
        "a_offset" : 0,
        "alpha" : 0
    }
]

```

Thus your file structure would look something like this
```
some directory ---
                 |-> arm.json
                 |-> BaseModified.obj
                 |-> Link1Modified.obj
                 |-> Link2Modified.obj
                 |-> Link3Modified.obj
                 |-> Link4Modified.obj
                 |-> Link5Modified.obj
                 |-> Link6Modified.obj
                 |-> Link7Modified.obj
```

Now in your C++ code you can just launch a function which appends the robot to the window and then moves the robot in real time

```cpp
#include "rendering/SequencialLinks.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>

void move_robot(curan::renderable::Window& window, std::atomic<bool>& continue_updating){
    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
    window << robotRenderable;

    double angle = 0.0;
    double time = 0.0;
    while(continue_updating.load()){
        auto robot = robotRenderable->cast<curan::renderable::SequencialLinks>();
        for(size_t index = 0; index < 7 ; ++index)
            robot->set(index,angle);
        angle = std::sin(time)*1.5;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
        time += 0.016;
    }
}

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::atomic<bool> continue_updating = true;
        std::thread local_thread{move_robot(window,continue_updating)};

        window.run();
        continue_updating.store(false);
        local_thread.join();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

```

And finaly this is the result of all of our hard work
![world_with_box](assets/images/sequencial_links.png)