#ifndef CURAN_WINDOW_RENDERABLE_HEADER_FILE_
#define CURAN_WINDOW_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <variant>
#include "Renderable.h"
#include "ImGUIInterface.h"
#include <optional>
#include <list>
#include <utility>
#include <mutex>

namespace curan {
    namespace renderable {

        constexpr bool full_screen_mode = true;

        struct Window {
            vsg::ref_ptr<vsg::Window> window;
            vsg::ref_ptr<vsg::WindowTraits> traits;
            vsg::ref_ptr<vsg::ResourceHints> resourceHints;
            vsg::ref_ptr<vsg::Group> root;
            vsg::ref_ptr<vsg::Group> root_plus_floor;
            vsg::ref_ptr<vsg::Viewer> viewer;
            vsg::ref_ptr<vsg::Camera> camera;
            vsg::ref_ptr<vsg::CommandGraph> commandGraph;
            vsg::ref_ptr<vsg::ProjectionMatrix> perspective;
            vsg::ref_ptr<vsg::ViewportState> viewportState;
            size_t number_of_images = 0;
            std::unordered_map<std::string, vsg::ref_ptr<Renderable>> contained_objects;
            std::list<std::pair<size_t,vsg::ref_ptr<Renderable>>> deleted_resource_manager;
            std::mutex mut;
        public:

            struct WindowSize {
                size_t width = 800;
                size_t height = 800;
            };

            struct Info {
                bool is_debug = false;
                bool api_dump = false;
                int screen_number = 0;
                std::string display = "";
                std::string title = "noname";
                bool full_screen;
                std::variant<bool, WindowSize> window_size;
                std::optional<vsg::ref_ptr<ImGUIInterface>> imgui_interface;
            };

            Window(Info& info);

            ~Window();

            bool run_once();

            void run();

            using tranverser = std::function<void(const std::unordered_map<std::string, vsg::ref_ptr<Renderable>>&)>;

            inline void transverse_identifiers(tranverser&& transv) {
                std::lock_guard<std::mutex> g{mut};
                transv(contained_objects);
            }

            bool erase(const std::string& identifier);

            bool erase(vsg::ref_ptr<Renderable> renderable);

            friend Window& operator<<(Window& ref, vsg::ref_ptr<Renderable> renderable);
        };

    }
}

#endif