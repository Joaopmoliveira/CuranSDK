#ifndef CURAN_IMGUI_INTERFACE_RENDERABLE_HEADER_FILE_
#define CURAN_IMGUI_INTERFACE_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/Texture.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

namespace curan{
namespace renderable{

struct ImGUIInterface : public vsg::Inherit<vsg::Command, ImGUIInterface> {

    using im_gui_callable = std::function<void(vsg::CommandBuffer& cb)>;

    struct Info {
        im_gui_callable callable;

        Info(im_gui_callable in_callable) : callable{in_callable}{

        }
    };

    im_gui_callable callable;
            
    ImGUIInterface(Info& info) ;

    void compile(vsg::Context& context) override;

    void record(vsg::CommandBuffer& cb) const override;

    static vsg::ref_ptr<ImGUIInterface> make(Info& info);
};


}
}
#endif