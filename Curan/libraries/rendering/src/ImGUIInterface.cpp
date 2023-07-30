#include "renderable/ImGUIInterface.h"

namespace curan{
namespace renderable{
            
ImGUIInterface::ImGUIInterface(Info& info) : callable{info.callable}{

}

void ImGUIInterface::compile(vsg::Context& context)
{

}

void ImGUIInterface::record(vsg::CommandBuffer& cb)
{
    callable(cb);
}

static vsg::ref_ptr<ImGUIInterface> ImGUIInterface::make(Info& info){
    return ImGUIInterface::create(info);
}

}
}