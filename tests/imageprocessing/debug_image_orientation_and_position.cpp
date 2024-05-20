#include "imageprocessing/StaticReconstructor.h"
#include <optional>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Sphere.h"


int main(){
try{
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

    curan::renderable::DynamicTexture::Info infotexture;
    infotexture.height = 100;
    infotexture.width = 100;
    infotexture.builder = vsg::Builder::create();
    infotexture.spacing = {0.02,0.02,0.02};
    infotexture.origin = {0.0,0.0,0.0};
    auto texture = curan::renderable::DynamicTexture::make(infotexture);
    window << texture;

    texture->cast<curan::renderable::DynamicTexture>()->update_texture([](vsg::vec4Array2D& image){
        for(size_t y = 0 ; y < image.height(); ++y)
            for(size_t x = 0 ; x < image.width(); ++x){
                auto val = std::sqrt(y*y+x*x)/std::sqrt(image.height()*image.height()+image.width()*image.width());
                image.set(x,y,vsg::vec4(val,val,val,1.0));
            }      
    });

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.02f,0.0,0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0,0.02f,0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0,0.0,0.02f);
    infosphere.stateInfo.blending = true;
    auto sphere = curan::renderable::Sphere::make(infosphere);
    auto mat = vsg::translate(0.0,0.0,0.0);
    sphere->update_transform(mat);
    window << sphere;

    infosphere.geomInfo.color = vsg::vec4(0.0,1.0,0.0,1.0);

    auto sphere1 = curan::renderable::Sphere::make(infosphere);
    auto mat1 = vsg::translate(1.0,0.0,0.0);
    sphere1->update_transform(mat1);
    window << sphere1;

    infosphere.geomInfo.color = vsg::vec4(0.0,0.0,1.0,1.0);

    auto sphere2 = curan::renderable::Sphere::make(infosphere);
    auto mat2 = vsg::translate(0.0,1.0,0.0);
    sphere2->update_transform(mat2);
    window << sphere2;

    window.run();
    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });
	return 0;
}
catch(std::exception& e){
    std::cout << "exception was thrown: " << e.what() << std::endl;
    return 0;
}
}