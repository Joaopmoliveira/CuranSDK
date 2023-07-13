#include "rendering/IntegratedVolumeReconstructor.h"

namespace curan{
namespace renderable {

IntegratedReconstructor::Info(std::array<double,3> spacing,std::array<double,3> origin, std::array<double,3> size, std::array<std::array<double,3>,3> direction){

}

vsg::ref_ptr<Renderable> IntegratedReconstructor::make(Info& info){

}

IntegratedReconstructor::IntegratedReconstructor(const Info& info){

}

IntegratedReconstructor::~IntegratedReconstructor(){

}

IntegratedReconstructor& IntegratedReconstructor::set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){

}

IntegratedReconstructor& IntegratedReconstructor::set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){

}

IntegratedReconstructor& IntegratedReconstructor::set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy){

}

IntegratedReconstructor& IntegratedReconstructor::set_clipping(const Clipping& new_clipping){

}

IntegratedReconstructor& IntegratedReconstructor::add_frame(output_type::Pointer image_pointer){

}

IntegratedReconstructor& IntegratedReconstructor::add_frames(std::vector<output_type::Pointer>& images_vector){

}

output_type::Pointer IntegratedReconstructor::get_output_pointer(){

}

bool IntegratedReconstructor::update(){

}

bool IntegratedReconstructor::multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool){
    
}

}
}