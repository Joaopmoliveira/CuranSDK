#include "rendering/integrated_shaders/VolumeIntegratedShaders.h"
#include "imageprocessing/TemplatedVolumeAlgorithms.h"
#include "rendering/IntegratedVolumeReconstructor.h"

namespace curan{
namespace renderable {

IntegratedReconstructor::Info::Info(std::array<double,3> inspacing,std::array<double,3> inorigin, std::array<double,3> insize, std::array<std::array<double,3>,3> indirection){
	auto inextent = gte::Vector3<double>{insize[0]/2.0,insize[1]/2.0,insize[2]/2.0};

    auto origin = gte::Vector3<double>{inorigin[0],inorigin[1],inorigin[2]};
	std::array<gte::Vector3<double>, 3> alignement;
	//alignement[0] = {indirection[0][0],indirection[0][1],indirection[0][2]};
	//alignement[1] = {indirection[1][0],indirection[1][1],indirection[1][2]};
	//alignement[2] = {indirection[2][0],indirection[2][1],indirection[2][2]};

    //auto output_origin = origin
	//	+ alignement[0] * inextent[0]
//		+ alignement[1] * inextent[1]
//		+ alignement[2] * inextent[2];

  //  volumetric_bounding_box = gte::OrientedBox3<double>{output_origin,alignement,inextent};
   // spacing[0] = in_spacing[0];
   // spacing[1] = in_spacing[1];
   // spacing[2] = in_spacing[2];
}

vsg::ref_ptr<Renderable> IntegratedReconstructor::make(Info& info){
    vsg::ref_ptr<IntegratedReconstructor> sphere_to_add = IntegratedReconstructor::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

IntegratedReconstructor::IntegratedReconstructor(const Info& info){

}

IntegratedReconstructor::~IntegratedReconstructor(){

}

IntegratedReconstructor& IntegratedReconstructor::set_interpolation(const curan::image::reconstruction::Interpolation& new_interpolation_strategy){
	std::lock_guard<std::mutex> g{mut};
    interpolation_strategy = new_interpolation_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_compound(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_fillstrategy(const curan::image::reconstruction::Compounding& new_compounding_strategy){
	std::lock_guard<std::mutex> g{mut};
    compounding_strategy = new_compounding_strategy;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::set_clipping(const Clipping& new_clipping){
	std::lock_guard<std::mutex> g{mut};
    clipping = new_clipping;
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frame(output_type::Pointer image_pointer){
	std::lock_guard<std::mutex> g{mut};
	frame_data.push_back(image_pointer);
    return *(this);
}

IntegratedReconstructor& IntegratedReconstructor::add_frames(std::vector<output_type::Pointer>& images_vector){
	std::lock_guard<std::mutex> g{mut};
    frame_data.insert(std::end(frame_data), std::begin(images_vector), std::end(images_vector));
    return *(this);
}

IntegratedReconstructor::output_type::Pointer IntegratedReconstructor::get_output_pointer(){
    return out_volume;
}

bool IntegratedReconstructor::update(){
    return true;
}

bool IntegratedReconstructor::multithreaded_update(std::shared_ptr<utilities::ThreadPool>pool){
    return true;
}

}
}