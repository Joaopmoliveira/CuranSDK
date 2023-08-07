#include "imageprocessing/StaticReconstructor.h"
#include <optional>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Box.h"
#include <iostream>
#include <map>
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "imageprocessing/VolumeReconstructor.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"
#include <vsg/all.h>

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;

using OutputPixelType = unsigned char;
using InputImageType = itk::Image<unsigned char, 3>;
using OutputImageType = itk::Image<OutputPixelType, 3>;
using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;

struct SharedState{
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> texture;
    curan::image::BoundingBox4Reconstruction box_class;
    vsg::ref_ptr<curan::renderable::Renderable> caixa;
    vsg::ref_ptr<curan::renderable::Renderable> volume;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context, vsg::ref_ptr<curan::renderable::Renderable> in_volume) : window{in_window},context{in_context}, volume{in_volume}
    {}
};


void updateBaseTexture3D(vsg::vec4Array2D& image, OutputImageType::Pointer image_to_render)
{
    using OutputPixelType = unsigned char;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using InputPixelType = float;
    using InputImageType = itk::Image<InputPixelType, 3>;
    using FilterType = itk::CastImageFilter<OutputImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<InputImageType, OutputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try{
        rescale->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    OutputImageType::Pointer out = rescale->GetOutput();
    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        OutputImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0],idx[1],vsg::vec4(outputIt.Get(),outputIt.Get(),outputIt.Get(),1.0));
    }
}

void process_image_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    OutputImageType::Pointer image_to_render;
    igtl::ImageMessage::Pointer imageMessage = igtl::ImageMessage::New();
    imageMessage->Copy(received_transform);
	int c = imageMessage->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything

    curan::image::igtl2ITK_im_convert(imageMessage, image_to_render);
    const itk::SpacePrecisionType spacing[3] = {0.0001852,0.0001852,0.0001852};
    image_to_render->SetSpacing(spacing);
    auto origin = image_to_render->GetOrigin();
    origin[0] *= 1e-3;
    origin[1] *= 1e-3;
    origin[2] *= 1e-3;
    image_to_render->SetOrigin(origin);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    shared_state.box_class.add_frame(image_to_render);
    shared_state.box_class.update();
    auto caixa = shared_state.box_class.get_final_volume_vertices();

    std::chrono::steady_clock::time_point elapsed_for_bound_box = std::chrono::steady_clock::now();
    auto val_elapsed_for_bound_box = (int)std::chrono::duration_cast<std::chrono::microseconds>(elapsed_for_bound_box - begin).count();
    //std::printf("added image - elapsed time: %d microseconds\n",val_elapsed_for_bound_box);


    //igtl2ITK_im_convert(imageMessage, image_to_render);
    if(!shared_state.texture){
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = height;
        infotexture.width = width;
        infotexture.builder = vsg::Builder::create();
        infotexture.spacing = {0.0001852,0.0001852,1.0};
        infotexture.origin = {0.0,0.0,0.0};
        shared_state.texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state.window << *shared_state.texture;

        curan::renderable::Box::Info infobox;
        infobox.builder = vsg::Builder::create();
        infobox.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
        infobox.geomInfo.dx = vsg::vec3(1.0f,0.0,0.0);
        infobox.geomInfo.dy = vsg::vec3(0.0,1.0f,0.0);
        infobox.geomInfo.dz = vsg::vec3(0.0,0.0,1.0f);
        infobox.stateInfo.wireframe = true;
        infobox.geomInfo.position = vsg::vec3(0.5,0.5,0.5);
        shared_state.caixa = curan::renderable::Box::make(infobox);
        shared_state.window << shared_state.caixa;
    }

    vsg::dmat4 transform_matrix;

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row)
            transform_matrix(col,row) = image_to_render->GetDirection()[row][col];

    transform_matrix(3,0) = image_to_render->GetOrigin()[0];
    transform_matrix(3,1) = image_to_render->GetOrigin()[1];
    transform_matrix(3,2) = image_to_render->GetOrigin()[2];
    
    vsg::dmat3 rotation_0_1;

    vsg::dmat4 box_transform_matrix = vsg::translate(0.0,0.0,0.0);

    for(size_t col = 0; col < 3; ++col)
        for(size_t row = 0; row < 3; ++row){
            box_transform_matrix(col,row) = caixa.axis[col][row];
            rotation_0_1(col,row) = box_transform_matrix(col,row);
        }

    vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = caixa.center[0];
    position_of_center_in_global_frame[1] = caixa.center[1];
    position_of_center_in_global_frame[2] = caixa.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = caixa.extent[0];
    position_in_local_box_frame[1] = caixa.extent[1];
    position_in_local_box_frame[2] = caixa.extent[2]; 

    auto global_corner_position = position_of_center_in_global_frame-rotation_0_1*position_in_local_box_frame;

    box_transform_matrix(3,0) = global_corner_position[0];
    box_transform_matrix(3,1) = global_corner_position[1];
    box_transform_matrix(3,2) = global_corner_position[2];
    box_transform_matrix(3,3) = 1;

    //std::cout << "centro caixa: " << caixa.extent[0] << std::endl;;
    //std::printf("extent (%f %f %f)\n",caixa.extent[0],caixa.extent[1],caixa.extent[2]);
    
    
    shared_state.caixa->cast<curan::renderable::Box>()->set_scale(caixa.extent[0]*2,caixa.extent[1]*2,caixa.extent[2]*2);
    //shared_state.caixa->cast<curan::renderable::Box>()->set_scale(1e-1,1e-1,1e-1);
    shared_state.caixa->update_transform(box_transform_matrix);

    //std::cout << "tranf matrix: \n" << box_transform_matrix << std::endl;

    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transform_matrix);
    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_texture([image_to_render](vsg::vec4Array2D& image)
    {
        updateBaseTexture3D(image,image_to_render);
    }
    );
}

std::map<std::string,std::function<void(SharedState& shared_state,igtl::MessageBase::Pointer)>> functions{
    {"IMAGE",process_image_message}
};

void bar(SharedState& shared_state,size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	//std::cout << "received message\n";
	assert(val.IsNotNull());
	if (er){
        shared_state.context.stop();
        return;
    } 
    if (auto search = functions.find(val->GetMessageType()); search != functions.end())
        search->second(shared_state,val);
    else
        //std::cout << "No functionality for function received\n";
    return;
}


void connect(curan::renderable::Window& window,asio::io_context& io_context,SharedState& shared_state){
    
    try{
        unsigned short port = 18944; 
    	curan::communication::interface_igtl igtlink_interface;
	    curan::communication::Client::Info construction{ io_context,igtlink_interface };
	    asio::ip::tcp::resolver resolver(io_context);
	    auto endpoints = resolver.resolve("localhost", std::to_string(port));
	    construction.endpoints = endpoints;
	    curan::communication::Client client{ construction };

	    auto connectionstatus = client.connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val)
        {
            bar(shared_state,protocol_defined_val,er,val);
        });
	    io_context.run();
        std::cout << "io context stopped running" << std::endl;
	    return ;
    }  catch(std::exception & e){
        std::cout << "communication failure: " << e.what() << std::endl;
        return;
    }
};

float final_spacing [3] = {0.02 ,0.02, 0.02};
float final_size[3] = {50,50,50};

int main(){
    asio::io_context io_context;
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

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = final_size[0]; 
    volumeinfo.height = final_size[1];
    volumeinfo.depth = final_size[2];
    volumeinfo.spacing_x = final_spacing[0]*1e3;
    volumeinfo.spacing_y = final_spacing[1]*1e3;
    volumeinfo.spacing_z = final_spacing[2]*1e3;
    auto volume = curan::renderable::Volume::make(volumeinfo);
	auto casted_volume = volume->cast<curan::renderable::Volume>();
    window << volume;

    SharedState shared_state = SharedState{window,io_context,volume}; 
    std::thread connector{[&](){connect(window,io_context,shared_state);}};
    window.run();
    std::cout << "stopping content from main thread" << std::endl;
    io_context.stop();
    connector.join();
	return 0;
};