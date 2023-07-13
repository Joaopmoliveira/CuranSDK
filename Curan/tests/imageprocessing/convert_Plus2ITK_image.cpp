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
#include <iostream>
#include <map>
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "imageprocessing/VolumeReconstructor.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;

using OutputPixelType = float;
using InputImageType = itk::Image<unsigned char, 3>;
using OutputImageType = itk::Image<OutputPixelType, 3>;
using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;

struct SharedState{
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> texture;
    vsg::ref_ptr<curan::renderable::Renderable> volume;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context, vsg::ref_ptr<curan::renderable::Renderable> in_volume) : window{in_window},context{in_context}, volume{in_volume}
    {}
};


void plus2ITK_im_convert(const igtl::ImageMessage::Pointer& imageMessage, OutputImageType::Pointer& image_to_render, vsg::dmat4& transformmat) {
    ImportFilterType::RegionType region;
    int width, height, depth = 0;
    imageMessage->GetDimensions(width,height,depth);
    ImportFilterType::SizeType size;
        size[0] = width;
        size[1] = height;
        size[2] = depth;
    region.SetSize(size);
     
    ImportFilterType::IndexType start;
    start.Fill(0);
    region.SetIndex(start);

    auto importFilter = ImportFilterType::New();
    importFilter->SetRegion(region);

    float xx, yy, zz = 0.0;
    imageMessage->GetOrigin(xx, yy, zz);
    const itk::SpacePrecisionType origin[Dimension] = { xx,yy,zz};
    importFilter->SetOrigin(origin);

    float xxx, yyy, zzz = 0.0;
    imageMessage->GetSpacing(xxx, yyy, zzz);
    const itk::SpacePrecisionType spacing[Dimension] = {xxx,yyy,zzz};
    importFilter->SetSpacing(spacing);


    const bool importImageFilterWillOwnTheBuffer = false;
    importFilter->SetImportPointer(static_cast<unsigned char*>(imageMessage->GetScalarPointer()), imageMessage->GetImageSize(), importImageFilterWillOwnTheBuffer);

    auto filter = FilterType::New();
    filter->SetInput(importFilter->GetOutput());

    try{
        filter->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        return;
    }

    image_to_render = filter->GetOutput();

    igtl::Matrix4x4 local_mat;
    imageMessage->GetMatrix(local_mat);

    igtl::PrintMatrix(local_mat);

    local_mat[0][3] = local_mat[0][3]*1e-3;
    local_mat[1][3] = local_mat[1][3]*1e-3;
    local_mat[2][3] = -local_mat[2][3]*1e-3;


    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat(col,row) = local_mat[row][col];

};


void updateBaseTexture3D(vsg::vec4Array2D& image, OutputImageType::Pointer image_to_render)
{
    using OutputPixelType = float;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using FilterType = itk::CastImageFilter<OutputImageType, OutputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
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
    vsg::dmat4 transformmat;
    igtl::ImageMessage::Pointer imageMessage = igtl::ImageMessage::New();
    imageMessage->Copy(received_transform);
	int c = imageMessage->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    plus2ITK_im_convert(imageMessage, image_to_render, transformmat);
    if(!shared_state.texture){
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = height;
        infotexture.width = width;
        infotexture.builder = vsg::Builder::create();
        infotexture.spacing = {0.00024,0.00024,1};
        infotexture.origin = {0.0,0.0,0.0};
        shared_state.texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state.window << *shared_state.texture;
    }
    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat);
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