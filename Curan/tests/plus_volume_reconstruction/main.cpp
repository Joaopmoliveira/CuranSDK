#include "dummy_server.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include <atomic>

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
    curan::renderable::Window & window;
    asio::io_context& context;
    std::atomic<bool> stop_service = false;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context) : window{in_window},context{in_context} {}
};

void process_transform_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    //std::cout << "received transform message" << std::endl;
    igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(received_transform);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    if(shared_state.texture==std::nullopt)
        return;
    assert(shared_state.texture!=std::nullopt);
    igtl::Matrix4x4 local_mat;
	transform_message->GetMatrix(local_mat);
    vsg::dmat4 transformmat;

    local_mat[0][3] = local_mat[0][3]*1e-3;
    local_mat[1][3] = local_mat[0][3]*1e-3;
    local_mat[2][3] = local_mat[0][3]*1e-3;

    igtl::PrintMatrix(local_mat);

    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat(col,row) = local_mat[row][col];
    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat* vsg::rotate(vsg::radians(90.0),1.0,0.0,0.0));
}

using imageType = itk::Image<unsigned char,3>;
void updateBaseTexture3D(vsg::vec4Array2D& image, OutputImageType::Pointer image_to_render)
{
    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(image_to_render, image_to_render->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        imageType::IndexType idx = outputIt.GetIndex();
        size_t x = idx[0];
        size_t y = idx[1];
        image.set(x,y,vsg::vec4(outputIt.Get(),outputIt.Get(),outputIt.Get(),1.0));
    }
}

void process_image_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    std::cout << "received image message" << std::endl;
    igtl::ImageMessage::Pointer imageMessage = igtl::ImageMessage::New();
	imageMessage->Copy(received_transform);
	int c = imageMessage->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return ; //failed to unpack message or the texture is not set yet, therefore returning without doing anything
    if(!shared_state.texture){
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = height;
        infotexture.width = width;
        infotexture.builder = vsg::Builder::create();
        infotexture.geomInfo.dx = vsg::vec3(0.00024*height,0.0,0.0);
        infotexture.geomInfo.dy = vsg::vec3(0.0,0.00024*width,0.0);
        infotexture.geomInfo.dz = vsg::vec3(0.0,0.0,0.0);
        infotexture.geomInfo.position = vsg::vec3(0.0,0.0,0.0);
        shared_state.texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state.window << *shared_state.texture;
    }
    assert(shared_state.texture!=std::nullopt);
    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_texture([imageMessage](vsg::vec4Array2D& image)
    {
        int x,y,z =0;
        imageMessage->GetDimensions(x,y,z);
        ImportFilterType::SizeType size;
        size[0] = x;
        size[1] = y;
        size[2] = z; 
 
        ImportFilterType::IndexType start;
        start.Fill(0);
 
        ImportFilterType::RegionType region;
        region.SetIndex(start);
        region.SetSize(size);
        auto importFilter = ImportFilterType::New();

        importFilter->SetRegion(region);
        const itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0, 0.0 };
        importFilter->SetOrigin(origin);
        const itk::SpacePrecisionType spacing[Dimension] = { 1.0, 1.0, 1.0 };
        importFilter->SetSpacing(spacing);

        const bool importImageFilterWillOwnTheBuffer = false;
        importFilter->SetImportPointer(static_cast<unsigned char*>(imageMessage->GetScalarPointer()), imageMessage->GetImageSize(), importImageFilterWillOwnTheBuffer);

        auto filter = FilterType::New();
        filter->SetInput(importFilter->GetOutput());

        using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
        auto rescale = RescaleType::New();
        rescale->SetInput(filter->GetOutput());
        rescale->SetOutputMinimum(0.0);
        rescale->SetOutputMaximum(1.0);

        try{
            rescale->Update();
        } catch (const itk::ExceptionObject& e) {
            std::cerr << "Error: " << e << std::endl;
            return;
        }

        updateBaseTexture3D(image,rescale->GetOutput());
    }
    );
}

std::map<std::string,std::function<void(SharedState& shared_state,igtl::MessageBase::Pointer)>> functions{
    {"TRANSFORM",process_transform_message},
    {"IMAGE",process_image_message}
};

void bar(SharedState& shared_state,size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val) {
	std::cout << "received message\n";
	assert(val.IsNotNull());
	if (er){
        shared_state.context.stop();
        return;
    } 
    if (auto search = functions.find(val->GetMessageType()); search != functions.end())
        search->second(shared_state,val);
    else
        std::cout << "No functionality for function received\n";
    return;
}

void connect(curan::renderable::Window& window,asio::io_context& io_context,SharedState& shared_state){
    
    try{
        asio::io_context io_context;
        unsigned short port = 18944;
        //std::thread server_thread{[&](){foo(port,io_context,server_running);}}; 
    	curan::communication::interface_igtl igtlink_interface;
	    curan::communication::Client::Info construction{ io_context,igtlink_interface };
	    asio::ip::tcp::resolver resolver(io_context);
	    auto endpoints = resolver.resolve("localhost", std::to_string(port));
	    construction.endpoints = endpoints;
	    curan::communication::Client client{ construction };
        
	    auto connectionstatus = client.connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val)
        {
            bar(shared_state,protocol_defined_val,er,val);
            if(shared_state.stop_service)
                client.get_socket().close();
        });
	    io_context.run();
        std::cout << "io context stopped running" << std::endl;
        //server_thread.join();
	    return ;
    }  catch(std::exception & e){
        std::cout << "communication failure: " << e.what() << std::endl;
        return;
    }
};

int main(){
    asio::io_context io_context;
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
        SharedState shared_state = SharedState{window,io_context}; 
        std::thread connector{[&](){connect(window,io_context,shared_state);}};
        window.run();
        std::cout << "stopping content from main thread" << std::endl;
        shared_state.stop_service = true;
        connector.join();
        return 0;
    }  catch(std::exception & e){
        std::cout << "display failure: " << e.what() << std::endl;
        io_context.stop();
        return 1;
    }
}