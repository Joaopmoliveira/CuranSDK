#include "imageprocessing/StaticReconstructor.h"
#include <optional>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include "Mathematics/MinimumVolumeBox3.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Box.h"
#include "dummy_classic.h"

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

void updateBaseTexture3D(vsg::floatArray3D& image, curan::image::StaticReconstructor::output_type::Pointer image_to_render)
{
    using OutputPixelType = float;
    using InputImageType = itk::Image<unsigned char, 3>;
    using OutputImageType = itk::Image<OutputPixelType, 3>;
    using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;
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
        curan::image::StaticReconstructor::output_type::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}

void process_image_message(SharedState& shared_state,igtl::MessageBase::Pointer received_transform){
    //std::cout << "received image message" << std::endl;
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
        infotexture.spacing = {0.00024,0.00024,1};
        infotexture.origin = {0.0,0.0,0.0};
        shared_state.texture = curan::renderable::DynamicTexture::make(infotexture);
        shared_state.window << *shared_state.texture;
        std::cout << "Height: " << height << "      Width: " << width << std::endl;
    }
    assert(shared_state.texture!=std::nullopt);
    igtl::Matrix4x4 local_mat;
    imageMessage->GetMatrix(local_mat);
    vsg::dmat4 transformmat;
    //shared_state.reconstructor.add_frame();

    //std::printf("(%4.2f) (%4.2f) (%4.2f) \n",local_mat[0][3],local_mat[1][3],local_mat[2][3]);

    local_mat[0][3] = local_mat[0][3]*1e-3;
    local_mat[1][3] = local_mat[1][3]*1e-3;
    local_mat[2][3] = local_mat[2][3]*1e-3;



    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat(col,row) = local_mat[row][col];

    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat);
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

        try{
            filter->Update();
        } catch (const itk::ExceptionObject& e) {
            std::cerr << "Error: " << e << std::endl;
            return;
        }
        updateBaseTexture3D(image,filter->GetOutput());
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
        std::cout << "No functionality for function received\n";
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

void volume_update_operation(SharedState& shared_state){
    while(!shared_state.context.stopped()){
        if(shared_state.reconstructor.update() && shared_state.texture){
            auto buffer = shared_state.reconstructor.get_output_pointer();
            auto updater = [buffer](vsg::floatArray3D& image){
                updateBaseTexture3D(image, buffer);
            };
            shared_state.volume->cast<curan::renderable::Volume>()->update_volume(updater);
        };
    };
}

constexpr long width = 50;
constexpr long height = 50;
float spacing[3] = {0.02 , 0.02 , 0.02};
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

    curan::renderable::Box::Info infobox;
    infobox.builder = vsg::Builder::create();
    infobox.geomInfo.color = vsg::vec4(1.0,0.0,0.0,0.5);
    infobox.geomInfo.dx = vsg::vec3(.121f,0.0,0.0);
    infobox.geomInfo.dy = vsg::vec3(0.0,.146f,0.0);
    infobox.geomInfo.dz = vsg::vec3(0.0,0.0,.271f);
    infobox.geomInfo.position = vsg::vec3( 0.0606,0.0731,0.1349);
    auto box = curan::renderable::Box::make(infobox);
    vsg::dmat4 mat = vsg::translate(0.0,0.0,0.0);
    mat[0] = vsg::vec4(0.9490,-0.0330,-0.3136,0.0);
    mat[1] = vsg::vec4(0.0136,0.9979,-0.0638,0.0);
    mat[2] = vsg::vec4(0.3150,0.0562,0.9474,0.0);
    mat[3] = vsg::vec4(0.0750,-0.0500,-1.1710,1.0);
    box->update_transform(mat);
    window << box;

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

    std::array<double,3> vol_origin = {0.0,0.0,0.0};
	std::array<double,3> vol_spacing = {final_spacing[0],final_spacing[1],final_spacing[2]};
	std::array<double,3> vol_size = {1.0,1.0,1.0};
	std::array<std::array<double,3>,3> vol_direction;
	vol_direction[0] = {1.0,0.0,0.0};
	vol_direction[1] = {0.0,1.0,0.0};
	vol_direction[2] = {0.0,0.0,1.0};
	curan::image::StaticReconstructor::Info recon_info{vol_spacing,vol_origin,vol_size,vol_direction};

    SharedState shared_state = SharedState{window,io_context,recon_info,volume}; 
    std::thread connector{[&](){connect(window,io_context,shared_state);}};
    std::thread volume_updater{[&](){volume_update_operation(shared_state);}};
    window.run();
    std::cout << "stopping content from main thread" << std::endl;
    io_context.stop();
    connector.join();
	return 0;
};