#include "imageprocessing/IntegratedVolumeReconstructor.h"
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
        infotexture.spacing = {0.0001852,0.0001852,1};
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
    local_mat[2][3] = -local_mat[2][3]*1e-3;



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
	    asio::ip::tcp::resolver resolver(io_context);
	    auto client = curan::communication::Client<protocols::igtlink>::make(io_context,resolver.resolve("localhost", std::to_string(port)));

	    client->connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val)
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

/*

void volume_update_operation(SharedState& shared_state){
    while(!shared_state.context.stopped()){
        if(shared_state.reconstructor.update() && shared_state.texture){
            auto buffer = shared_state.reconstructor.get_output_pointer();
        };
    };
}

*/

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

    SharedState shared_state = SharedState{window,io_context}; 
    std::thread connector{[&](){connect(window,io_context,shared_state);}};
    window.run();
    std::cout << "stopping content from main thread" << std::endl;
    io_context.stop();
    connector.join();
	return 0;
};