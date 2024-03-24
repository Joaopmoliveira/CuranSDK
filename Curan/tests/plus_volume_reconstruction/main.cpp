#include "dummy_server.h"

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
    local_mat[1][3] = local_mat[1][3]*1e-3;
    local_mat[2][3] = -local_mat[2][3]*1e-3;


    for(size_t col = 0; col < 4; ++col)
        for(size_t row = 0; row < 4; ++row)
            transformmat(col,row) = local_mat[row][col];

    shared_state.texture->cast<curan::renderable::DynamicTexture>()->update_transform(transformmat);
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
    }
    assert(shared_state.texture!=std::nullopt);
    shared_state.recorded_images.push_back(imageMessage);
    igtl::Matrix4x4 local_mat;
    imageMessage->GetMatrix(local_mat);
    igtl::PrintMatrix(local_mat);
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
//        std::thread server_thread{[&](){foo(port,io_context);}}; 
    	curan::communication::interface_igtl igtlink_interface;
	    curan::communication::Client::Info construction{ io_context,igtlink_interface };
	    asio::ip::tcp::resolver resolver(io_context);
	    auto endpoints = resolver.resolve("localhost", std::to_string(port));
	    construction.endpoints = endpoints;
	    curan::communication::Client client{ construction };
        
	    client.connect([&](size_t protocol_defined_val,std::error_code er, igtl::MessageBase::Pointer val)
        {
            bar(shared_state,protocol_defined_val,er,val);
        });
	    io_context.run();
        std::cout << "io context stopped running" << std::endl;
//       server_thread.join();
	    return ;
    }  catch(std::exception & e){
        std::cout << "communication failure: " << e.what() << std::endl;
        return;
    }
};

std::vector<curan::image::VolumeReconstructor::output_type::Pointer> get_converted_images(std::vector<igtl::ImageMessage::Pointer> openigtlinkmessages){
    std::vector<curan::image::VolumeReconstructor::output_type::Pointer> output;
    output.reserve(openigtlinkmessages.size());
    for(auto& image : openigtlinkmessages ){
        int x,y,z =0;
        image->GetDimensions(x,y,z);
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
        itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0, 0.0 };

        igtl::Matrix4x4 local_mat;
        image->GetMatrix(local_mat);

        origin[0] = local_mat[0][3]*1e-3;
        origin[1] = local_mat[1][3]*1e-3;
        origin[2] = -local_mat[2][3]*1e-3;

        importFilter->SetOrigin(origin);
        const itk::SpacePrecisionType spacing[Dimension] = { 0.00024, 0.00024, 1.0 };
        importFilter->SetSpacing(spacing);

        
        vsg::dmat4 transformmat;

        local_mat[0][3] = local_mat[0][3]*1e-3;
        local_mat[1][3] = local_mat[1][3]*1e-3;
        local_mat[2][3] = -local_mat[2][3]*1e-3;

        itk::Matrix<double,3,3> direction;

        for(size_t col = 0; col < 3 ; ++col)
            for(size_t row = 0; row < 3 ; ++row)
                direction[row][col] = local_mat[row][col];

        importFilter->SetDirection(direction);

        const bool importImageFilterWillOwnTheBuffer = false;
        importFilter->SetImportPointer(static_cast<unsigned char*>(image->GetScalarPointer()), image->GetImageSize(), importImageFilterWillOwnTheBuffer);

        importFilter->Update();
        output.push_back(importFilter->GetOutput());
    }
    return output;
}

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
        io_context.stop();
        connector.join();

        // Now that everything is finished we can try and compute the volumetric size of our image. 
        // Once this is done I need to change the API of the volume reconstructor into two distinct classes,
        // one which is dynamic and one which is static

        std::vector<curan::image::VolumeReconstructor::output_type::Pointer> vector_of_images = get_converted_images(shared_state.recorded_images);
        std::cout << "the number of images is: " << shared_state.recorded_images.size() << std::endl;
   
        curan::image::VolumeReconstructor reconstructor;

        std::array<double,2> clip_origin = {0.0,0.0};
        std::array<double,2> clip_size = {100,100};
        float spacing[3] = {0.00024 , 0.00024, 0.00024};

	    reconstructor.set_output_spacing(spacing);
	    reconstructor.set_fill_strategy(curan::image::reconstruction::FillingStrategy::GAUSSIAN);
	    reconstructor.set_clipping_bounds(clip_origin, clip_size);
	    reconstructor.add_frames(vector_of_images);

	    std::cout << "Started volumetric reconstruction: \n";
	    reconstructor.update();
	    std::cout << "Finished volumetric reconstruction: \n";

        curan::image::VolumeReconstructor::output_type::Pointer buffer;
	    reconstructor.get_output_pointer(buffer);

        auto sizeout = buffer->GetLargestPossibleRegion().GetSize();
        std::cout << "Size : " << sizeout << std::endl; 
        auto outorigin = buffer->GetOrigin();
        auto direct = buffer->GetDirection();

        std::cout << "Origin : " << outorigin << std::endl;
        std::cout << "Direction : " << direct << std::endl;
        std::cout << "Spacing : " << buffer->GetSpacing() << std::endl;
        return 0;

	    std::cout << "Started volumetric filling: \n";
	    curan::image::reconstruction::KernelDescriptor descript;
	    descript.fillType = curan::image::reconstruction::FillingStrategy::DISTANCE_WEIGHT_INVERSE;
        descript.size = 5;
	    descript.stdev = 1;
	    descript.minRatio = 0.1;
	    reconstructor.add_kernel_descritor(descript);
	    reconstructor.fill_holes();
	    std::cout << "Finished volumetric filling: \n";

        return 0;
    }  catch(std::exception & e){
        std::cout << "display failure: " << e.what() << std::endl;
        return 1;
    }
}