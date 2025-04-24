#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"
#include "itkImageFileWriter.h"
#include "itkPNGImageIO.h"

template <typename TImage>
typename TImage::Pointer DeepCopy(typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        outputIterator.Set(inputIterator.Get());
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
}

bool process_transform_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	processor->open_viwer->process_message(val);
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
	return true;
}

bool process_image_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = begin;
	processor->open_viwer->process_message(val);
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything

	int x, y, z;
	message_body->GetDimensions(x, y, z);
	using PixelType = unsigned char;
	constexpr unsigned int Dimension = 2;
	using ImageType = itk::Image<PixelType, Dimension>;

	using FloatImageType = itk::Image<float, Dimension>;
	using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;
	auto importFilter = ImportFilterType::New();

	ImportFilterType::SizeType size;
	size[0] = x;
	size[1] = y;
	ImportFilterType::IndexType start;
	start.Fill(0);
	ImportFilterType::RegionType region;
	region.SetIndex(start);
	region.SetSize(size);

	importFilter->SetRegion(region);
	const itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0 };
	importFilter->SetOrigin(origin);
	const itk::SpacePrecisionType spacing[Dimension] = { 1.0, 1.0 };
	importFilter->SetSpacing(spacing);
	
	const bool importImageFilterWillOwnTheBuffer = false;
	importFilter->SetImportPointer((PixelType*)message_body->GetScalarPointer(), message_body->GetScalarSize(), importImageFilterWillOwnTheBuffer);
	importFilter->Update();

    double physicalspace[2];
    physicalspace[0] = size[0] * spacing[0];
    physicalspace[1] = size[1] * spacing[1];

    auto output_size = size;
    output_size[0] = (size_t)std::floor((1.0 / 2) * output_size[0]);
    output_size[1] = (size_t)std::floor((1.0 / 2) * output_size[1]);

    itk::SpacePrecisionType output_spacing[Dimension];
    output_spacing[0] = physicalspace[0] / output_size[0];
    output_spacing[1] = physicalspace[1] / output_size[1];

    auto interpolator = itk::LinearInterpolateImageFunction<ImageType, double>::New();
    auto transform = itk::AffineTransform<double, 2>::New();
    transform->SetIdentity();
    auto resampleFilter = itk::ResampleImageFilter<ImageType, ImageType>::New();
    resampleFilter->SetInput(importFilter->GetOutput());
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(interpolator);
    resampleFilter->SetOutputDirection(importFilter->GetDirection());
    resampleFilter->SetSize(output_size);
    resampleFilter->SetOutputSpacing(output_spacing);
    resampleFilter->SetOutputOrigin(origin);

	try {
		resampleFilter->Update();
	}
	catch (...) {
		std::printf("failure\n");
		return false;
	}

    igtl::Matrix4x4 image_transform;
    message_body->GetMatrix(image_transform);
   	Eigen::Matrix<double,4,4> homogeneous_transformation = Eigen::Matrix<double,4,4>::Identity();
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            homogeneous_transformation(row, col) = image_transform[row][col];

    homogeneous_transformation(0,3) *= 1e-3;
    homogeneous_transformation(1,3) *= 1e-3;
    homogeneous_transformation(2,3) *= 1e-3;

	Eigen::Matrix<double,4,4> image_pose = homogeneous_transformation*processor->calibration_transformation;

	const Eigen::Matrix<double,3,1> normal_to_plane = image_pose.block<3,1>(0,2);
	const Eigen::Matrix<double,3,1> point_in_plane = image_pose.block<3,1>(0,3);
	size_t index = 1; 
	static size_t frame_index = 1;

	processor->processed_viwer->clear_custom_drawingcall();
	ImageType::Pointer post_processed_image = resampleFilter->GetOutput();
    auto size_itk =  post_processed_image->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(post_processed_image->GetBufferPointer(),size_itk[0]*size_itk[1]*sizeof(char),post_processed_image);
    curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};

	std::vector<Eigen::Matrix<double,3,1>> points_intersecting_image_plane;
	points_intersecting_image_plane.reserve(3);

	for(auto& [rot,offset] : processor->line_parameterization){
		const Eigen::Matrix<double,3,1> along_line = rot.block<3,1>(0,2);
		const Eigen::Matrix<double,3,1> point_in_line = -rot.block<3,1>(0,1)*offset;

		const double t = -(point_in_line - point_in_plane).dot(normal_to_plane) / along_line.dot(normal_to_plane);
		Eigen::Matrix<double,4,1> intersection = Eigen::Matrix<double,4,1>::Ones();
		intersection.block<3,1>(0,0) = point_in_line + t * along_line;
		Eigen::Matrix<double,4,1> local_intersection = image_pose.inverse()*intersection;
		//std::cout << "frame: (" << frame_index <<") line id (" << index << ") intersection: [" << intersection.transpose() << "] local: [" << local_intersection.transpose() <<"]\n";

		const Eigen::Matrix<double,3,1> theta = rot.block<3,1>(0,0);
		const Eigen::Matrix<double,3,1> phi = rot.block<3,1>(0,1);
		double thetha_residue = theta.transpose()*intersection.block<3,1>(0,0);
		double phi_residue = phi.transpose()*intersection.block<3,1>(0,0)+ offset;
		std::cout << "check "<< index <<" *should be zero* : (theta plane " << (abs(thetha_residue) < 1e-10 ? 0.0 : thetha_residue) << ") (phi plane " <<  (abs(phi_residue) < 1e-10 ? 0.0 : phi_residue )<< ") point:";
		Eigen::Matrix<double,3,1> intersection_image_coordinates = (0.5/0.00018867924)*local_intersection.block<3,1>(0,0);
		std::printf(" %.2f %.2f image size [%d %d]\n",intersection_image_coordinates[0],intersection_image_coordinates[1],(int)size_itk[0],(int)size_itk[1]);

		if(intersection_image_coordinates[0]>0 && intersection_image_coordinates[0]<size_itk[0] && 
			intersection_image_coordinates[1]>0 && intersection_image_coordinates[1]<size_itk[1])
			points_intersecting_image_plane.push_back(intersection_image_coordinates);

		++index;
	}
	//std::cout << "number of points: " << points_intersecting_image_plane.size() << std::endl;
	++frame_index;

	auto local_colors = processor->colors;
	processor->processed_viwer->update_batch([=](SkCanvas* canvas, SkRect image_area,SkRect allowed_area){
		float scalling_factor_x = image_area.width()/size_itk[0];
		float scalling_factor_y = image_area.height()/size_itk[1];
		float radius = 5;
		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorGREEN);
		auto coliter = local_colors.begin();
		for (const auto& circles : points_intersecting_image_plane) {
			float xloc = image_area.x()+ scalling_factor_x * circles[0];
			float yloc = image_area.y()+ scalling_factor_y * circles[1];
			SkPoint center{xloc,yloc};
			paint_square.setColor(*coliter);
			canvas->drawCircle(center,radius, paint_square);
			++coliter;
		}
	},
	wrapper);

	if(processor->record_images)
		processor->pool->submit("data recording",[=](){
    		itk::ImageFileWriter<ImageType>::Pointer writer = itk::ImageFileWriter<ImageType>::New();
			std::stringstream ss;
			ss << CURAN_COPIED_RESOURCE_PATH"/ai_training/id_" << frame_index;
			for (const auto& circles : points_intersecting_image_plane) 
				ss << "[" << (int) circles[0] << "," << (int) circles[1]<< "]";
			ss << ".png";
    		writer->SetFileName(ss.str());
			writer->SetInput(post_processed_image);
    		try {
        		writer->Update();
        		std::cout << "Image successfully written to output_image.png!" << std::endl;
    		} catch (itk::ExceptionObject & e) {
        		std::cerr << "Error: " << e.GetDescription() << std::endl;
        		return ;
    		}
		});

	end = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	if(time_elapsed>40)
		std::printf("warning: reduce brightness of image because processing size is too large (%lld milliseconds)\n",time_elapsed);
	return true;
}

std::map<std::string,std::function<bool(ProcessingMessage*,igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"TRANSFORM",process_transform_message},
	{"IMAGE",process_image_message}
};

ProcessingMessage::ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,curan::ui::OpenIGTLinkViewer* in_open_viwer) : processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer }{
	
}

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	//std::cout << "received message\n";
	assert(val.IsNotNull());
	if (er){
        return true;
    } 
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(this,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

void ProcessingMessage::communicate() {
	std::cout << "connecting to server" << std::endl;
	button->set_waiting_color(SK_ColorGREEN);
	io_context.restart();
	asio::ip::tcp::resolver resolver(io_context);
	auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(io_context,resolver.resolve("localhost", std::to_string(port)));
	connection_status.set(true);

	client->connect([this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		try{
			if (process_message(protocol_defined_val, er, val))
			{
				attempt_stop();
			}
		}catch(...){
			std::cout << "Exception was thrown\n";
		}
	});
	io_context.run();
	connection_status.set(false);
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}