#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"

std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic>& current, Eigen::Matrix<double, 3, Eigen::Dynamic>& previous, double threshold) {
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distance_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(current.cols(), previous.cols());
	for (int distance_row = 0; distance_row < current.cols(); ++distance_row) {
		Eigen::Matrix<double, 3, 1> current_col = current.col(distance_row);
		for (int distance_col = 0; distance_col < previous.cols(); ++distance_col) {
			Eigen::Matrix<double, 3, 1> previous_col = previous.col(distance_col);
			distance_matrix(distance_row, distance_col) = (previous_col - current_col).norm();
		}
	}
	Eigen::Matrix<double, 3, Eigen::Dynamic> local_copy = previous;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Index minIndex;
	std::vector<size_t> index_mapping;
	index_mapping.resize(previous.cols());
	size_t ordered_indices = 0;
	for (const auto& row : distance_matrix.colwise()) {
		double cost = row.minCoeff(&minIndex);
		index_mapping[ordered_indices] = minIndex;
		local_copy.col(ordered_indices) = current.col(minIndex); 
		if (cost > threshold) {
			return std::nullopt;
		}
		++ordered_indices;
	}

	std::sort(index_mapping.begin(),index_mapping.end());
	for(int i = 0; i < index_mapping.size() - 1; i++) {
    	if (index_mapping[i] == index_mapping[i + 1]) {
        	return std::nullopt;
    	}
	}
	return local_copy;
}

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
	
	auto input_image = DeepCopy<ImageType>(importFilter->GetOutput());

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

	using FilterType = itk::ThresholdImageFilter<ImageType>;
	auto filter = FilterType::New();
	unsigned char lowerThreshold = (unsigned int)processor->configuration.threshold;
	unsigned char upperThreshold = 255;
	filter->SetInput(resampleFilter->GetOutput());
	filter->ThresholdOutside(lowerThreshold, upperThreshold);
	filter->SetOutsideValue(0);

	using RescaleTypeToFloat = itk::RescaleIntensityImageFilter<ImageType, FloatImageType>;
	auto rescaletofloat = RescaleTypeToFloat::New();
	rescaletofloat->SetInput(filter->GetOutput());
	rescaletofloat->SetOutputMinimum(0.0);
	rescaletofloat->SetOutputMaximum(1.0);

	using FilterTypeBlur = itk::DiscreteGaussianImageFilter<FloatImageType, FloatImageType>;
	auto blurfilter = FilterTypeBlur::New();
	blurfilter->SetInput(rescaletofloat->GetOutput());
	blurfilter->SetVariance(processor->configuration.variance);
	blurfilter->SetMaximumKernelWidth(7);

	using RescaleTypeToImageType = itk::RescaleIntensityImageFilter<FloatImageType, ImageType>;
	auto rescaletochar = RescaleTypeToImageType::New();
	rescaletochar->SetInput(blurfilter->GetOutput());
	rescaletochar->SetOutputMinimum(0);
	rescaletochar->SetOutputMaximum(255);

	using AccumulatorPixelType = unsigned int;
	using RadiusPixelType = double;
	using AccumulatorImageType = itk::Image<AccumulatorPixelType, Dimension>;
	
	using HoughTransformFilterType =
		itk::HoughTransform2DCirclesImageFilter<PixelType,
		AccumulatorPixelType,
		RadiusPixelType>;
	auto houghFilter = HoughTransformFilterType::New();
	if (processor->list_size() == 0){
		houghFilter->SetNumberOfCircles(processor->number_of_circles);
	} else {
		houghFilter->SetNumberOfCircles(processor->number_of_circles_plus_extra);	
	}
	houghFilter->SetMinimumRadius(processor->configuration.minimum_radius);
	houghFilter->SetMaximumRadius(processor->configuration.maximum_radius);
	houghFilter->SetSweepAngle(processor->configuration.sweep_angle);
	houghFilter->SetSigmaGradient(processor->configuration.sigma_gradient);
	houghFilter->SetVariance(processor->configuration.variance);
	houghFilter->SetDiscRadiusRatio(processor->configuration.disk_ratio);
	houghFilter->SetInput(rescaletochar->GetOutput());

	using RescaleType = itk::RescaleIntensityImageFilter<AccumulatorImageType, ImageType>;
	auto rescale = RescaleType::New();
	rescale->SetInput(houghFilter->GetOutput());
	rescale->SetOutputMinimum(0);
	rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

    auto to_interpolator = itk::LinearInterpolateImageFunction<ImageType, double>::New();
    auto to_resampleFilter = itk::ResampleImageFilter<ImageType, ImageType>::New();
    to_resampleFilter->SetInput(rescale->GetOutput());
    to_resampleFilter->SetTransform(transform);
    to_resampleFilter->SetInterpolator(interpolator);
    to_resampleFilter->SetOutputDirection(importFilter->GetDirection());
    to_resampleFilter->SetSize(size);
    to_resampleFilter->SetOutputSpacing(spacing);
    to_resampleFilter->SetOutputOrigin(origin);

	try {
		to_resampleFilter->Update();
	}
	catch (...) {
		std::printf("failure\n");
		return false;
	}

	HoughTransformFilterType::CirclesListType circles;
	circles = houghFilter->GetCircles();

	using CirclesListType = HoughTransformFilterType::CirclesListType;
	ObservationEigenFormat observation_n;
	CirclesListType::const_iterator itCircles = circles.begin();
	Eigen::Matrix<double, 3, Eigen::Dynamic> segmented_wires;
	if (processor->list_size() == 0){
		segmented_wires = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, processor->number_of_circles);
	} else {
		segmented_wires = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, processor->number_of_circles_plus_extra);
	}
	size_t circle_index = 0;
	while (itCircles != circles.end())
	{
		Eigen::Matrix<double, 3, 1> segmented_point = Eigen::Matrix<double, 3, 1>::Zero();
		const HoughTransformFilterType::CircleType::PointType centerPoint =(*itCircles)->GetCenterInObjectSpace();
		segmented_point(0, 0) = 2.0*centerPoint[0];
		segmented_point(1, 0) = 2.0*centerPoint[1];
		segmented_wires.col(circle_index) = segmented_point;
		itCircles++;
		++circle_index;
	}

	igtl::Matrix4x4 local_mat;
	message_body->GetMatrix(local_mat);
	for (size_t cols = 0; cols < 4; ++cols)
		for (size_t lines = 0; lines < 4; ++lines)
			observation_n.flange_data(lines, cols) = local_mat[lines][cols];

	if(processor->should_record){
		if(processor->list_size()){ //there are already points on the list
			auto possible_arrangement = rearrange_wire_geometry(segmented_wires, processor->list_back().segmented_wires,processor->threshold);
			if (possible_arrangement) {
				observation_n.segmented_wires = *possible_arrangement;
				segmented_wires = observation_n.segmented_wires;
				processor->list_push_back(observation_n);
			} else{
				std::printf("possible arrangement failure \n");
			}
		} else { // there are no points on the list
			observation_n.segmented_wires = segmented_wires;
			processor->list_push_back(observation_n);
		}
	}

	if (processor->show_circles.load()) {
		auto local_colors = processor->colors;
		ImageType::Pointer post_processed_image = to_resampleFilter->GetOutput();
    	ImageType::SizeType size_itk =  post_processed_image->GetLargestPossibleRegion().GetSize();
    	auto buff = curan::utilities::CaptureBuffer::make_shared(post_processed_image->GetBufferPointer(),size_itk[0]*size_itk[1]*sizeof(char),post_processed_image);
    	curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
		auto special_custom = [x = 2.0*output_size[0],y = 2.0*output_size[1],processor,segmented_wires, local_colors](SkCanvas* canvas, SkRect image_area,SkRect allowed_area) {
			float scalling_factor_x = image_area.width()/x;
			float scalling_factor_y = image_area.height()/y;
			float radius = 5;
			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SK_ColorGREEN);

			auto coliter = local_colors.begin();
			if(!processor->should_record || processor->list_size()==0){
				for (const auto& circles : segmented_wires.colwise()) {
					float xloc = image_area.x()+ scalling_factor_x * circles(0, 0);
					float yloc = image_area.y()+ scalling_factor_y * circles(1, 0);
					SkPoint center{xloc,yloc};
					paint_square.setColor(*coliter);
					canvas->drawCircle(center,radius, paint_square);
					++coliter;
				}
			} else{
				for (const auto& circles : processor->list_back().segmented_wires.colwise()) {
					float xloc = image_area.x()+ scalling_factor_x * circles(0, 0);
					float yloc = image_area.y()+ scalling_factor_y * circles(1, 0);
					SkPoint center{xloc,yloc};
					paint_square.setColor(*coliter);
					canvas->drawCircle(center,radius, paint_square);
					++coliter;
				}
			}
		};
		processor->processed_viwer->update_batch(special_custom,wrapper);
	}
	else {
		processor->processed_viwer->clear_custom_drawingcall();
		ImageType::Pointer post_processed_image = to_resampleFilter->GetOutput();
    	ImageType::SizeType size_itk =  post_processed_image->GetLargestPossibleRegion().GetSize();
    	auto buff = curan::utilities::CaptureBuffer::make_shared(post_processed_image->GetBufferPointer(),size_itk[0]*size_itk[1]*sizeof(char),post_processed_image);
    	curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
		processor->processed_viwer->update_image(wrapper);
	}
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
	{
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points.clear();
		
	}
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