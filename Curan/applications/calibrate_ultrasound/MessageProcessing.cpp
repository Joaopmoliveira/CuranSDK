#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"

bool PotentialObservationEigenFormat::is_complete(ObservationEigenFormat& observation) {
	bool val = flange_data.has_value() && segmented_wires.has_value();
	if (!val)
		return val;
	observation.flange_data = *flange_data;
	observation.segmented_wires = *segmented_wires;
	flange_data = std::nullopt;
	segmented_wires = std::nullopt;
	return val;
}

void PotentialObservationEigenFormat::set_flange_data(Eigen::Matrix<double, 4, 4> flange_dat) {
	flange_data = flange_dat;
}

void PotentialObservationEigenFormat::set_segmented_wires(Eigen::Matrix<double, 3, Eigen::Dynamic> in_segmented_wires) {
	segmented_wires = in_segmented_wires;
}

std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic>& current, Eigen::Matrix<double, 3, Eigen::Dynamic>& previous, double threshold) {
	assert(current.cols() == previous.cols());
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distance_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(previous.cols(), previous.cols());
	for (size_t distance_row = 0; distance_row < previous.cols(); ++distance_row) {
		Eigen::Matrix<double, 3, 1> current_col = current.col(distance_row);
		for (size_t distance_col = 0; distance_col < previous.cols(); ++distance_col) {
			Eigen::Matrix<double, 3, 1> previous_col = previous.col(distance_col);
			distance_matrix(distance_row, distance_col) = (previous_col - current_col).norm();
		}
	}
	Eigen::Matrix<double, 3, Eigen::Dynamic> local_copy = current;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Index   minIndex;
	size_t ordered_indices = 0;
	for (const auto& row : distance_matrix.rowwise()) {
		double cost = row.minCoeff(&minIndex);
		local_copy.col(ordered_indices) = current.col(minIndex);
		if (cost > threshold) {
			return std::nullopt;
		}
		++ordered_indices;
	}
	return local_copy;
}

bool process_transform_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	processor->open_viwer->process_message(val);
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
}

bool process_image_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	processor->open_viwer->process_message(val);
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything

	igtl::Matrix4x4 local_mat;
	message_body->GetMatrix(local_mat);
	Eigen::Matrix<double, 4, 4> flange_data;
	for (size_t cols = 0; cols < 4; ++cols)
		for (size_t lines = 0; lines < 4; ++lines)
			flange_data(lines, cols) = local_mat[lines][cols];
	processor->observation_to_propagete.set_flange_data(flange_data);

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

	using FilterType = itk::ThresholdImageFilter<ImageType>;
	auto filter = FilterType::New();
	unsigned char lowerThreshold = (unsigned int)processor->configuration.threshold;
	unsigned char upperThreshold = 255;
	filter->SetInput(importFilter->GetOutput());
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
	blurfilter->SetMaximumKernelWidth(10);

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

	houghFilter->SetNumberOfCircles(processor->number_of_circles);
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

	try {
		rescale->Update();
	}
	catch (...) {
		return false;
	}

	HoughTransformFilterType::CirclesListType circles;
	circles = houghFilter->GetCircles();

	using CirclesListType = HoughTransformFilterType::CirclesListType;
	CirclesListType::const_iterator itCircles = circles.begin();
	Eigen::Matrix<double, 3, Eigen::Dynamic> local_segmented_wires = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, processor->number_of_circles);
	size_t circle_index = 0;
	while (itCircles != circles.end())
	{
		Eigen::Matrix<double, 3, 1> segmented_point = Eigen::Matrix<double, 3, 1>::Zero();
		const HoughTransformFilterType::CircleType::PointType centerPoint =
			(*itCircles)->GetCenterInObjectSpace();
		segmented_point(0, 0) = centerPoint[0];
		segmented_point(1, 0) = centerPoint[1];
		local_segmented_wires.col(circle_index) = segmented_point;
		itCircles++;
		++circle_index;
	}

	if (processor->list_of_recorded_points.size() == 0) {
		processor->observation_to_propagete.set_segmented_wires(local_segmented_wires);
	}
	else {
		auto possible_arrangement = rearrange_wire_geometry(local_segmented_wires, processor->list_of_recorded_points.back().segmented_wires,processor->threshold);
		if (possible_arrangement) {
			local_segmented_wires = *possible_arrangement;
			processor->observation_to_propagete.set_segmented_wires(*possible_arrangement);
		} else {
			local_segmented_wires = processor->list_of_recorded_points.back().segmented_wires;
		}
	}

	if (processor->show_circles.load()) {
		ImageType::Pointer localImage = importFilter->GetOutput();
		auto lam = [message_body,localImage, x, y](SkPixmap& requested) {
			auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
			size_t row_size = x * sizeof(unsigned char);
			SkPixmap map{ inf,localImage->GetBufferPointer(),row_size };
			requested = map;
			return;
		};
		auto local_colors = processor->colors;
		auto special_custom = [x,y,local_segmented_wires, local_colors](SkCanvas* canvas, SkRect allowed_region) {
			float scalling_factor_x = allowed_region.width()/x;
			float scalling_factor_y = allowed_region.height()/y;
			float radius = 15;
			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SK_ColorGREEN);
			assert(local_segmented_wires.cols() == local_colors.size());
			auto coliter = local_colors.begin();
			for (const auto& circles : local_segmented_wires.colwise()) {
				float xloc = allowed_region.x()+ scalling_factor_x * circles(0, 0);
				float yloc = allowed_region.y()+ scalling_factor_y * circles(1, 0);
				SkPoint center{xloc,yloc};
				paint_square.setColor(*coliter);
				canvas->drawCircle(center,radius, paint_square);
				++coliter;
			}
		};
		processor->processed_viwer->update_batch(special_custom,lam);
	}
	else {
		processor->processed_viwer->clear_custom_drawingcall();
		ImageType::Pointer localImage = rescale->GetOutput();
		auto lam = [localImage, x, y](SkPixmap& requested) {
			auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
			size_t row_size = x * sizeof(unsigned char);
			SkPixmap map{ inf,localImage->GetBufferPointer(),row_size };
			requested = map;
		};
		processor->processed_viwer->update_image(lam);
	}
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
	list_of_recorded_points.clear();
	using namespace curan::communication;
	button->set_waiting_color(SK_ColorGREEN);
	io_context.reset();
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	Client client{ construction };
	connection_status->set();

	auto lam = [this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		if (process_message(protocol_defined_val, er, val))
		{
			connection_status->clear();
			attempt_stop();
		}
	};
	auto connectionstatus = client.connect(lam);
	io_context.run();
	button->set_waiting_color(SK_ColorRED);
	list_of_recorded_points.clear();
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}