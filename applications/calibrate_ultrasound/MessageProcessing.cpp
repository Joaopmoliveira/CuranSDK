#include "MessageProcessing.h"

struct hash_of_previous_segmentations {
	std::optional<std::vector<Point>> previous_points;

	std::vector<Point> compute_distance(std::vector<Point> new_points) {
		if (!previous_points) {
			previous_points = new_points;
			return *previous_points;
		}
		auto prev_points = *previous_points;
		std::vector<std::vector<double>> distance_to_previous_points;
		distance_to_previous_points.reserve(new_points.size()* prev_points.size());
		auto iter = distance_to_previous_points.begin();
		for (const auto& val : prev_points) {
			for (const auto& new_value : new_points) {

			}
		}
		return new_points;
	}
};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
if (er)
	return true;

assert(val.IsNotNull());
std::string tmp = val->GetMessageType();
std::string transform = "TRANSFORM";
std::string image = "IMAGE";
if (!tmp.compare(transform)) {
	open_viwer->process_message(val);
}
else if (!tmp.compare(image)) {
	open_viwer->process_message(val);
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (c & igtl::MessageHeader::UNPACK_BODY) {
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
		unsigned char lowerThreshold = (unsigned int)configuration.threshold;
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
		blurfilter->SetVariance(10);
		blurfilter->SetMaximumKernelWidth(10);

		using RescaleTypeToImageType = itk::RescaleIntensityImageFilter<FloatImageType, ImageType>;
		auto rescaletochar = RescaleTypeToImageType::New();
		rescaletochar->SetInput(blurfilter->GetOutput());
		rescaletochar->SetOutputMinimum(0);
		rescaletochar->SetOutputMaximum(255);

		using AccumulatorPixelType = unsigned int;
		using RadiusPixelType = double;
		ImageType::IndexType localIndex;
		using AccumulatorImageType = itk::Image<AccumulatorPixelType, Dimension>;

		using HoughTransformFilterType =
			itk::HoughTransform2DCirclesImageFilter<PixelType,
			AccumulatorPixelType,
			RadiusPixelType>;
		auto houghFilter = HoughTransformFilterType::New();

		houghFilter->SetNumberOfCircles(3);
		houghFilter->SetMinimumRadius(configuration.minimum_radius);
		houghFilter->SetMaximumRadius(configuration.maximum_radius);
		houghFilter->SetSweepAngle(configuration.sweep_angle);
		houghFilter->SetSigmaGradient(configuration.sigma_gradient);
		houghFilter->SetVariance(configuration.variance);
		houghFilter->SetDiscRadiusRatio(configuration.disk_ratio);
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

		std::vector<Point> local_centers;

		if (show_circles.load()) {

			ImageType::Pointer localImage = importFilter->GetOutput();
			auto lam = [message_body,localImage, x, y](SkPixmap& requested) {
				auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
				size_t row_size = x * sizeof(unsigned char);
				SkPixmap map{ inf,localImage->GetBufferPointer(),row_size };
				requested = map;
				return;
			};
			

			HoughTransformFilterType::CirclesListType circles;
			circles = houghFilter->GetCircles();

			using CirclesListType = HoughTransformFilterType::CirclesListType;
			CirclesListType::const_iterator itCircles = circles.begin();
			local_centers.reserve(circles.size());

			while (itCircles != circles.end())
			{
				const HoughTransformFilterType::CircleType::PointType centerPoint =
					(*itCircles)->GetCenterInObjectSpace();
				Point p;
				p.x = centerPoint[0];
				p.y = centerPoint[1];
				local_centers.push_back(p);

				itCircles++;
			}

			auto special_custom = [=](SkCanvas* canvas, SkRect allowed_region) {
				float scalling_factor_x = allowed_region.width()/x;
				float scalling_factor_y = allowed_region.height()/y;
				float radius = 5;
				SkPaint paint_square;
				paint_square.setStyle(SkPaint::kFill_Style);
				paint_square.setAntiAlias(true);
				paint_square.setStrokeWidth(4);
				paint_square.setColor(SK_ColorGREEN);
				for (const auto& circles : local_centers) {
					float xloc = allowed_region.x()+ scalling_factor_x * circles.x;
					float yloc = allowed_region.y()+ scalling_factor_y*(y-circles.y);
					SkPoint center{xloc,yloc};
					canvas->drawCircle(center,radius, paint_square);
				}
			};
			processed_viwer->update_batch(special_custom,lam);
		}
		else {
			processed_viwer->clear_custom_drawingcall();
			ImageType::Pointer localImage = rescale->GetOutput();
			auto lam = [localImage, x, y](SkPixmap& requested) {
				auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
				size_t row_size = x * sizeof(unsigned char);
				SkPixmap map{ inf,localImage->GetBufferPointer(),row_size };
				requested = map;
			};
			processed_viwer->update_image(lam);
		}

			//if (should_record.load() && local_centers.size()>0) {
			//	list_of_recorded_points.push_back(local_centers);
			//}
		}
	}
	else {
		std::cout << "Unknown Message: " << tmp << "\n";
	}
	return false;
};

void ProcessingMessage::communicate() {
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
	auto val = io_context.run();
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}