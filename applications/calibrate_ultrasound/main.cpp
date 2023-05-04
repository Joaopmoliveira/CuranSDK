#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/Job.h"
#include "utils/TheadPool.h"
#include "imageprocessing/FilterAlgorithms.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <iostream>


struct Point {
	double x;
	double y;
};

float s[3];


struct ConfigurationData {
	int port = 18944;
	double minimum_radius = 8;
	double maximum_radius = 10;
	double sweep_angle = 0.06;
	double sigma_gradient = 10;
	double variance = 10;
	double disk_ratio = 1;
	unsigned char threshold = 110;

};

struct ProcessingMessage {

	std::shared_ptr<curan::ui::ImageDisplay> processed_viwer;
	std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer;
	std::shared_ptr<curan::utils::Flag> connection_status;
	std::shared_ptr<curan::ui::Button> button;
	std::shared_ptr<curan::ui::Button> button_start_collection;
	asio::io_context io_context;
	ConfigurationData configuration;
	std::list<std::vector<Point>> list_of_recorded_points;
	std::atomic<bool> should_record = false;
	short port = 10000;

	ProcessingMessage(std::shared_ptr<curan::ui::ImageDisplay> in_processed_viwer, 
					  std::shared_ptr<curan::ui::OpenIGTLinkViewer> in_open_viwer, 
					  std::shared_ptr<curan::utils::Flag> flag) : connection_status{ flag }, processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer } 
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
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

				message_body->GetSpacing(s);

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
				unsigned char lowerThreshold = configuration.threshold;
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

				using RescaleTypeToImageType = itk::RescaleIntensityImageFilter<FloatImageType,ImageType>;
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

				ImageType::Pointer localImage = rescale->GetOutput();
				//HoughTransformFilterType::CirclesListType circles;
				//circles = houghFilter->GetCircles();
				//using OutputPixelType = unsigned char;
				//using OutputImageType = itk::Image<OutputPixelType, Dimension>;

				//auto localOutputImage = OutputImageType::New();
				//region.SetSize(localImage->GetLargestPossibleRegion().GetSize());
				//region.SetIndex(localImage->GetLargestPossibleRegion().GetIndex());
				//localOutputImage->SetRegions(region);
				//localOutputImage->SetOrigin(localImage->GetOrigin());
				//localOutputImage->SetSpacing(localImage->GetSpacing());
				//localOutputImage->Allocate(true); // initializes buffer to zero

				//using CirclesListType = HoughTransformFilterType::CirclesListType;
				//CirclesListType::const_iterator itCircles = circles.begin();

				//std::vector<Point> local_centers;
				//local_centers.reserve(circles.size());
				//while (itCircles != circles.end())
				//{
				//	const HoughTransformFilterType::CircleType::PointType centerPoint =
				//		(*itCircles)->GetCenterInObjectSpace();
				//	Point p;
				//	p.x = centerPoint[0];
				//	p.y = centerPoint[1];
				//	local_centers.push_back(p);
				//	for (double angle = 0; angle <= itk::Math::twopi;
				//		angle += itk::Math::pi / 60.0)
				//	{
				//		using IndexValueType = ImageType::IndexType::IndexValueType;
				//		localIndex[0] = itk::Math::Round<IndexValueType>(
				//			centerPoint[0] +
				//			(*itCircles)->GetRadiusInObjectSpace()[0] * std::cos(angle));
				//		localIndex[1] = itk::Math::Round<IndexValueType>(
				//			centerPoint[1] +
				//			(*itCircles)->GetRadiusInObjectSpace()[0] * std::sin(angle));
				//		OutputImageType::RegionType outputRegion =
				//			localOutputImage->GetLargestPossibleRegion();

				//		if (outputRegion.IsInside(localIndex))
				//		{
				//			localOutputImage->SetPixel(localIndex, 255);
				//		}
				//	}
				//	itCircles++;
				//}

				//if (should_record.load() && local_centers.size()>0) {
				//	list_of_recorded_points.push_back(local_centers);
				//}

				auto lam = [localImage,x,y](SkPixmap& requested) {
					auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
					size_t row_size = x * sizeof(unsigned char);
					SkPixmap map{ inf,localImage->GetBufferPointer(),row_size };
					requested = map;
					return;
				};
				processed_viwer->update_image(lam);
			}
		}
		else {
			std::cout << "Unknown Message\n";
		}
		return false;
	};

	void communicate() {
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
			if (process_message(protocol_defined_val,er,val))
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

	void attempt_stop() {
		io_context.stop();	
	}
};


int parse_arguments(const int& argc, char* argv[], ConfigurationData& data) {
	if (argc != 9) { //-port -minimum_radius -maximum_radius -sweep_angle -sigma_gradient -variance -disk_ratio -threshold
		std::cout << "the ultrasound calibration app parses nine arguments: \n"
			"- the port of the server to connect to\n"
			"- the minimum radius\n"
			"- maximum radius\n"
			"- the sweep angle\n"
			"- the sigma gradient\n"
			"- variance\n"
			"- disk_ratio\n"
			"- threshold\n"
			"and the disk ratio\n"
			"If the nine are not supplied default values will be used";
		return 1;
	}

	std::string val = { argv[1] }; //port
	size_t pos = 0;
	int port = 0;
	try {
		port = std::stoi(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 2;
	}

	val = { argv[2] }; //minimum radius
	double minimum_radius = 0;
	try {
		minimum_radius = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed minimum radius is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed minimum radius is not valid, please try again\n";
		return 2;
	}

	val = { argv[3] }; //maximum radius
	double maximum_radius = 0;
	try {
		maximum_radius = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed maximum radius is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed maximum radius is not valid, please try again\n";
		return 2;
	}

	val = { argv[4] }; //sweep angle
	double sweep_angle = 0;
	try {
		sweep_angle = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed sweep angle is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed sweep angle is not valid, please try again\n";
		return 2;
	}

	val = { argv[5] }; //sigma gradient 
	double sigma_gradient = 0;
	try {
		sigma_gradient = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed sigma gradient is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed sigma gradient is not valid, please try again\n";
		return 2;
	}

	val = { argv[6] }; //variance
	double variance = 0;
	try {
		variance = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed variance is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed variance is not valid, please try again\n";
		return 2;
	}

	val = { argv[7] }; //disk ratio
	double disk_ratio = 0;
	try {
		disk_ratio = std::stod(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed disk ratio is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed disk ratio is not valid, please try again\n";
		return 2;
	}

	val = { argv[8] }; //threshold
	int threshold = 0;
	try {
		threshold = std::stoi(val, &pos);
	}
	catch (...) {
		std::cout << "the parsed threshold is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed threshold is not valid, please try again\n";
		return 2;
	}

	data.port = port;
	data.disk_ratio = disk_ratio;
	data.maximum_radius = maximum_radius;
	data.minimum_radius = minimum_radius;
	data.sigma_gradient = sigma_gradient;
	data.sweep_angle = sweep_angle;
	data.variance = variance;
	data.threshold = (unsigned char)threshold;
	return 0;
}


int main(int argc, char* argv[]) {
	using namespace curan::ui;
	curan::utils::initialize_thread_pool(10);

	ConfigurationData data;
	parse_arguments(argc,argv,data);
	std::cout << "the received port is: " << data.port << "\n";

	IconResources resources{ "C:/dev/Curan/resources" };
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),2200,1800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	SkColor colbuton = { SK_ColorBLACK };
	SkColor coltext = { SK_ColorWHITE };

	SkPaint paint_square;
	paint_square.setStyle(SkPaint::kFill_Style);
	paint_square.setAntiAlias(true);
	paint_square.setStrokeWidth(4);
	paint_square.setColor(colbuton);

	SkPaint paint_text;
	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(coltext);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

	SkFont text_font = SkFont(typeface, 15, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SK_ColorBLACK);

	OpenIGTLinkViewer::Info infoviewer;
	infoviewer.text_font = text_font;
	infoviewer.size = SkRect::MakeWH(0, 0);
	std::shared_ptr<OpenIGTLinkViewer> open_viwer = OpenIGTLinkViewer::make(infoviewer);

	ImageDisplay::Info processed_viwer_info;
	processed_viwer_info.height = 0;
	processed_viwer_info.width = 0;
	std::shared_ptr<ImageDisplay> processed_viwer = ImageDisplay::make(processed_viwer_info);

	Container::InfoLinearContainer info;
	info.paint_layout = paint_square2;
	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.5 , 1.0 };
	info.layouts = { open_viwer,processed_viwer };
	std::shared_ptr<Container> viwers_container = Container::make(info);

	auto flag = curan::utils::Flag::make_shared_flag();

	std::shared_ptr<ProcessingMessage> processing = std::make_shared<ProcessingMessage>(processed_viwer, open_viwer,flag);
	processing->port = data.port;
	processing->configuration = data;

	auto lam = [processing]() {
		if (!processing->connection_status->value()) {
			curan::utils::Job val;
			val.description = "connection thread";
			val.function_to_execute = [processing]() {
				processing->communicate();
			};
			curan::utils::pool->submit(val);
		}
		else {
			processing->attempt_stop();
		}
	};

	Button::Info infor{ resources };
	infor.button_text = "Connect";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorRED;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeWH(100, 80);
	infor.textFont = text_font;
	infor.callback = lam;
	std::shared_ptr<Button> start_connection = Button::make(infor);

	auto change_recording_status = [processing]() {
		auto val = !processing->should_record.load();
		processing->should_record.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		processing->button_start_collection->set_waiting_color(color);
	};

	infor.button_text = "Data Collection";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorBLACK;
	infor.callback = change_recording_status;
	infor.size = SkRect::MakeWH(200, 80);
	std::shared_ptr<Button> button_start_collection = Button::make(infor);

	info.layouts = { start_connection,button_start_collection };
	std::shared_ptr<Container> button_container = Container::make(info);

	processing->button = start_connection;
	processing->button_start_collection = button_start_collection;
	start_connection->set_waiting_color(SK_ColorRED);

	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.1 , 1.0 };
	info.layouts = { button_container,viwers_container };
	std::shared_ptr<Container> container = Container::make(info);
	
	auto rec = viewer->get_size();
	Page::Info information;
	information.backgroundcolor = SK_ColorBLACK;
	information.contained = container;
	std::shared_ptr<Page> page = Page::make(information);
	page->propagate_size_change(rec);
	
	int width = rec.width();
	int height = rec.height();

	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		auto temp_height = pointer_to_surface->height();
		auto temp_width = pointer_to_surface->width();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (temp_height != height || temp_width != width) {
			rec = SkRect::MakeWH(temp_width, temp_height);
			page->propagate_size_change(rec);
		}
		page->draw(canvas);
		auto signals = viewer->process_pending_signals();
		if (!signals.empty())
			page->propagate_signal(signals.back());
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	processing->attempt_stop();
	std::cout << "trying to stop everything" << std::endl;

	int tasks_n = 0;
	int tasks_queue = 0;
	curan::utils::pool->get_number_tasks(tasks_n, tasks_queue);
	std::cout << "Number of tasks executing: " << tasks_n << " number of tasks in queue" << tasks_queue << "\n";
	curan::utils::terminate_thread_pool();
	std::cout << "Number of frame recordings: " << processing->list_of_recorded_points.size() << "\n";
	std::cout << "Received spacing: \n";
	
	int counter_f = 1;
	for (const auto& f : processing->list_of_recorded_points) {
		int counter_p = 0;
		std::cout << "slice : ";
		for (const auto& p : f) {
			std::cout << "	point(" << counter_p << ") -> (" << p.x << ", " << p.y << ")\n";
			++counter_p;
		}
		++counter_f;
	}

	std::cout << "\n";
	return 0;
}