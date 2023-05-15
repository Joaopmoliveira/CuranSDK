#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/Slider.h"
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

struct ConfigurationData {
	int port = 18944;

	std::array<double, 2> minimum_radius_limit = {5.0,10.0};
	std::array<double, 2> maximum_radius_limit = {11.0,30.0};
	std::array<double, 2> sweep_angle_limit = {0.1,0.8};
	std::array<double, 2> sigma_gradient_limit = {1.0,20.0};
	std::array<double, 2> variance_limit = {1.0,20.0};
	std::array<double, 2> disk_ratio_limit = {0.1,10.0};
	std::array<double, 2>  threshold_limit = {50.0,150.0};

	double minimum_radius = 8;
	double maximum_radius = 12.0;
	double sweep_angle = 0.1;
	double sigma_gradient = 10;
	double variance = 10;
	double disk_ratio = 1;
	double threshold = 110;

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

				auto lam = [localImage, x, y](SkPixmap& requested) {
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

	void attempt_stop() {
		io_context.stop();
	}
};


std::shared_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing) {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };

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

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SK_ColorTRANSPARENT);

		const char* fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		Container::InfoLinearContainer infocontainer;
		infocontainer.paint_layout = paint_square2;
		infocontainer.arrangement = curan::ui::Arrangement::HORIZONTAL;

		Slider::Info infor{};
		infor.click_color = SK_ColorLTGRAY;
		infor.hover_color = SK_ColorCYAN;
		infor.waiting_color = SK_ColorDKGRAY;
		infor.sliderColor = SK_ColorGRAY;
		infor.paintButton = paint_square;
		infor.size = SkRect::MakeWH(200, 40);
		infor.limits = { 0.0f, 300.0f };
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.minimum_radius = processing->configuration.minimum_radius_limit[0] + slider->get_current_value()*(processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
		};
		std::shared_ptr<Slider> button = Slider::make(infor);
		double current_val = (processing->configuration.minimum_radius - processing->configuration.minimum_radius_limit[0]) / (processing->configuration.minimum_radius_limit[1] - processing->configuration.minimum_radius_limit[0]);
		button->set_current_value(current_val);

		TextBlob::Info infotext;
		infotext.button_text = "Minimum Radius";
		infotext.paint = paint_square;
		infotext.paintText = paint_text;
		infotext.size = SkRect::MakeWH(200, 40);
		infotext.textFont = text_font;
		std::shared_ptr<TextBlob> text = TextBlob::make(infotext);

		infocontainer.layouts = { text,button };
		std::shared_ptr<Container> container = Container::make(infocontainer);

		infotext.button_text = "Maximum Radius";
		std::shared_ptr<TextBlob> text1 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.maximum_radius = processing->configuration.maximum_radius_limit[0] + slider->get_current_value() * (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
		};
		std::shared_ptr<Slider> button1 = Slider::make(infor);
		double current_val1 = (processing->configuration.maximum_radius - processing->configuration.maximum_radius_limit[0]) / (processing->configuration.maximum_radius_limit[1] - processing->configuration.maximum_radius_limit[0]);
		button1->set_current_value(current_val1);
		infocontainer.layouts = { text1,button1 };
		std::shared_ptr<Container> container1 = Container::make(infocontainer);

		infotext.button_text = "Sweep Angle";
		std::shared_ptr<TextBlob> text2 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.sweep_angle = processing->configuration.sweep_angle_limit[0] + slider->get_current_value() * (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
		};
		std::shared_ptr<Slider> button2 = Slider::make(infor);
		double current_val2 = (processing->configuration.sweep_angle - processing->configuration.sweep_angle_limit[0]) / (processing->configuration.sweep_angle_limit[1] - processing->configuration.sweep_angle_limit[0]);
		button2->set_current_value(current_val2);
		infocontainer.layouts = { text2,button2 };
		std::shared_ptr<Container> container2 = Container::make(infocontainer);

		infotext.button_text = "Sigma Gradient";
		std::shared_ptr<TextBlob> text3 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.sigma_gradient = processing->configuration.sigma_gradient_limit[0] + slider->get_current_value() * (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
		};
		std::shared_ptr<Slider> button3 = Slider::make(infor);
		double current_val3 = (processing->configuration.sigma_gradient - processing->configuration.sigma_gradient_limit[0]) / (processing->configuration.sigma_gradient_limit[1] - processing->configuration.sigma_gradient_limit[0]);
		button3->set_current_value(current_val3);
		infocontainer.layouts = { text3,button3 };
		std::shared_ptr<Container> container3 = Container::make(infocontainer);

		infotext.button_text = "Variance";
		std::shared_ptr<TextBlob> text4 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.variance = processing->configuration.variance_limit[0] + slider->get_current_value() * (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
		};
		std::shared_ptr<Slider> button4 = Slider::make(infor);
		double current_val4 = (processing->configuration.variance - processing->configuration.variance_limit[0]) / (processing->configuration.variance_limit[1] - processing->configuration.variance_limit[0]);
		button4->set_current_value(current_val4);
		infocontainer.layouts = { text4,button4 };
		std::shared_ptr<Container> container4 = Container::make(infocontainer);

		infotext.button_text = "Disk Ratio";
		std::shared_ptr<TextBlob> text5 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.disk_ratio = processing->configuration.disk_ratio_limit[0] + slider->get_current_value() * (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
		};
		std::shared_ptr<Slider> button5 = Slider::make(infor);
		double current_val5 = (processing->configuration.disk_ratio - processing->configuration.disk_ratio_limit[0]) / (processing->configuration.disk_ratio_limit[1] - processing->configuration.disk_ratio_limit[0]);
		button5->set_current_value(current_val5);
		infocontainer.layouts = { text5,button5 };
		std::shared_ptr<Container> container5 = Container::make(infocontainer);

		infotext.button_text = "Threshold";
		std::shared_ptr<TextBlob> text6 = TextBlob::make(infotext);
		infor.callback = [&processing](Slider* slider, ConfigDraw* config) {
			processing->configuration.threshold = (double)processing->configuration.threshold_limit[0] + slider->get_current_value() * (processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
		};
		std::shared_ptr<Slider> button6 = Slider::make(infor);
		double current_val6 = (processing->configuration.threshold - processing->configuration.threshold_limit[0]) / (double)(processing->configuration.threshold_limit[1] - processing->configuration.threshold_limit[0]);
		button6->set_current_value(current_val6);
		infocontainer.layouts = { text6,button6 };
		std::shared_ptr<Container> container6 = Container::make(infocontainer);

		infocontainer.arrangement = curan::ui::Arrangement::VERTICAL;
		infocontainer.layouts = { container,container1,container2,container3,container4,container5,container6 };
		std::shared_ptr<Container> containerotions = Container::make(infocontainer);

		Overlay::Info information;
		information.backgroundcolor = SK_ColorTRANSPARENT;
		information.contained = containerotions;
		return Overlay::make(information);
}

std::shared_ptr<curan::ui::Page> create_main_page(ConfigurationData& data,std::shared_ptr<ProcessingMessage>& processing) {
	using namespace curan::ui;
	IconResources resources{ "C:/dev/Curan/resources" };

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

	processing = std::make_shared<ProcessingMessage>(processed_viwer, open_viwer, flag);
	processing->port = data.port;
	processing->configuration = data;

	auto lam = [processing](Button* button,ConfigDraw* config) {
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

	auto change_recording_status = [processing](Button* button, ConfigDraw* config) {
		auto val = !processing->should_record.load();
		processing->should_record.store(val);
		SkColor color = (val) ? SK_ColorCYAN : SK_ColorBLACK;
		button->set_waiting_color(color);
	};

	infor.button_text = "Data Collection";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorBLACK;
	infor.callback = change_recording_status;
	infor.size = SkRect::MakeWH(200, 80);
	std::shared_ptr<Button> button_start_collection = Button::make(infor);

	infor.button_text = "Options";
	infor.click_color = SK_ColorGRAY;
	infor.hover_color = SK_ColorDKGRAY;
	infor.waiting_color = SK_ColorBLACK;
	infor.size = SkRect::MakeWH(200, 80);

	infor.callback = [&processing](Button* button, ConfigDraw* config) {
		auto overlay = create_options_overlay(processing);
		config->stack_page->stack(overlay);
	};
	std::shared_ptr<Button> button_options = Button::make(infor);

	info.layouts = { start_connection,button_start_collection,button_options };
	std::shared_ptr<Container> button_container = Container::make(info);

	processing->button = start_connection;
	processing->button_start_collection = button_start_collection;
	start_connection->set_waiting_color(SK_ColorRED);

	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.1 , 1.0 };
	info.layouts = { button_container,viwers_container };
	std::shared_ptr<Container> container = Container::make(info);

	Page::Info information;
	information.backgroundcolor = SK_ColorBLACK;
	information.contained = container;
	std::shared_ptr<Page> page = Page::make(information);
	return page;
}


int main(int argc, char* argv[]) {
	using namespace curan::ui;
	curan::utils::initialize_thread_pool(10);

	ConfigurationData data;
	std::cout << "the received port is: " << data.port << "\n";
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),2200,1800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(data,processing);

	auto rec = viewer->get_size();
	page->propagate_size_change(rec);

	int width = rec.width();
	int height = rec.height();

	ConfigDraw config{page.get()};

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
			page->propagate_signal(signals.back(), &config);
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