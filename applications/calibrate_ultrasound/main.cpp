#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/SingletonIconResources.h"
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

struct ProcessingMessage {

	std::shared_ptr<curan::ui::ImageDisplay> processed_viwer;
	std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer;
	std::shared_ptr<curan::utils::Flag> connection_status;
	std::shared_ptr<curan::ui::Button> button;
	asio::io_context io_context;
	std::list<std::vector<Point>> list_of_recorded_points;
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

				curan::image::filtering::ImportFilter::ImportFilterType::SizeType size;
				size[0] = x; // size along X
				size[1] = y; // size along Y
				curan::image::filtering::ImportFilter::ImportFilterType::IndexType start;
				start.Fill(0);

				curan::image::filtering::ImportFilter::Info info_intro;
				info_intro.buffer = (unsigned char*)message_body->GetScalarPointer();
				info_intro.memory_owner = false;
				info_intro.number_of_pixels = x * y;
				info_intro.origin = { 0.0, 0.0 };
				info_intro.size = size;
				info_intro.spacing = { 1.0, 1.0 };
				info_intro.start = start;
				auto import_filer = curan::image::filtering::ImportFilter::make(info_intro);

				curan::image::filtering::CircleFilter::Info info;
				info.number_of_wires = 9;
				info.variance = 10;
				info.sigma_gradient = 10;
				info.min_radius = 5;
				info.disk_radius_ratio = 10;
				info.max_radius = 8;
				auto circle = curan::image::filtering::CircleFilter::make(info);

				curan::image::filtering::Filter filter;
				filter << import_filer;
				filter << circle;

				auto hough_filter = circle->get_filter();
				try {
					hough_filter->Update();
				}
				catch (...) {
					return false;
				}
				
				auto image = import_filer->get_output();

				
				using OutputPixelType = unsigned char;
				using OutputImageType = itk::Image<OutputPixelType, 2>;
				OutputImageType::Pointer localOutputImage = OutputImageType::New();
				OutputImageType::RegionType region;
				region.SetSize(image->GetLargestPossibleRegion().GetSize());
				region.SetIndex(image->GetLargestPossibleRegion().GetIndex());
				localOutputImage->SetRegions(region);
				localOutputImage->SetOrigin(image->GetOrigin());
				localOutputImage->SetSpacing(image->GetSpacing());
				localOutputImage->Allocate(true);

				auto circles = hough_filter->GetCircles();

				std::vector<Point> local_centers;
				local_centers.reserve(circles.size());

				using CirclesListType = curan::image::filtering::CircleFilter::HoughTransformFilterType::CirclesListType;
				CirclesListType::const_iterator itCircles = circles.begin();
				OutputImageType::IndexType localIndex;
				while (itCircles != circles.end())
				{
					const curan::image::filtering::CircleFilter::HoughTransformFilterType::CircleType::PointType centerPoint =
						(*itCircles)->GetCenterInObjectSpace();
					Point p;
					p.x = centerPoint[0];
					p.y = centerPoint[1];
					local_centers.push_back(p);
					for (double angle = 0; angle <= itk::Math::twopi;
						angle += itk::Math::pi / 60.0)
					{

						using IndexValueType = OutputImageType::IndexType::IndexValueType;
						localIndex[0] = itk::Math::Round<IndexValueType>(
							centerPoint[0] +
							(*itCircles)->GetRadiusInObjectSpace()[0] * std::cos(angle));
						localIndex[1] = itk::Math::Round<IndexValueType>(
							centerPoint[1] +
							(*itCircles)->GetRadiusInObjectSpace()[0] * std::sin(angle));
						OutputImageType::RegionType outputRegion =
							localOutputImage->GetLargestPossibleRegion();

						if (outputRegion.IsInside(localIndex))
						{
							localOutputImage->SetPixel(localIndex, 255);
						}
					}
					itCircles++;
				}

				list_of_recorded_points.push_back(local_centers);

				auto lam = [localOutputImage,x,y](SkPixmap& requested) {
					auto inf = SkImageInfo::Make(x, y, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
					size_t row_size = x * sizeof(char);
					SkPixmap map{ inf,localOutputImage->GetBufferPointer(),row_size };
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
		button->update_color(SK_ColorGREEN);
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
		button->update_color(SK_ColorRED);
		return;
	}

	void attempt_stop() {
		io_context.stop();	
	}
};



int main(int argc, char* argv[]) {
	using namespace curan::ui;
	curan::utils::initialize_thread_pool(10);
	if (argc != 2) {
		std::cout << "the ultrasound calibration app only parses one argument, the port of the server to connect to\n";
		return 1;
	}

	std::string val = { argv[1] };
	size_t pos = 0;
	int port = 0;
	try{
		port = std::stoi(val, &pos);
	} catch (...) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 3;
	}
	if (pos != val.size()) {
		std::cout << "the parsed port is not valid, please try again\n";
		return 2;
	}

	std::cout << "the received port is: " << port << "\n";

	IconResources* resources = IconResources::Load("C:/dev/Curan/resources");
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

	SkFont text_font = SkFont(typeface, 20, 1.0f, 0.0f);
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
	std::shared_ptr<Container> container2 = Container::make(info);

	auto flag = curan::utils::Flag::make_shared_flag();

	std::shared_ptr<ProcessingMessage> processing = std::make_shared<ProcessingMessage>(processed_viwer, open_viwer,flag);
	processing->port = port;

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

	Button::Info infor;
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
	std::shared_ptr<Button> button = Button::make(infor);

	processing->button = button;
	button->update_color(SK_ColorRED);

	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.1 , 1.0 };
	info.layouts = { button,container2 };
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
	for (const auto& f : s)
		std::cout << " s :" << f;
	std::cout << "\n";
	return 0;
}