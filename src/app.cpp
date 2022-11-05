#define STB_IMAGE_IMPLEMENTATION
#include <iostream>
#include <deque>
#include <chrono>
#include <thread>
#include "DkLibrary.h"
#include "UserInterface.h"
#include "CkLibrary.h"
#include <iostream>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#include <chrono>
#include "Mathematics/ConvexHull3.h"
#include "Mathematics/ArbitraryPrecision.h"
#include <Mathematics/MinimumVolumeBox3.h>

class TestingWindow : public curan::ui::WindowBody
{

public:
	TestingWindow(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param) : WindowBody(in_context, param, window_identifier)
	{
	};

	void init() override
	{
		std::shared_ptr<curan::display::Page> image_display_page;
		create_segmentation_page(image_display_page);
		viewer->add_page(image_display_page);
		viewer->initialize(context);
		viewer->connect_handler();
	};

	void run() override
	{
		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			viewer->draw(canvas, page_index);
			if (overlay_page_index > 0) {
				auto image = canvas->getSurface()->makeImageSnapshot();
				canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
			}
			viewer->swapBuffers();
			glfwPollEvents();
			viewer->process_pending_signals(page_index);
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		viewer->destroy();
		curan::ui::WindowManager* manager = curan::ui::WindowManager::Get();
		manager->unregister_window(identifier());
	};

	static std::shared_ptr<TestingWindow> make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param)
	{
		return std::make_shared<TestingWindow>(in_context, window_identifier, volume_index, param);
	};

private:

	void create_segmentation_page(std::shared_ptr<curan::display::Page>& page)
	{
		SkColor colbuton = { SK_ColorWHITE };
		SkColor coltext = { SK_ColorBLACK };

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

		SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

		curan::display::RadioButton::Info info_radio;
		info_radio.background_color = SK_ColorBLACK;
		info_radio.color = SK_ColorWHITE;
		info_radio.is_exclusive = false;
		info_radio.options = { "Option 1","Option 2" };
		info_radio.layout = curan::display::RadioButton::RadioButtonLayout::VERTICAL;
		info_radio.size = SkRect::MakeXYWH(0, 0, 200, 200);
		info_radio.text_font = text_font;

		std::shared_ptr<curan::display::RadioButton> radio = curan::display::RadioButton::make(info_radio);

		curan::display::LayoutLinearWidgetContainer::Info info1;
		info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
		info1.paintLayout = paint_square2;
		info1.widgets = { radio };
		std::shared_ptr<curan::display::LayoutLinearWidgetContainer> button_container = curan::display::LayoutLinearWidgetContainer::make(info1);

		curan::display::Page::Info info5;
		info5.page_layout = std::static_pointer_cast<curan::display::Layout>(button_container);
		info5.paint_page = SK_ColorWHITE;
		page = curan::display::Page::make(info5);

		page->monitor(std::static_pointer_cast<curan::display::Widget>(radio));
	};

	/*
		void create_segmentation_page(std::shared_ptr<curan::display::Page>& page)
	{
		SkColor colbuton = { SK_ColorWHITE };
		SkColor coltext = { SK_ColorBLACK };

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

		SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

		curan::display::RadioButton::Info info_radio;
		info_radio.background_color = SK_ColorBLACK;
		info_radio.color = SK_ColorWHITE;
		info_radio.is_exclusive = false;
		info_radio.options = {"Rita","Cu da Rita"};
		info_radio.layout = curan::display::RadioButton::RadioButtonLayout::VERTICAL;
		info_radio.size = SkRect::MakeXYWH(0,0,200,200);
		info_radio.text_font = text_font;

		std::shared_ptr<curan::display::RadioButton> radio = curan::display::RadioButton::make(info_radio);

		curan::display::LayoutLinearWidgetContainer::Info info1;
		info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
		info1.paintLayout = paint_square2;
		info1.widgets = { radio };
		std::shared_ptr<curan::display::LayoutLinearWidgetContainer> button_container = curan::display::LayoutLinearWidgetContainer::make(info1);

		curan::display::Page::Info info5;
		info5.page_layout = std::static_pointer_cast<curan::display::Layout>(button_container);
		info5.paint_page = SK_ColorWHITE;
		page = curan::display::Page::make(info5);

		page->monitor(std::static_pointer_cast<curan::display::Widget>(radio));
	};
	*/

};

void start_up()
{
	curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
	curan::display::IconResources* resources = curan::display::IconResources::Load("C:/dev/Curan/resources");

	curan::image::StudyManager* study_manager = curan::image::StudyManager::Get();
	study_manager->LoadStudies({ "C:/libraries/data/MRBRAIN.DCM" });

	curan::display::Context context;
	context.initialize_context();

	curan::display::DisplayParams param;
	std::shared_ptr<TestingWindow> testing_window = TestingWindow::make(&context, 0, 0, param);
	testing_window->init();
	testing_window->run();
	//curan::ui::WindowManager* manager = curan::ui::WindowManager::Get();

	//manager->set_context(&context);
	//manager->lauch_biopsy_visualizer(0);
	//manager->lauch_medical_visualizer(0);
	//manager->lauch_faiviewer_visualizer(0);
	//manager->process_os_events();

	//curan::communication::ChannelManager* channel_manager = curan::communication::ChannelManager::Get();
	//channel_manager->terminate_all_channels();

	context.destroy_context();

	pool->Shutdown();
};


int main()
{
	try { start_up(); }
	catch (...) {
		curan::utils::console->info("Exception was thrown");
		return 1;
	}

	/*

	curan::pk::StudyManager* study_manager = curan::pk::StudyManager::Get();
	study_manager->LoadStudies({ "C:\\dev\\DICOM_radial_image" });

	curan::pk::Study study;
	study_manager->GetStudy(0, study);

	itk::Vector<double> spacing = study.study_img[0]->GetSpacing();
	itk::ImageRegion region = study.study_img[0]->GetLargestPossibleRegion();
	itk::Size<3> size = region.GetSize();

	std::cout << "Input Spacing: " << spacing[0] << " " << spacing[1] << " " << spacing[2] << std::endl;

	double min_spacing = std::min(std::min(spacing[0], spacing[1]),std::min(spacing[1], spacing[2]));
	itk::Vector<double> new_spacing;
	new_spacing[0] = min_spacing;
	new_spacing[1] = min_spacing;
	new_spacing[2] = min_spacing;

	std::array<double, 2> clipOrigin = { 0,0 };
	std::array<double, 2> clipSize = { size[0],size[1]};

	curan::pk::VolumeReconstructor reconstructor;
	reconstructor.SetOutputSpacing(new_spacing);
	reconstructor.SetFillStrategy(curan::pk::VolumeReconstructor::FillingStrategy::GAUSSIAN);
	reconstructor.SetClippingBounds(clipOrigin, clipSize);
	reconstructor.AddFrames(study.study_img);

	curan::utils::console->info("Started volumetric reconstruction: ");
	reconstructor.Update();
	curan::utils::console->info("Finished volumetric reconstruction: ");

	curan::utils::console->info("Started volumetric filling: ");
	curan::pk::VolumeReconstructor::KernelDescriptor descript;
	descript.fillType = curan::pk::VolumeReconstructor::FillingStrategy::DISTANCE_WEIGHT_INVERSE;
	descript.size = 5;
	descript.stdev = 1;
	descript.minRatio = 0.1;
	reconstructor.AddKernelDescritor(descript);
	reconstructor.FillHoles();
	curan::utils::console->info("Finished volumetric filling: ");


	curan::pk::VolumeReconstructor::OutputType::Pointer buffer;
	reconstructor.GetOutputPointer(buffer);

	auto img_region = buffer->GetLargestPossibleRegion();
	auto output_size = img_region.GetSize();

	std::cout << "Output Size: " << output_size[0] << " " << output_size[1] << " " << output_size[2];


	itk::ImageRegionConstIterator<curan::pk::VolumeReconstructor::OutputType> iterator(buffer,buffer->GetLargestPossibleRegion());

	std::ofstream volume_on_disk{"data.txt"};

	if (!volume_on_disk){
		curan::utils::console->info("Could not open the destination file");
		return 1;
	}

	curan::utils::console->info("Started writing file");
	for (iterator.GoToBegin(); !iterator.IsAtEnd();++iterator){
		volume_on_disk << static_cast<int>(iterator.Get()) << '\n';
	}
	curan::utils::console->info("Finished writing file");
	volume_on_disk.close();

	*/
	return 0;
}

/*


class TestingWindow : public curan::ui::WindowBody
{
	curan::image::Study current_volume;
	std::shared_ptr<curan::display::ImageDisplayDinamicLayout> dinamic_image_display;

public:
	TestingWindow(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param) : WindowBody(in_context,param, window_identifier)
	{
		curan::image::StudyManager* manager = curan::image::StudyManager::Get();
		curan::image::Result res = manager->GetStudy(volume_index, current_volume);
	};
	
	void init() override
	{
		std::shared_ptr<curan::display::Page> image_display_page;
		create_segmentation_page(image_display_page);
		viewer->add_page(image_display_page);
		viewer->initialize(context);
		viewer->connect_handler();
	};

	void run() override
	{
		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			viewer->draw(canvas, page_index);
			if (overlay_page_index > 0) {
				auto image = canvas->getSurface()->makeImageSnapshot();
				canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
			}
			viewer->swapBuffers();
			glfwPollEvents();
			viewer->process_pending_signals(page_index);
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		viewer->destroy();
		curan::ui::WindowManager* manager = curan::ui::WindowManager::Get();
		manager->unregister_window(identifier());
	};
	
	static std::shared_ptr<TestingWindow> make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param)
	{
		return std::make_shared<TestingWindow>(in_context,window_identifier,volume_index,param);
	};

private:
	void create_segmentation_page(std::shared_ptr<curan::display::Page>& page)
	{
		SkColor colbuton = { SK_ColorWHITE };
		SkColor coltext = { SK_ColorBLACK };

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

		SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		curan::display::OptionBox::Info info10;
		info10.movable = true;
		info10.option_box_size = { 60 * 6,60 };
		std::shared_ptr<curan::display::OptionBox> option_box = curan::display::OptionBox::make(info10);

		curan::display::ImageDisplayDinamicLayout::Info info8;
		info8.alpha_channel = kOpaque_SkAlphaType;
		info8.color_type = kGray_8_SkColorType;
		info8.vol = current_volume;
		info8.initial_interaction_mode = curan::display::ImageDisplayDinamicLayout::InteractionMode::CLICK;
		info8.layout = curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_ONE;
		info8.option_box = option_box;
		std::shared_ptr<curan::display::ImageDisplayDinamicLayout> img_disp = curan::display::ImageDisplayDinamicLayout::make(info8);

		std::shared_ptr<curan::display::Page> option_box_page;

		//since our option box directly should feed data to 
		//the image display dinamic layout then we need to 
		//provide the pointer to this instance so that the
		//correct callbacks can be made
		create_segmentation_option_box(option_box_page, img_disp);
		option_box->change_page(option_box_page);

		dinamic_image_display = img_disp;

		curan::display::Button::Info info;
		info.paintButton = paint_square;
		info.size = SkRect::MakeWH(70, 70);
		info.paintText = paint_text;
		info.icon_identifier = "save.png";
		info.button_text = "Save";
		info.textFont = text_font;
		info.click_color = SkColorSetARGB(255, 77, 195, 255);
		info.hover_color = SkColorSetARGB(255, 128, 212, 255);
		info.waiting_color = SkColorSetARGB(255, 201, 201, 201);
		std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

		info.icon_identifier = "pan.png";
		info.button_text = "Move";
		std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info);
		b3->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::pan_mode, img_disp.get());

		info.icon_identifier = "zoom.png";
		info.button_text = "Zoom";
		std::shared_ptr<curan::display::Button> b4 = curan::display::Button::make(info);
		b4->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::zoom_mode, img_disp.get());

		info.icon_identifier = "rotate.png";
		info.button_text = "Rotate";
		std::shared_ptr<curan::display::Button> b5 = curan::display::Button::make(info);
		b5->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::rotate_mode, img_disp.get());

		info.icon_identifier = "annotate.png";
		info.button_text = "Annotate";
		std::shared_ptr<curan::display::Button> b8 = curan::display::Button::make(info);
		b8->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::change_option_box_presentation_state,img_disp.get());

		info.icon_identifier = "measure.png";
		info.button_text = "Measure";
		std::shared_ptr<curan::display::Button> b9 = curan::display::Button::make(info);

		std::vector<std::shared_ptr<curan::display::Widget>> left;
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b3));
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b4));
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b5));
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b8));
		left.push_back(std::static_pointer_cast<curan::display::Widget>(b9));

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

		curan::display::LayoutLinearWidgetContainer::Info info1;
		info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
		info1.paintLayout = paint_square2;
		info1.widgets = left;
		std::shared_ptr<curan::display::LayoutLinearWidgetContainer> button_container = curan::display::LayoutLinearWidgetContainer::make(info1);

		curan::display::LayoutLinearWidgetContainer::Info info3;
		info3.arrangement = curan::display::Layout::Arrangement::VERTICAL;
		info3.paintLayout = paint_square2;
		info3.widgets = { std::static_pointer_cast<curan::display::Widget>(dinamic_image_display) };
		std::shared_ptr<curan::display::LayoutLinearWidgetContainer> managed_image_container = curan::display::LayoutLinearWidgetContainer::make(info3);

		std::vector<std::shared_ptr<curan::display::Layout>> main_layout;
		main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(button_container));
		main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(managed_image_container));

		curan::display::LayoutLinearContainer::Info info4;
		info4.arrangement = curan::display::Layout::Arrangement::HORIZONTAL;
		info4.paint_layout = paint_square;
		info4.layouts = main_layout;
		info4.divisions = { 0.0, 0.15, 1.0 };
		std::shared_ptr<curan::display::LayoutLinearContainer> widget_cont = curan::display::LayoutLinearContainer::make(info4);

		curan::display::Page::Info info5;
		info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_cont);
		info5.paint_page = SK_ColorWHITE;
		page = curan::display::Page::make(info5);

		page->monitor(std::static_pointer_cast<curan::display::Widget>(dinamic_image_display));
		for (auto p : left) {
			page->monitor(p);
		}

	//	page->monitor(std::static_pointer_cast<curan::display::Widget>(option_box));
	};

	void create_segmentation_option_box(std::shared_ptr<curan::display::Page>& image_page, std::shared_ptr<curan::display::ImageDisplayDinamicLayout> associated_display)
	{
		SkColor colbuton = { SK_ColorWHITE };
		SkColor coltext = { SK_ColorBLACK };

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

		SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

		curan::display::Button::Info info;
		info.paintButton = paint_square;
		info.size = SkRect::MakeWH(50, 50);
		info.paintText = paint_text;
		info.button_text = "";
		info.icon_identifier = "adjust_contour.png";
		info.textFont = text_font;
		info.click_color = SkColorSetARGB(255, 77, 195, 255);
		info.hover_color = SkColorSetARGB(255, 128, 212, 255);
		info.waiting_color = SkColorSetARGB(255, 201, 201, 201);
		std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);
		b1->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::change_path, associated_display.get());

		info.icon_identifier = "hide_contour.png";
		std::shared_ptr<curan::display::Button> b2 = curan::display::Button::make(info);
		b2->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::hide_path, associated_display.get());

		info.icon_identifier = "contour_vertices.png";
		std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info);
		b3->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::new_path, associated_display.get());

		info.icon_identifier = "submit.png";
		std::shared_ptr<curan::display::Button> b4 = curan::display::Button::make(info);
		b4->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::submit_path, associated_display.get());
		
		info.icon_identifier = "erase.png";
		std::shared_ptr<curan::display::Button> b5 = curan::display::Button::make(info);
		b5->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::eliminate_path, associated_display.get());
		
		std::vector<std::shared_ptr<curan::display::Widget>> horizontal_bar;
		horizontal_bar.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
		horizontal_bar.push_back(std::static_pointer_cast<curan::display::Widget>(b2));
		horizontal_bar.push_back(std::static_pointer_cast<curan::display::Widget>(b3));
		horizontal_bar.push_back(std::static_pointer_cast<curan::display::Widget>(b4));
		horizontal_bar.push_back(std::static_pointer_cast<curan::display::Widget>(b5));

		SkPaint paint_square2;
		paint_square2.setStyle(SkPaint::kFill_Style);
		paint_square2.setAntiAlias(true);
		paint_square2.setStrokeWidth(4);
		paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

		curan::display::LayoutLinearWidgetContainer::Info info1;
		info1.arrangement = curan::display::Layout::Arrangement::HORIZONTAL;
		info1.paintLayout = paint_square2;
		info1.widgets = horizontal_bar;
		std::shared_ptr<curan::display::LayoutLinearWidgetContainer> button_container = curan::display::LayoutLinearWidgetContainer::make(info1);

		curan::display::Page::Info info5;
		info5.paint_page = SK_ColorWHITE;
		info5.page_layout = { std::static_pointer_cast<curan::display::Layout>(button_container) };
		image_page = curan::display::Page::make(info5);

		for (auto wid : horizontal_bar)
			image_page->monitor(wid);
	};
};

*/


/*
	DkContext context;
	context.initialize_context();

	DkDisplayParams param;

	DkWindow viewer{ param };
	viewer.initialize(&context);

	while (!glfwWindowShouldClose(viewer.window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer.getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		canvas->clear(SK_ColorBLACK);
		viewer.swapBuffers();
		glfwPollEvents();
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(30) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}

	viewer.destroy();
	context.destroy_context();

*/

/*
	std::ifstream istrm("C:\\libraries\\assss.txt", std::ios::binary);
	if (!istrm.is_open()) {
		std::cout << "failed to open file";
	}
	else {
		uint8_t d = 0;
		uint32_t crc = 0;
		long increment = 0;
		while (istrm.peek()!= EOF){
			istrm.read((char*)&d,1);
			crc = crc32c_extend(crc,&d, 1);
			++increment;
		}
		std::cout << "The crc of the file is: " << crc << " with a size of " << increment << " bytes";
	}
*/

/*
try {	start_up(); }
catch (...){
		console->info("Exception was thrown");
		return 1;
	}
*/