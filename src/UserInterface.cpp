#include "UserInterface.h"

namespace curan {
	namespace ui {
		MainWindow::MainWindow(curan::display::Context* in_context, int window_identifier, curan::display::DisplayParams in_param) : WindowBody(in_context, in_param, window_identifier)
		{
		}

		void MainWindow::init()
		{
			std::shared_ptr<curan::display::Page> load_files_page;
			create_load_file_page(load_files_page);
			viewer->add_page(load_files_page);

			std::shared_ptr<curan::display::Page> tasks_page;
			create_tasks_page(tasks_page);
			viewer->add_page(tasks_page);

			std::shared_ptr<curan::display::Page> load_overlay_page;
			create_load_file_overlay(load_overlay_page);
			viewer->add_page(load_overlay_page);

			viewer->initialize(context);
			viewer->connect_handler();
		}

		void MainWindow::run()
		{
			while (!glfwWindowShouldClose(viewer->window)) {
				auto start = std::chrono::high_resolution_clock::now();
				SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
				SkCanvas* canvas = pointer_to_surface->getCanvas();
				viewer->draw(canvas, page_index);
				if (overlay_page_index > 0) {
					auto image = canvas->getSurface()->makeImageSnapshot();
					canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
					viewer->draw(canvas, overlay_page_index);
					viewer->process_pending_signals(overlay_page_index);
				}
				else
					viewer->process_pending_signals(page_index);

				viewer->swapBuffers();
				auto end = std::chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds(33) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
				//		std::string s = "delay: " + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());;
				//		console->info(s.c_str());
			}
			viewer->destroy();
			WindowManager* manager = WindowManager::Get();
			manager->unregister_window(window_identifier);
		}

		std::shared_ptr<MainWindow> MainWindow::make(curan::display::Context* in_context, int window_identifier, curan::display::DisplayParams in_param)
		{
			return std::make_shared<MainWindow>(in_context, window_identifier, in_param);
		}

		void MainWindow::create_load_file_page(std::shared_ptr<curan::display::Page>& first_page)
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
			info.size = SkRect::MakeWH(100, 100);
			info.paintText = paint_text;
			info.button_text = "Load Files";
			info.icon_identifier = "upload.png";
			info.textFont = text_font;
			info.click_color = SkColorSetARGB(255, 77, 195, 255);
			info.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info.waiting_color = SkColorSetARGB(0, 201, 201, 201);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

			auto lambda_open_load_file_overlay = [this](curan::display::MousePressSignal signal) {
				overlay_page_index = 2;
			};

			b1->signal_click.connect(lambda_open_load_file_overlay);

			info.button_text = "Unload Files";
			info.icon_identifier = "trash.png";
			std::shared_ptr<curan::display::Button> b2 = curan::display::Button::make(info);

			info.button_text = "Procedures";
			info.icon_identifier = "tasks.png";
			std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info);

			auto lambda_change_to_task_page = [this](curan::display::MousePressSignal signal) {
				page_index = 1;
			};

			b3->signal_click.connect(lambda_change_to_task_page);

			std::vector<std::shared_ptr<curan::display::Widget>> top;
			top.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
			top.push_back(std::static_pointer_cast<curan::display::Widget>(b2));
			top.push_back(std::static_pointer_cast<curan::display::Widget>(b3));

			SkPaint paint_square2;
			paint_square2.setStyle(SkPaint::kFill_Style);
			paint_square2.setAntiAlias(true);
			paint_square2.setStrokeWidth(4);
			paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

			curan::display::LayoutLinearWidgetContainer::Info info1;
			info1.arrangement = curan::display::Layout::Arrangement::HORIZONTAL;
			info1.paintLayout = paint_square2;
			info1.widgets = top;
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> widget_container_top = curan::display::LayoutLinearWidgetContainer::make(info1);

			SkPaint paint_square3;
			paint_square3.setStyle(SkPaint::kFill_Style);
			paint_square3.setAntiAlias(true);
			paint_square3.setStrokeWidth(4);
			paint_square3.setColor(SK_ColorBLACK);

			curan::display::ItemPreview::Info info2;
			info2.color_background_right = SK_ColorBLACK;
			info2.color_hover = SK_ColorDKGRAY;
			info2.color_selected = SkColorSetARGB(255, 77, 195, 255);
			info2.color_waiting = SkColorSetARGB(255, 217, 217, 217);
			info2.color_background_left = SkColorSetARGB(255, 135, 138, 140);
			info2.font = text_font;
			info2.text_paint = paint_text;
			paint_square2.setColor(SK_ColorWHITE);
			info1.paintLayout = paint_square2;
			managed_images = curan::display::ItemPreview::make(info2);

			curan::display::ThreadPollStatusDisplay::Info infoDkThreadPoll;
			infoDkThreadPoll.color_background = SkColorSetARGB(255, 51, 153, 255);
			infoDkThreadPoll.color_text = SK_ColorWHITE;
			infoDkThreadPoll.font = text_font;
			std::shared_ptr<curan::display::ThreadPollStatusDisplay> background_tasks = curan::display::ThreadPollStatusDisplay::make(infoDkThreadPoll);

			std::vector<std::shared_ptr<curan::display::Widget>> bottom;
			bottom.push_back(std::static_pointer_cast<curan::display::Widget>(managed_images));
			bottom.push_back(std::static_pointer_cast<curan::display::Widget>(background_tasks));

			info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info1.widgets = bottom;
			info1.divisions = { 0.0, 0.98, 1.0 };
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> layout_container_bottom = curan::display::LayoutLinearWidgetContainer::make(info1);

			std::vector<std::shared_ptr<curan::display::Layout>> main_layout;
			main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(widget_container_top));
			main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(layout_container_bottom));

			curan::display::LayoutLinearContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info4.paint_layout = paint_square;
			info4.layouts = main_layout;
			info4.divisions = { 0.0, 0.15, 1.0 };
			std::shared_ptr<curan::display::LayoutLinearContainer> widget_cont = curan::display::LayoutLinearContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_cont);
			info5.paint_page = SK_ColorWHITE;
			first_page = curan::display::Page::make(info5);

			first_page->monitor(std::static_pointer_cast<curan::display::Widget>(managed_images));
			for (auto p : top) {
				first_page->monitor(p);
			}
		}

		void MainWindow::create_tasks_page(std::shared_ptr<curan::display::Page>& second_page)
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
			info.size = SkRect::MakeWH(100, 100);
			info.paintText = paint_text;
			info.button_text = "Image previewer";
			info.icon_identifier = "imagemanager.png";
			info.textFont = text_font;
			info.click_color = SkColorSetARGB(255, 77, 195, 255);
			info.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info.waiting_color = SkColorSetARGB(255, 201, 201, 201);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

			auto lambda_change_to_volume_previewer_page = [this](curan::display::MousePressSignal signal) {
				page_index = 0;
			};

			b1->signal_click.connect(lambda_change_to_volume_previewer_page);

			std::vector<std::shared_ptr<curan::display::Widget>> top;
			top.push_back(std::static_pointer_cast<curan::display::Widget>(b1));

			SkPaint paint_square2;
			paint_square2.setStyle(SkPaint::kFill_Style);
			paint_square2.setAntiAlias(true);
			paint_square2.setStrokeWidth(4);
			paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

			curan::display::LayoutLinearWidgetContainer::Info info1;
			info1.arrangement = curan::display::Layout::Arrangement::HORIZONTAL;
			info1.paintLayout = paint_square2;
			info1.widgets = top;
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> widget_container_top = curan::display::LayoutLinearWidgetContainer::make(info1);

			SkPaint paint_square3;
			paint_square3.setStyle(SkPaint::kFill_Style);
			paint_square3.setAntiAlias(true);
			paint_square3.setStrokeWidth(4);
			paint_square3.setColor(SK_ColorBLACK);

			curan::display::Task task1;
			task1.name = "2D Viewer";
			task1.description = "A viewer which allows one manipulate images with segmentation and registration algorithms";
			task1.icon_identifier = "medicalviewer.png";
			task1.task_type = curan::display::TasksEnumeration::MEDICAL_VIEWER;

			curan::display::Task task2;
			task2.name = "Biopsy Procedure";
			task2.description = "A routine which implements in stages of the medical procedure using a robotic system";
			task2.icon_identifier = "biopsyviewer.png";
			task2.task_type = curan::display::TasksEnumeration::BIOPSY_VIEWER;

			curan::display::TaskPreviewer::Info info6;
			info6.click_color = SkColorSetARGB(255, 77, 195, 255);
			info6.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info6.waiting_color = SkColorSetARGB(255, 201, 201, 201);
			info6.font = text_font;
			info6.relative_dimensions = SkRect::MakeLTRB(0.2, 0.2, 0.8, 0.8);
			info6.supported_tasks = { task1,task2 };
			info6.text_color = coltext;
			std::shared_ptr<curan::display::TaskPreviewer> task_preview = curan::display::TaskPreviewer::make(info6);

			curan::display::ThreadPollStatusDisplay::Info infoDkThreadPoll;
			infoDkThreadPoll.color_background = SkColorSetARGB(255, 51, 153, 255);
			infoDkThreadPoll.color_text = SK_ColorWHITE;
			infoDkThreadPoll.font = text_font;
			std::shared_ptr<curan::display::ThreadPollStatusDisplay> background_tasks = curan::display::ThreadPollStatusDisplay::make(infoDkThreadPoll);

			curan::display::Button::Info info11;
			info11.paintButton = paint_square;
			info11.size = SkRect::MakeWH(100, 100);
			info11.paintText = paint_text;
			info11.button_text = "Lauch Window";
			info11.textFont = text_font;
			info11.click_color = SkColorSetARGB(255, 77, 195, 255);
			info11.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info11.waiting_color = SkColorSetARGB(255, 201, 201, 201);
			std::shared_ptr<curan::display::Button> b11 = curan::display::Button::make(info11);

			b11->signal_click.connect(lambda_change_to_volume_previewer_page);


			std::vector<std::shared_ptr<curan::display::Widget>> bottom;
			bottom.push_back(std::static_pointer_cast<curan::display::Widget>(task_preview));
			bottom.push_back(std::static_pointer_cast<curan::display::Widget>(b11));
			bottom.push_back(std::static_pointer_cast<curan::display::Widget>(background_tasks));

			info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info1.widgets = bottom;
			info1.divisions = { 0.0, 0.7, 0.98, 1.0 };
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> layout_container_bottom = curan::display::LayoutLinearWidgetContainer::make(info1);

			std::vector<std::shared_ptr<curan::display::Layout>> main_layout;
			main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(widget_container_top));
			main_layout.push_back(std::static_pointer_cast<curan::display::Layout>(layout_container_bottom));

			curan::display::LayoutLinearContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info4.paint_layout = paint_square;
			info4.layouts = main_layout;
			info4.divisions = { 0.0, 0.15, 1.0 };
			std::shared_ptr<curan::display::LayoutLinearContainer> widget_cont = curan::display::LayoutLinearContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_cont);
			info5.paint_page = SK_ColorWHITE;
			second_page = curan::display::Page::make(info5);

			second_page->monitor(std::static_pointer_cast<curan::display::Widget>(task_preview));
			for (auto p : top) {
				second_page->monitor(p);
			}
		}


		void MainWindow::create_load_file_overlay(std::shared_ptr<curan::display::Page>& page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorWHITE };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SkColorSetARGB(0, 0, 0, 0));

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
			info.size = SkRect::MakeWH(300, 200);
			info.paintText = paint_text;
			info.button_text = "Drag dicom file or directory to read";
			info.icon_identifier = "drag_drop.png";
			info.textFont = text_font;
			info.click_color = SkColorSetARGB(150, 40, 40, 40);
			info.hover_color = SkColorSetARGB(150, 40, 40, 40);
			info.waiting_color = SkColorSetARGB(150, 40, 40, 40);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

			auto lambda_close_overlays = [this](curan::display::MousePressSignal signal) {
				overlay_page_index = 0;
			};

			auto lambda_load_files = [this](curan::display::ItemDroppedSignal signal) {
				curan::image::StudyManager* manager = curan::image::StudyManager::Get();

				std::vector<std::filesystem::path> paths;
				paths.resize(signal.count);
				for (int index = 0; index < signal.count; ++index) {
					std::string small_path = std::string{ signal.paths[index] };
					paths[index] = std::filesystem::path{ small_path };
				}

				curan::image::Result res = manager->LoadStudies(paths);

				for (int index = 0; index < signal.count; ++index) {
					char* array_of_data = signal.paths[index];
					delete[] array_of_data;
				}
				char** paths_received = signal.paths;
				delete[] paths_received;

				overlay_page_index = 0;
			};

			b1->signal_click.connect(lambda_close_overlays);
			b1->signal_dropped.connect(lambda_load_files);

			curan::display::LayoutLinearWidgetContainer::Info info1;
			info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info1.paintLayout = paint_square;
			info1.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> widget_container_top = curan::display::LayoutLinearWidgetContainer::make(info1);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_container_top);
			info5.paint_page = SK_ColorWHITE;
			page = curan::display::Page::make(info5);

			page->monitor(std::static_pointer_cast<curan::display::Widget>(b1));
		}

		void MainWindow::create_error_feedback_page(std::shared_ptr<curan::display::Page>& fourth_page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorWHITE };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SkColorSetARGB(0, 0, 0, 0));

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
			info.size = SkRect::MakeWH(300, 200);
			info.paintText = paint_text;
			info.button_text = "Error";
			info.icon_identifier = "warning.png";
			info.textFont = text_font;
			info.click_color = SkColorSetARGB(150, 40, 40, 40);
			info.hover_color = SkColorSetARGB(150, 40, 40, 40);
			info.waiting_color = SkColorSetARGB(150, 40, 40, 40);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

			auto lambda_close_overlays = [this](curan::display::MousePressSignal signal) {
				page_index = 0;
			};

			b1->signal_click.connect(lambda_close_overlays);

			curan::display::LayoutLinearWidgetContainer::Info info1;
			info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
			info1.paintLayout = paint_square;
			info1.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
			std::shared_ptr<curan::display::LayoutLinearWidgetContainer> widget_container_top = curan::display::LayoutLinearWidgetContainer::make(info1);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_container_top);
			info5.paint_page = SK_ColorWHITE;
			fourth_page = curan::display::Page::make(info5);

			fourth_page->monitor(std::static_pointer_cast<curan::display::Widget>(b1));
		}

		BiopsyViewer::BiopsyViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams in_param) : WindowBody(in_context, in_param, window_identifier)
		{

			curan::image::StudyManager* manager = curan::image::StudyManager::Get();
			curan::image::Result res = manager->GetStudy(volume_index, current_volume);
		};

		void BiopsyViewer::init()
		{
			std::shared_ptr<curan::display::Page> page0;
			create_image_display_page(page0);
			viewer->add_page(page0);

			std::shared_ptr<curan::display::Page> page1;
			create_validation_robot_motion(page1);
			viewer->add_page(page1);

			std::shared_ptr<curan::display::Page> page2;
			create_communication_validation_page(page2);
			viewer->add_page(page2);

			std::shared_ptr<curan::display::Page> page3;
			create_calibration_page(page3);
			viewer->add_page(page3);

			viewer->initialize(context);
			viewer->connect_handler();
		}


		void BiopsyViewer::run()
		{
			while (!glfwWindowShouldClose(viewer->window)) {
				auto start = std::chrono::high_resolution_clock::now();
				SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
				SkCanvas* canvas = pointer_to_surface->getCanvas();
				viewer->draw(canvas, page_index);
				if (overlay_page_index > 0) {
					auto image = canvas->getSurface()->makeImageSnapshot();
					canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
					viewer->draw(canvas, overlay_page_index);
					viewer->process_pending_signals(overlay_page_index);
				}
				else
					viewer->process_pending_signals(page_index);
				viewer->swapBuffers();
				auto end = std::chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
			}
			viewer->destroy();
			WindowManager* manager = WindowManager::Get();
			manager->unregister_window(identifier());
		}
		std::shared_ptr<BiopsyViewer> BiopsyViewer::make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams in_param)
		{
			return std::make_shared<BiopsyViewer>(in_context, window_identifier, volume_index, in_param);
		}

		void BiopsyViewer::create_image_display_page(std::shared_ptr<curan::display::Page>& image_page)
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

			curan::display::ThreeWayButton::Info info7;
			info7.size = SkRect::MakeWH(600, 70);
			info7.stages_array = { "planning stage","robot validation","system validation","calibration" };
			info7.click_color_side = SkColorSetARGB(255, 77, 195, 255);
			info7.waiting_color_side = SkColorSetARGB(255, 204, 247, 255);
			info7.hover_color_side = SkColorSetARGB(255, 128, 212, 255);
			info7.font = text_font;
			info7.color_center = SkColorSetARGB(255, 255, 230, 204);
			stage_button = curan::display::ThreeWayButton::make(info7);

			auto lambda_change_pages = [this](curan::display::MousePressSignal signal, curan::display::ThreeWayButton::ButtonPressed button_pressed) {
				switch (button_pressed)
				{
				case curan::display::ThreeWayButton::ButtonPressed::LEFT:
				{
					uint32_t page_index = get_page_index();
					change_page_index(page_index - 1);
				}
				break;
				case curan::display::ThreeWayButton::ButtonPressed::RIGHT:
				{
					uint32_t page_index = get_page_index();
					change_page_index(page_index + 1);
				}
				break;
				default:
					break;
				}
			};

			stage_button->signal_click.connect(lambda_change_pages);

			curan::display::ImageDisplayDinamicLayout::Info info2;
			info2.alpha_channel = kOpaque_SkAlphaType;
			info2.color_type = kGray_8_SkColorType;
			info2.vol = current_volume;
			info2.initial_interaction_mode = curan::display::ImageDisplayDinamicLayout::InteractionMode::CLICK;
			info2.layout = curan::display::ImageDisplayDinamicLayout::LayoutState::TWO_BY_THREE;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> img_disp = curan::display::ImageDisplayDinamicLayout::make(info2);

			dinamic_image_display = img_disp;

			curan::display::Button::Info info3;
			info3.paintButton = paint_square;
			info3.size = SkRect::MakeWH(100, 100);
			info3.paintText = paint_text;
			info3.button_text = "Click";
			info3.icon_identifier = "click.png";
			info3.textFont = text_font;
			info3.click_color = SkColorSetARGB(255, 77, 195, 255);
			info3.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info3.waiting_color = SkColorSetARGB(255, 255, 255, 255);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info3);

			info3.button_text = "Zoom";
			info3.icon_identifier = "zoom.png";
			std::shared_ptr<curan::display::Button> b2 = curan::display::Button::make(info3);

			info3.button_text = "Rotate";
			info3.icon_identifier = "rotate.png";
			std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info3);

			info3.button_text = "Pan";
			info3.icon_identifier = "pan.png";
			std::shared_ptr<curan::display::Button> b4 = curan::display::Button::make(info3);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.2, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b1));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.4, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b2));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.6, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b3));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.8, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b4));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.2, 0.2, 0.8, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(img_disp));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b1));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b2));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b3));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b4));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(img_disp));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
		}

		void BiopsyViewer::create_validation_robot_motion(std::shared_ptr<curan::display::Page>& image_page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorBLACK };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(colbuton);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			curan::display::ManipulatorPosition::Info info3;
			info3.color_background = SK_ColorGRAY;
			info3.color_ranges = SK_ColorMAGENTA;
			info3.color_text = SK_ColorBLACK;
			info3.font = text_font;
			info3.joint_names = { "First Joint", "Second Joint", "Third Joint","Fourth Joint", "Fifth Joint" };
			info3.ranges = { {-120,120},{-90,90},{-10,10},{-100,100},{-100,100} };

			std::shared_ptr<curan::display::ManipulatorPosition> special_button = curan::display::ManipulatorPosition::make(info3);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(1.0 / 3, 0.2, 1.0 / 3, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(special_button));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(special_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));

		}

		void BiopsyViewer::create_communication_validation_page(std::shared_ptr<curan::display::Page>& image_page)
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


			curan::display::ImageDisplayDinamicLayout::Info info2;
			info2.alpha_channel = kOpaque_SkAlphaType;
			info2.color_type = kGray_8_SkColorType;
			info2.vol = current_volume;
			info2.initial_interaction_mode = curan::display::ImageDisplayDinamicLayout::InteractionMode::CLICK;
			info2.layout = curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_THREE;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> img_disp = curan::display::ImageDisplayDinamicLayout::make(info2);

			curan::display::IconResources* resources = curan::display::IconResources::Get();

			sk_sp<SkImage> image1;
			resources->GetIcon(image1, "monitor_validation.png");

			sk_sp<SkImage> image2;
			resources->GetIcon(image2, "robot_validation.png");

			sk_sp<SkImage> image3;
			resources->GetIcon(image3, "ultrasound_validation.png");

			curan::display::SystemValidation::Info info3;
			info3.icons_of_devices = { image1,image2,image3 };
			info3.path_validation_colors = { SK_ColorRED, SK_ColorGREEN };
			std::shared_ptr<curan::display::SystemValidation> special_button = curan::display::SystemValidation::make(info3);

			curan::display::SystemValidation* pointer_to_status_connection = special_button.get();

			auto lambda = [pointer_to_status_connection]() {
				asio::io_context* context;
				
				curan::communication::GetIOContext(&context);
				curan::communication::ChannelManager* channel_manager = curan::communication::ChannelManager::Get();
				asio::ip::tcp::resolver resolver(*context);
				asio::error_code error;
				asio::ip::tcp::resolver::results_type results = resolver.resolve("localhost", "18944", error);
				channel_manager->start_channel("plusserver", *context);
				std::shared_ptr<curan::communication::ProcessorOpenIGTLink> channel;
				auto error1 = channel_manager->get_channel("plusserver", channel);
				channel->signal_connection_status.connect(&curan::display::SystemValidation::update_connection_status, pointer_to_status_connection);
				channel->connect(results);
			};

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.2, 1, 0.6));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(special_button));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			info5.initialize_page_callback = lambda;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(special_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
		}

		void BiopsyViewer::create_calibration_page(std::shared_ptr<curan::display::Page>& image_page)
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

			curan::display::CommunicationDisplay::Info info;
			info.text_font = text_font;
			std::shared_ptr<curan::display::CommunicationDisplay> receiving_images = curan::display::CommunicationDisplay::make(info);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.2, 1, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(receiving_images));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::CommunicationDisplay* pointer_to_communication_display = receiving_images.get();

			auto lambda = [pointer_to_communication_display]() {
				curan::communication::ChannelManager* channel_manager = curan::communication::ChannelManager::Get();
				std::shared_ptr<curan::communication::ProcessorOpenIGTLink> channel;
				auto error1 = channel_manager->get_channel("plusserver", channel);
				if (curan::communication::ChannelManager::Error::SUCCESS == error1)
				{
					channel->signal_received.connect(&curan::display::CommunicationDisplay::process_message, pointer_to_communication_display);
					pointer_to_communication_display->connection_status(channel->getStatus());
					channel->signal_connection_status.connect(&curan::display::CommunicationDisplay::connection_status, pointer_to_communication_display);
				}
			};

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			info5.initialize_page_callback = lambda;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(receiving_images));
		}

		void BiopsyViewer::create_registration_page(std::shared_ptr<curan::display::Page>& image_page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorBLACK };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(colbuton);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			curan::display::ManipulatorPosition::Info info3;
			info3.color_background = SK_ColorGRAY;
			info3.color_ranges = SK_ColorMAGENTA;
			info3.color_text = SK_ColorBLACK;
			info3.font = text_font;
			info3.joint_names = { "First Joint", "Second Joint", "Third Joint","Fourth Joint", "Fifth Joint" };
			info3.ranges = { {-120,120},{-90,90},{-10,10},{-100,100},{-100,100} };

			std::shared_ptr<curan::display::ManipulatorPosition> special_button = curan::display::ManipulatorPosition::make(info3);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.0, 0.2, 1, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(special_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
		}

		void BiopsyViewer::create_registration_validation_page(std::shared_ptr<curan::display::Page>& image_page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorBLACK };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(colbuton);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			curan::display::ManipulatorPosition::Info info3;
			info3.color_background = SK_ColorGRAY;
			info3.color_ranges = SK_ColorMAGENTA;
			info3.color_text = SK_ColorBLACK;
			info3.font = text_font;
			info3.joint_names = { "First Joint", "Second Joint", "Third Joint","Fourth Joint", "Fifth Joint" };
			info3.ranges = { {-120,120},{-90,90},{-10,10},{-100,100},{-100,100} };

			std::shared_ptr<curan::display::ManipulatorPosition> special_button = curan::display::ManipulatorPosition::make(info3);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.3, 0.2, 0.7, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(special_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
		}

		void BiopsyViewer::create_intraoperative_page(std::shared_ptr<curan::display::Page>& image_page) {
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorBLACK };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(colbuton);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			curan::display::ManipulatorPosition::Info info3;
			info3.color_background = SK_ColorGRAY;
			info3.color_ranges = SK_ColorMAGENTA;
			info3.color_text = SK_ColorBLACK;
			info3.font = text_font;
			info3.joint_names = { "First Joint", "Second Joint", "Third Joint","Fourth Joint", "Fifth Joint" };
			info3.ranges = { {-120,120},{-90,90},{-10,10},{-100,100},{-100,100} };

			std::shared_ptr<curan::display::ManipulatorPosition> special_button = curan::display::ManipulatorPosition::make(info3);

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.3, 0.2, 0.7, 0.8));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(stage_button));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(special_button));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(stage_button));
		}


		MedicalViewer::MedicalViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams in_param) : WindowBody(in_context, in_param, window_identifier)
		{
			
			curan::image::StudyManager* manager = curan::image::StudyManager::Get();
			curan::image::Result res = manager->GetStudy(volume_index, current_volume);
		}

		void MedicalViewer::init()
		{
			std::shared_ptr<curan::display::Page> image_display_page;
			create_image_display_page(image_display_page);
			viewer->add_page(image_display_page);

			std::shared_ptr<curan::display::Page> overlay_image_icon;
			create_overlay_image_display(overlay_image_icon);
			viewer->add_page(overlay_image_icon);

			viewer->initialize(context);
			viewer->connect_handler();
		}

		void MedicalViewer::run()
		{
			while (!glfwWindowShouldClose(viewer->window)) {
				auto start = std::chrono::high_resolution_clock::now();
				SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
				SkCanvas* canvas = pointer_to_surface->getCanvas();
				viewer->draw(canvas, page_index);
				if (overlay_page_index > 0) {
					auto image = canvas->getSurface()->makeImageSnapshot();
					canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
					viewer->draw(canvas, overlay_page_index);
					viewer->process_pending_signals(overlay_page_index);
				}
				else
				{
					viewer->process_pending_signals(page_index);
				}
				viewer->swapBuffers();
				auto end = std::chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
				curan::utils::console->info(std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()));
			}
			viewer->destroy();
			WindowManager* manager = WindowManager::Get();
			manager->unregister_window(identifier());
		}

		std::shared_ptr<MedicalViewer> MedicalViewer::make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams in_param)
		{
			return std::make_shared<MedicalViewer>(in_context, window_identifier, volume_index, in_param);
		}

		void MedicalViewer::change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState new_layout)
		{
			dinamic_image_display->change_layout(new_layout);
			add_overlay(0);
		}

		void MedicalViewer::create_image_display_page(std::shared_ptr<curan::display::Page>& image_page)
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

			curan::display::ImageDisplayDinamicLayout::Info info2;
			info2.alpha_channel = kOpaque_SkAlphaType;
			info2.color_type = kGray_8_SkColorType;
			info2.vol = current_volume;
			info2.initial_interaction_mode = curan::display::ImageDisplayDinamicLayout::InteractionMode::PAN;
			info2.layout = curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_ONE;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> img_disp = curan::display::ImageDisplayDinamicLayout::make(info2);

			dinamic_image_display = img_disp;

			curan::display::Button::Info info3;
			info3.paintButton = paint_square;
			info3.size = SkRect::MakeWH(100, 100);
			info3.paintText = paint_text;
			info3.button_text = "Click";
			info3.icon_identifier = "click.png";
			info3.textFont = text_font;
			info3.click_color = SkColorSetARGB(255, 77, 195, 255);
			info3.hover_color = SkColorSetARGB(255, 128, 212, 255);
			info3.waiting_color = SkColorSetARGB(255, 255, 255, 255);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info3);
			b1->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::click_mode, img_disp.get());

			info3.button_text = "Zoom";
			info3.icon_identifier = "zoom.png";
			std::shared_ptr<curan::display::Button> b2 = curan::display::Button::make(info3);
			b2->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::zoom_mode, img_disp.get());

			info3.button_text = "Rotate";
			info3.icon_identifier = "rotate.png";
			std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info3);
			b3->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::rotate_mode, img_disp.get());

			info3.button_text = "Pan";
			info3.icon_identifier = "pan.png";
			std::shared_ptr<curan::display::Button> b4 = curan::display::Button::make(info3);
			b4->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::pan_mode, img_disp.get());

			info3.button_text = "change layout";
			info3.icon_identifier = "change_layout.png";
			std::shared_ptr<curan::display::Button> b5 = curan::display::Button::make(info3);

			auto lambda_change_page_to_layout_overlay = [this](curan::display::MousePressSignal press) {
				overlay_page_index = 1;
			};

			b5->signal_click.connect(lambda_change_page_to_layout_overlay);

			info3.button_text = "create contour";
			info3.icon_identifier = "contour.png";
			std::shared_ptr<curan::display::Button> b6 = curan::display::Button::make(info3);
			//b6->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::change_path_state, img_disp.get());

			curan::display::LayoutVariableWidgetContainer::Info info4;
			info4.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info4.paintLayout = paint_square;

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b1));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 2 / 15.0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b2));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 2 * 2 / 15.0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b3));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 3 * 2 / 15.0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b4));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 4 * 2 / 15.0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b5));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 5 * 2 / 15.0, 0.2, 0.2));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b6));

			info4.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.2, 0, 0.8, 1));
			info4.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(img_disp));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> container = curan::display::LayoutVariableWidgetContainer::make(info4);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(container);
			info5.paint_page = SK_ColorWHITE;
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b1));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b2));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b3));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b4));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b5));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b6));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(img_disp));
		}

		void MedicalViewer::create_overlay_image_display(std::shared_ptr<curan::display::Page>& image_page)
		{
			SkColor colbuton = { SK_ColorWHITE };
			SkColor coltext = { SK_ColorWHITE };

			SkPaint paint_square;
			paint_square.setStyle(SkPaint::kFill_Style);
			paint_square.setAntiAlias(true);
			paint_square.setStrokeWidth(4);
			paint_square.setColor(SkColorSetARGB(0, 0, 0, 0));

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
			info.size = SkRect::MakeWH(300, 200);
			info.paintText = paint_text;
			info.button_text = "one by one";
			info.icon_identifier = "layout1x1.png";
			info.textFont = text_font;
			info.click_color = SkColorSetARGB(125, 77, 195, 255);
			info.hover_color = SkColorSetARGB(125, 128, 212, 255);
			info.waiting_color = SkColorSetARGB(150, 40, 40, 40);
			std::shared_ptr<curan::display::Button> b1 = curan::display::Button::make(info);

			auto lambda_change_layout_1x1 = [this](curan::display::MousePressSignal press) {
				dinamic_image_display->change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_ONE);
				add_overlay(0);
			};

			b1->signal_click.connect(lambda_change_layout_1x1);


			info.button_text = "one by two";
			info.icon_identifier = "layout1x2.png";
			std::shared_ptr<curan::display::Button> b2 = curan::display::Button::make(info);

			auto lambda_change_layout_1x2 = [this](curan::display::MousePressSignal press) {
				dinamic_image_display->change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_TWO);
				add_overlay(0);
			};

			b2->signal_click.connect(lambda_change_layout_1x2);

			info.button_text = "one by three";
			info.icon_identifier = "layout1x3.png";
			std::shared_ptr<curan::display::Button> b3 = curan::display::Button::make(info);

			auto lambda_change_layout_1x3 = [this](curan::display::MousePressSignal press) {
				dinamic_image_display->change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState::ONE_BY_THREE);
				add_overlay(0);
			};

			b3->signal_click.connect(lambda_change_layout_1x3);

			info.button_text = "two by three";
			info.icon_identifier = "layout2x3.png";
			std::shared_ptr<curan::display::Button> b4 = curan::display::Button::make(info);

			auto lambda_change_layout_2x3 = [this](curan::display::MousePressSignal press) {
				dinamic_image_display->change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState::TWO_BY_THREE);
				add_overlay(0);
			};

			b4->signal_click.connect(lambda_change_layout_2x3);

			curan::display::LayoutVariableWidgetContainer::Info info2;
			info2.arrangement = curan::display::Layout::Arrangement::VARIABLE;
			info2.paintLayout = paint_square;

			info2.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b1));
			info2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 0.5, 0.5));

			info2.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b2));
			info2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.5, 0, 0.5, 0.5));

			info2.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b3));
			info2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.5, 0.5, 0.5));

			info2.widgets.push_back(std::static_pointer_cast<curan::display::Widget>(b4));
			info2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.5, 0.5, 0.5, 0.5));

			std::shared_ptr<curan::display::LayoutVariableWidgetContainer> widget_container_top = curan::display::LayoutVariableWidgetContainer::make(info2);

			curan::display::Page::Info info5;
			info5.page_layout = std::static_pointer_cast<curan::display::Layout>(widget_container_top);
			image_page = curan::display::Page::make(info5);

			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b1));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b2));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b3));
			image_page->monitor(std::static_pointer_cast<curan::display::Widget>(b4));
		}

		WindowManager::WindowManager()
		{
		}

		void WindowManager::register_window(std::shared_ptr<WindowBody> new_window)
		{
			std::lock_guard<std::mutex> lk(mut);
			auto it = contained_windows.find(new_window->identifier());
			if (it != contained_windows.end())
			{
				//window is already present...
				return;
			}
			else
			{
				contained_windows.emplace(new_window->identifier(), new_window);
				++number_of_active_windows;
				++window_identifier;
			}

		}

		void WindowManager::unregister_window(int identifier_old_window)
		{
			std::lock_guard<std::mutex> lk(mut);
			auto it = contained_windows.find(identifier_old_window);

			if (it == contained_windows.end()) {// if true it means we did not find a match and we should responde with a message signaling the failure
				return;
			}
			else {
				--number_of_active_windows;
				if (number_of_active_windows <= 0)
					window_manager_active = false;
			}


		}

		void WindowManager::process_os_events()
		{
			while (window_manager_active) {
				glfwPollEvents();
				std::this_thread::sleep_for(std::chrono::milliseconds(16));
			}
		}

		void WindowManager::lauch_main_window()
		{
			int ident = next_window_identifier();
			curan::display::DisplayParams param;
			std::shared_ptr<MainWindow> main_window = MainWindow::make(context, ident, param);
			main_window->init();
			register_window(std::static_pointer_cast<WindowBody>(main_window));
			std::thread{ &MainWindow::run,main_window.get() }.detach();
		}

		void WindowManager::lauch_medical_visualizer(int volume_identifier)
		{
			int ident = next_window_identifier();
			curan::display::DisplayParams param;
			std::shared_ptr<MedicalViewer> main_window = MedicalViewer::make(context, ident, volume_identifier, param);
			main_window->init();
			register_window(std::static_pointer_cast<WindowBody>(main_window));
			std::thread{ &MedicalViewer::run,main_window.get() }.detach();
		}

		void WindowManager::lauch_biopsy_visualizer(int volume_identifier)
		{
			int ident = next_window_identifier();
			curan::display::DisplayParams param;
			std::shared_ptr<BiopsyViewer> main_window = BiopsyViewer::make(context, ident, volume_identifier, param);
			main_window->init();
			register_window(std::static_pointer_cast<WindowBody>(main_window));
			std::thread{ &BiopsyViewer::run,main_window.get() }.detach();
		}

		void WindowManager::lauch_faiviewer_visualizer(int volume_identifier)
		{
			int ident = next_window_identifier();
			curan::display::DisplayParams param;
			std::shared_ptr<FAIViewer> main_window = FAIViewer::make(context, ident, volume_identifier, param);
			main_window->init();
			register_window(std::static_pointer_cast<WindowBody>(main_window));
			std::thread{ &FAIViewer::run,main_window.get() }.detach();
		}

		int WindowManager::next_window_identifier()
		{
			std::lock_guard<std::mutex> lk(mut);
			return window_identifier;
		}

		void WindowManager::set_context(curan::display::Context* in_context)
		{
			context = in_context;
		}

		WindowManager* WindowManager::Get()
		{
			static WindowManager manager{};
			return &manager;
		}

		WindowBody::WindowBody(curan::display::Context* in_context, curan::display::DisplayParams in_param, int in_window_identifier)
		{
			overlay_paint.setImageFilter(SkImageFilters::Blur(15.0, 15.0, nullptr));
			context = in_context;
			window_identifier = in_window_identifier;
			param = in_param;
			viewer = new curan::display::Window{ param };
		}

		void WindowBody::init()
		{
		}

		void WindowBody::run()
		{
		}

		void WindowBody::change_page_index(uint32_t in_page_index)
		{
			page_index = in_page_index;
		}

		void WindowBody::add_overlay(uint32_t in_overlay_page_index)
		{
			overlay_page_index = in_overlay_page_index;
		}

		uint32_t WindowBody::get_page_index()
		{
			return page_index;
		}

		int WindowBody::identifier()
		{
			return window_identifier;
		}

		FAIViewer::FAIViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams in_param) : WindowBody(in_context, in_param, window_identifier)
		{
			curan::image::StudyManager* manager = curan::image::StudyManager::Get();
			curan::image::Result res = manager->GetStudy(volume_index, current_volume);
		}

		void FAIViewer::init()
		{
			std::shared_ptr<curan::display::Page> image_display_page;
			create_segmentation_page(image_display_page);
			viewer->add_page(image_display_page);
			viewer->initialize(context);
			viewer->connect_handler();
		}

		void FAIViewer::run()
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
				viewer->process_pending_signals(page_index);
				auto end = std::chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
			}
			viewer->destroy();
			WindowManager* manager = WindowManager::Get();
			manager->unregister_window(identifier());
		}

		std::shared_ptr<FAIViewer> FAIViewer::make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param)
		{
			return std::make_shared<FAIViewer>(in_context, window_identifier, volume_index, param);
		}

		void FAIViewer::create_segmentation_page(std::shared_ptr<curan::display::Page>& page)
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
			b8->signal_click.connect(&curan::display::ImageDisplayDinamicLayout::change_option_box_presentation_state, img_disp.get());

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
		}

		void FAIViewer::statistical_femur_comparison(std::shared_ptr<curan::display::Page>& page)
		{
		}

		void FAIViewer::create_overlay_image_display(std::shared_ptr<curan::display::Page>& image_page)
		{
		}

		void FAIViewer::create_segmentation_option_box(std::shared_ptr<curan::display::Page>& image_page, std::shared_ptr<curan::display::ImageDisplayDinamicLayout> associated_display)
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
		}

	}
}