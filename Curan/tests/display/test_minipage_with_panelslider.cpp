#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/MiniPage.h"
#include "userinterface/widgets/Overlay.h"
#include <unordered_map>
#include <optional>
#include <functional>

#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "userinterface/widgets/ImageWrapper.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

std::optional<ImageType::Pointer> get_volume(std::string path)
{
	using ReaderType = itk::ImageSeriesReader<DICOMImageType>;
	auto reader = ReaderType::New();

	using ImageIOType = itk::GDCMImageIO;
	auto dicomIO = ImageIOType::New();

	reader->SetImageIO(dicomIO);

	using NamesGeneratorType = itk::GDCMSeriesFileNames;
	auto nameGenerator = NamesGeneratorType::New();

	nameGenerator->SetUseSeriesDetails(true);
	nameGenerator->AddSeriesRestriction("0008|0021");

	nameGenerator->SetDirectory(path);

	using SeriesIdContainer = std::vector<std::string>;

	const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();

	auto seriesItr = seriesUID.begin();
	auto seriesEnd = seriesUID.end();
	while (seriesItr != seriesEnd)
	{
		std::cout << seriesItr->c_str() << std::endl;
		++seriesItr;
	}

	std::string seriesIdentifier;
	seriesIdentifier = seriesUID.begin()->c_str();

	using FileNamesContainer = std::vector<std::string>;
	FileNamesContainer fileNames;

	fileNames = nameGenerator->GetFileNames(seriesIdentifier);

	reader->SetFileNames(fileNames);

	using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
	auto rescale = RescaleType::New();
	rescale->SetInput(reader->GetOutput());
	rescale->SetOutputMinimum(0);
	rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

	using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
	auto filter = FilterType::New();
	filter->SetInput(rescale->GetOutput());

	try
	{
		filter->Update();
	}
	catch (const itk::ExceptionObject &ex)
	{
		std::cout << ex << std::endl;
		return std::nullopt;
	}

	return filter->GetOutput();
}

void create_one_page_horizontal_slider(curan::ui::IconResources& resources, curan::ui::VolumetricMask* mask,curan::ui::MiniPage* minipage){
	using namespace curan::ui;
    std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::X);

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
	*container << std::move(image_display);

    minipage->construct(std::move(container),SK_ColorBLACK);
};

void create_two_page_horizontal_slider(curan::ui::IconResources& resources, curan::ui::VolumetricMask* mask,curan::ui::MiniPage* minipage){
	using namespace curan::ui;
    std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::X);
	std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::Y);
	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
	*container << std::move(image_display_x) << std::move(image_display_y);

    minipage->construct(std::move(container),SK_ColorBLACK);
};

void create_three_page_horizontal_slider(curan::ui::IconResources& resources, curan::ui::VolumetricMask* mask,curan::ui::MiniPage* minipage){
	using namespace curan::ui;
    std::unique_ptr<curan::ui::SlidingPanel> image_display_x = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::X);
	std::unique_ptr<curan::ui::SlidingPanel> image_display_y = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::Y);
	std::unique_ptr<curan::ui::SlidingPanel> image_display_z = curan::ui::SlidingPanel::make(resources, mask, curan::ui::Direction::Z);

	auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
	*container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);

    minipage->construct(std::move(container),SK_ColorBLACK);
};

std::unique_ptr<curan::ui::Overlay> create_layout_page(curan::ui::IconResources& resources,curan::ui::MiniPage* minipage,curan::ui::VolumetricMask* mask) {
	using namespace curan::ui;
	auto button = Button::make(" ","layout1x1.png",resources);
	button->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 200));
	button->add_press_call([&resources,minipage,mask](Button* button, Press press ,ConfigDraw* config) {
		create_one_page_horizontal_slider(resources,mask,minipage);
	});

	auto button2 = Button::make(" ","layout1x2.png",resources);
	button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 200));
	button2->add_press_call([&resources,minipage,mask](Button* button, Press press ,ConfigDraw* config) {
		create_two_page_horizontal_slider(resources,mask,minipage);
	});

	auto button3 = Button::make(" ","layout1x3.png",resources);
	button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 200));
	button3->add_press_call([&resources,minipage,mask](Button* button, Press press ,ConfigDraw* config) {
		create_three_page_horizontal_slider(resources,mask,minipage);
	});

	auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*viwers_container << std::move(button) << std::move(button2) << std::move(button3);
	viwers_container->set_color(SK_ColorTRANSPARENT);
	
	return Overlay::make(std::move(viwers_container),SkColorSetARGB(10,125,125,125),true);
}

std::unique_ptr<curan::ui::Overlay> create_option_page(curan::ui::IconResources& resources,curan::ui::MiniPage* minipage,curan::ui::VolumetricMask* mask) {
	using namespace curan::ui;
	auto button = Button::make("Layout",resources);
	button->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
	button->add_press_call([&resources,minipage,mask](Button* button, Press press ,ConfigDraw* config) {
		if(config->stack_page!=nullptr){
			config->stack_page->stack(create_layout_page(resources,minipage,mask));
		}
	});

	auto button2 = Button::make("Resample AC-PC",resources);
	button2->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
	button2->add_press_call([&](Button* button, Press press ,ConfigDraw* config) {
		
	});

	auto button3 = Button::make("Define Trajectory",resources);
	button3->set_click_color(SK_ColorLTGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(200, 100));
	button3->add_press_call([&](Button* button, Press press ,ConfigDraw* config) {
		
	});

	auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*viwers_container << std::move(button) << std::move(button2) << std::move(button3);
	viwers_container->set_color(SK_ColorTRANSPARENT);
	
	return Overlay::make(std::move(viwers_container),SkColorSetARGB(10,125,125,125),true);
}

int main()
{
	try
	{
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();
		
		DisplayParams param{std::move(context), 2200, 1200};
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/mri_brain");
		if (!volume)
			return 1;

		VolumetricMask mask{*volume};
		VolumetricMask* mask_pointer = &mask;

		std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources,&mask, curan::ui::Direction::X);

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*container << std::move(image_display);

        std::unique_ptr<curan::ui::MiniPage> minipage = curan::ui::MiniPage::make(std::move(container), SK_ColorBLACK);
		curan::ui::MiniPage* minipage_pointer = minipage.get();
		minipage->add_key_call([&resources,minipage_pointer,mask_pointer](curan::ui::MiniPage* minipage,curan::ui::Key arg,curan::ui::ConfigDraw* draw){
			if (arg.key == GLFW_KEY_H && arg.action == GLFW_PRESS){
				if(draw->stack_page!=nullptr){
					draw->stack_page->stack(create_option_page(resources,minipage_pointer,mask_pointer));
				}
			}
		});

        auto minimage_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*minimage_container << std::move(minipage);

		curan::ui::Page page{std::move(minimage_container), SK_ColorBLACK};

		ConfigDraw config{&page};

		while (!glfwWindowShouldClose(viewer->window))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas *canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated())
			{
				page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				page.propagate_signal(signals.back(), &config);
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (const std::exception &e)
	{
		std::cout << "Exception thrown:" << e.what() << "\n";
	}
	catch (...)
	{
		std::cout << "Failed to create window for unknown reason\n";
		return 1;
	}
}