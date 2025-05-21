
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Drawable.h"

#include <unordered_map>
#include <optional>
#include <functional>


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
		curan::geometry::ClosedCylinder cube{2,100,1.0,1.0};
		mask.add_geometry(cube);
		std::unique_ptr<curan::ui::SlidingPanel> image_display = curan::ui::SlidingPanel::make(resources, &mask, curan::ui::Direction::Z);

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::VERTICAL);
		*container << std::move(image_display);

		curan::ui::Page page{std::move(container), SK_ColorBLACK};

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