#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Page.h"

#include "itkImage.h"


using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

ImageType::Pointer get_volume(size_t dimensions) {
  ImageType::Pointer memo = ImageType::New();

  ImageType::SizeType size;
  size[0] = dimensions;
  size[1] = dimensions;
  size[2] = dimensions;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::RegionType region;
  region.SetIndex(start);
  region.SetSize(size);

  const itk::SpacePrecisionType origin[Dimension] = {0.0, 0.0, 0.0};
  memo->SetOrigin(origin);
  const itk::SpacePrecisionType spacing[Dimension] = {1.0, 1.0, 1.0};
  memo->SetSpacing(spacing);
  memo->SetRegions(region);

  memo->Allocate();

  auto raw_buffer = memo->GetBufferPointer();
  for (size_t i = 0; i < dimensions * dimensions * dimensions; ++i)
    raw_buffer[i] = rand();

  return memo;
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

		VolumetricMask mask{get_volume(100)};
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