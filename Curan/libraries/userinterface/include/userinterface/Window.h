#ifndef CURAN_WINDOW_HEADER_FILE_
#define CURAN_WINDOW_HEADER_FILE_

#include "widgets/Signal.h"
#include <memory>
#include "utils/SafeQueue.h"
#include "Context.h"

namespace curan {
namespace ui {

/*
The Window class is by far on the more complicated 
side in terms of machinery used to control the 
interface between the operating system and our code. 

The usual loop to control a windows goes like 
this (comments start with % keyword): 

IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

% The IconResources is a class which loads icons to 
% be used by buttons, etc... 
% For it to load we need to pass a directory where these widgets are located

std::unique_ptr<Context> context = std::make_unique<Context>();

% The context contains the bridge between the GPU 
% context and our vulkan code, which uses SKIA to 
% generate drawing calls to the vulkan pipeline

DisplayParams param{ std::move(context),2200,1800 };
std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

% Now we have a window, which we can use to 
% obtain mouse events, and a swapchain to 
% which we can draw into
% Now comes the trycky part, 
% we will not go into detail, 
% but our library can generate drawing logic 
% which we can then use 

auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
page.update_page(viewer.get());

ConfigDraw config_draw{ &page};

viewer->set_minimum_size(page.minimum_size());

while (!glfwWindowShouldClose(viewer->window)) {
	auto start = std::chrono::high_resolution_clock::now();
	SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
	SkCanvas* canvas = pointer_to_surface->getCanvas();
	if (viewer->was_updated()) {
		page.update_page(viewer.get());
		viewer->update_processed();
	}
	page.draw(canvas);
	auto signals = viewer->process_pending_signals();

	if (!signals.empty())
		page.propagate_signal(signals.back(), &config_draw);
	page.propagate_heartbeat(&config_draw);
	glfwPollEvents();

	bool val = viewer->swapBuffers();
	if (!val)
		std::cout << "failed to swap buffers\n";
	auto end = std::chrono::high_resolution_clock::now();
	std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
}

There is alot of machinery in these lines, which we will try to explain as well as possible. 
*/

struct DisplayParams {
	DisplayParams(std::unique_ptr<Context> cxt, int width, int height)
		: fColorType(kN32_SkColorType)
		, fColorSpace(SkColorSpace::MakeSRGB())
		, fMSAASampleCount(1)
		, fSurfaceProps(0, kRGB_H_SkPixelGeometry)
		, fDisableVsync(false)
		, fDelayDrawableAcquisition(false)
		, fEnableBinaryArchive(false)
		, cxt{ std::move(cxt) }
		, windowName{"NoName"}
		, width{ width }
		, height{height}
	{}

	SkColorType         fColorType;
	sk_sp<SkColorSpace> fColorSpace;
	int                 fMSAASampleCount;
	GrContextOptions    fGrContextOptions;
	SkSurfaceProps      fSurfaceProps;
	bool                fDisableVsync;
	bool                fDelayDrawableAcquisition;
	bool                fEnableBinaryArchive;
	std::unique_ptr<Context> cxt;
	int width;
	int height;
	std::string windowName;
};

struct BackbufferInfo {
	uint32_t        fImageIndex;
	VkSemaphore     fRenderSemaphore;
};

class Window {
public:

	GLFWwindow* window = nullptr;
	utilities::SafeQueue<Signal> signal_queue;

	Window(DisplayParams&& pars);

	~Window();

	[[nodiscard]] bool swapBuffers();

	[[nodiscard]] SkSurface* getBackbufferSurface();

	void connect_handler();

	[[nodiscard]] bool initialize();

	void destroy();

	inline  SkRect get_size() const {
		SkRect rec = SkRect::MakeXYWH(0,0,static_cast<float>(width),static_cast<float>(height));
		return rec;
	}

	std::vector<Signal> process_pending_signals();

	[[nodiscard]] bool recreateDisplay();

	void set_minimum_size(SkRect minimum_size,float percent = 0.1);

	BackbufferInfo* getAvailableBackBuffer();

	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

	inline bool was_updated(){
		return user_space_was_updated;
	};
			 
	inline void update_processed(){
		user_space_was_updated = false;
	}

private:

	DisplayParams params;
	int width, height;
	std::unique_ptr<Context> context = nullptr;
	VkSurfaceKHR surface = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	VkQueue graphicsQueue = VK_NULL_HANDLE;
	VkQueue presentQueue = VK_NULL_HANDLE;
	std::unique_ptr<BackbufferInfo[]> fBackbuffers = nullptr;
	std::unique_ptr<GrVkBackendContext> vkContext = nullptr;
	std::unique_ptr<GrDirectContext> skia_context = nullptr;
	std::vector<SkSurface*> swapSurface;
	uint32_t fCurrentBackbufferIndex{ 0 };
	SkColorType colorType;
	VkSwapchainKHR swapChain = VK_NULL_HANDLE;
	std::vector<VkImage> swapChainImages;
	std::vector<VkImageLayout> swapChainImageLayout;
	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;
	VkImageUsageFlags usageFlags;
	size_t currentFrame = 0;
	bool framebufferResized = false;
	bool user_space_was_updated = false;
	std::string windowName;
};

}
}

#endif