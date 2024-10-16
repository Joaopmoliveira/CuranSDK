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
% which we can then use draw widgets.

% For now we will create a simple page with a single 
% button for illustrative purpouses. 

auto button = Button::make("Connect",resources);
auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
								 Container::Arrangement::HORIZONTAL);
*container << std::move(button);

auto page = Page{std::move(container),SK_ColorBLACK};

% We need to trigger an update call with information from
% the viewer so that the page can propagate the true size
% of the viewer throught all the widgets

page.update_page(viewer.get());

% now we create a config draw object, which has a pointer
% inside it of the page. This is used when we wish to call
% callbacks from users so that they can manipulate the page

ConfigDraw config_draw{ &page};

% we can also limit the minimum size of the window, 
% for that we can request the page to compute the 
% minimum size it should have given all the widgets we 
% have placed inside it

viewer->set_minimum_size(page.minimum_size());

% now that we have everything we can loop until a request to close 
% we window has been requested

while (!glfwWindowShouldClose(viewer->window)) {
	% we time the loop cycle of the drawing call so that we can guarantee the drawing cycle time

	auto start = std::chrono::high_resolution_clock::now();

	% when we request a SkSurface the Window class request 
	% a free image in the swapchain so that we can render into ot

	SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
	SkCanvas* canvas = pointer_to_surface->getCanvas();

	% if we wanted we could draw everything 
	% unto the canvas, but we have tried to simplify
	% things, thus the code with the page
	% before drawing we check if the window has been resized
	% if it has we propagate this new size 
	% throught the widgets contained in the page 

	if (viewer->was_updated()) {
		page.update_page(viewer.get());
		viewer->update_processed();
	}

	% now we draw everythin on screen

	page.draw(canvas);

	% once we have drawn things, we check on the 
	% internal queue of the window if there are 
	% pending signals to process

	auto signals = viewer->process_pending_signals();

	% if there are we propagate them through the page

	if (!signals.empty())
		page.propagate_signal(signals.back(), &config_draw);

	% lastly we propagate an emtpy signal, so that widgets 
	% that want to keep track of time can count these empty signals

	page.propagate_heartbeat(&config_draw);
	glfwPollEvents();

	% lastly we can swap the page that we just rendered things into
	% back to VULKAN so that the actual rendering can begin. 
	% Eventually this will be shown on the screen
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
		, window_size_info{std::make_pair(width,height)}
	{}

	DisplayParams(std::unique_ptr<Context> cxt)
		: fColorType(kN32_SkColorType)
		, fColorSpace(SkColorSpace::MakeSRGB())
		, fMSAASampleCount(1)
		, fSurfaceProps(0, kRGB_H_SkPixelGeometry)
		, fDisableVsync(false)
		, fDelayDrawableAcquisition(false)
		, fEnableBinaryArchive(false)
		, cxt{ std::move(cxt) }
		, windowName{"NoName"}
		, window_size_info{true}
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
	std::variant<std::pair<int,int>,bool> window_size_info;
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
	std::unique_ptr<skgpu::VulkanBackendContext> vkContext = nullptr;
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