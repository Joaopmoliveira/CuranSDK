#include "userinterface/Window.h"
#include "utils/Logger.h"

namespace curan {
namespace ui {

const int MAX_FRAMES_IN_FLIGHT = 2;
const int EXTRA_BACK_BUFFER = 1;

Window::Window(DisplayParams&& pars) : params{ std::move(pars) }, width{ params.width }, height{ params.height }, windowName{params.windowName} {
	context = std::move(params.cxt);
	params.cxt = nullptr;
	bool val = initialize();
	if(!val)
		throw std::runtime_error("failed to initialize window");
	connect_handler();
};

Window::~Window()
{
	destroy();
};

void Window::set_minimum_size(SkRect minimum_size,float percent){
	if(percent< 0.0 || percent > 1.0)
		throw std::runtime_error("the maximum relative percentange for minimum size is between 0.0 and 1.0");
	glfwSetWindowSizeLimits(window,minimum_size.width()*(1+percent),minimum_size.height()*(1+percent),GLFW_DONT_CARE,GLFW_DONT_CARE);
}

bool Window::swapBuffers()
{
	BackbufferInfo* backbuffer = fBackbuffers.get() + fCurrentBackbufferIndex;
	SkSurface* surface = swapSurface[backbuffer->fImageIndex];

	GrBackendSemaphore beSemaphore;
	beSemaphore.initVulkan(backbuffer->fRenderSemaphore);

	GrFlushInfo info;
	info.fNumSemaphores = 1;
	info.fSignalSemaphores = &beSemaphore;
	//GrBackendSurfaceMutableState hi;
	GrBackendSurfaceMutableState presentState(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, context->indices.presentFamily.value());
	//skgpu::MutableTextureState presentState(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, context->indices.presentFamily.value());
	if (surface->flush(info, &presentState) != GrSemaphoresSubmitted::kYes) {
		return false;
	}
	if (surface->recordingContext()->asDirectContext()->submit() != true)
		return false;

	// Submit present operation to present queue
	const VkPresentInfoKHR presentInfo =
	{
		VK_STRUCTURE_TYPE_PRESENT_INFO_KHR, // sType
		nullptr, // pNext
		1, // waitSemaphoreCount
		&backbuffer->fRenderSemaphore, // pWaitSemaphores
		1, // swapchainCount
		&swapChain, // pSwapchains
		&backbuffer->fImageIndex, // pImageIndices
		nullptr // pResults
	};

	vkQueuePresentKHR(presentQueue, &presentInfo);
	return true;
}

SkSurface* Window::getBackbufferSurface()
{
	BackbufferInfo* backbuffer = getAvailableBackBuffer();
	SkASSERT(backbuffer);

	// semaphores should be in unsignaled state
	VkSemaphoreCreateInfo semaphoreInfo;
	memset(&semaphoreInfo, 0, sizeof(VkSemaphoreCreateInfo));
	semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreInfo.pNext = nullptr;
	semaphoreInfo.flags = 0;
	VkSemaphore semaphore;
	if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &semaphore) != VK_SUCCESS)
		throw std::runtime_error("Failed creating a semaphore");

	// acquire the image
	VkResult res = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, semaphore, VK_NULL_HANDLE, &backbuffer->fImageIndex);

	if (VK_ERROR_SURFACE_LOST_KHR == res) {
		// need to figure out how to create a new vkSurface without the platformData*
		// maybe use attach somehow? but need a Window
		vkDestroySemaphore(device, semaphore, nullptr);
		return nullptr;
	}

	if (VK_ERROR_OUT_OF_DATE_KHR == res || res == VK_SUBOPTIMAL_KHR || framebufferResized) {
		framebufferResized = false;
		// tear swapchain down and try again
		if(!recreateDisplay()){
			throw std::runtime_error("failed to recreate the display after a resize request");
		}
		backbuffer = getAvailableBackBuffer();

		// acquire the image
		res = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX,
			semaphore, VK_NULL_HANDLE, &backbuffer->fImageIndex);

		if (VK_SUCCESS != res) {
			vkDestroySemaphore(device, semaphore, nullptr);
			return nullptr;
		}
	}
	SkSurface* surface = swapSurface[backbuffer->fImageIndex];
	GrBackendSemaphore beSemaphore;
	beSemaphore.initVulkan(semaphore);
	surface->wait(1, &beSemaphore);
	return surface;
}

void Window::connect_handler() {
	glfwSetCursorPosCallback(this->window, cursor_position_callback);
	glfwSetMouseButtonCallback(this->window, cursor_position_click_callback);
	glfwSetScrollCallback(this->window, scroll_callback);
	glfwSetDropCallback(this->window, item_droped_callback);
}

bool Window::initialize()
{
	VkResult res;
	// create a glfw window
	window = glfwCreateWindow(width, height, windowName.c_str(), nullptr, nullptr);
	glfwSetWindowUserPointer(window, this);
	glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);

	// create a KHR surface
	if (glfwCreateWindowSurface(context->instance, window, nullptr, &surface) != VK_SUCCESS) {
		return false;
	}

	VkPhysicalDeviceFeatures deviceFeatures{};

	VkDeviceQueueCreateFlags flags = 0;
	float queuePriorities[1] = { 0.0 };
	// Here we assume no need for swapchain queue
	// If one is needed, the client will need its own setup code
	const VkDeviceQueueCreateInfo queueInfo[2] = {
		{
			VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, // sType
			nullptr,                                    // pNext
			flags,                                      // VkDeviceQueueCreateFlags
			context->indices.graphicsFamily.value(),                         // queueFamilyIndex
			1,                                          // queueCount
			queuePriorities,                            // pQueuePriorities
		},{
			VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, // sType
			nullptr,                                    // pNext
			0,                                          // VkDeviceQueueCreateFlags
			context->indices.presentFamily.value(),                          // queueFamilyIndex
			1,                                          // queueCount
			queuePriorities,                            // pQueuePriorities
		}
	};
	uint32_t queueInfoCount = (context->indices.graphicsFamily.value() != context->indices.presentFamily.value()) ? 2 : 1;

	VkDeviceCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	createInfo.queueCreateInfoCount = queueInfoCount;
	createInfo.pQueueCreateInfos = queueInfo;
	createInfo.pEnabledFeatures = &deviceFeatures;
	createInfo.enabledExtensionCount = (uint32_t)context->deviceExtensionNames.size();
	createInfo.ppEnabledExtensionNames = context->deviceExtensionNames.data();
	createInfo.enabledLayerCount = 0;

	res = vkCreateDevice(context->physicalDevice, &createInfo, nullptr, &device);

	if (res != VK_SUCCESS)
		return false;

	vkGetDeviceQueue(device, context->indices.graphicsFamily.value(), 0, &graphicsQueue);
	vkGetDeviceQueue(device, context->indices.presentFamily.value(), 0, &presentQueue);

	if (vkContext == nullptr)
		vkContext = std::make_unique<GrVkBackendContext>();

	vkContext->fInstance = context->instance;
	vkContext->fPhysicalDevice = context->physicalDevice;
	vkContext->fDevice = device;
	vkContext->fQueue = graphicsQueue;
	vkContext->fGraphicsQueueIndex = context->indices.graphicsFamily.value();
	vkContext->fMaxAPIVersion = VK_API_VERSION_1_0;
	vkContext->fVkExtensions = context->extensions.get();
	vkContext->fDeviceFeatures = nullptr;
	vkContext->fGetProc = context->vulkan_pointer;
	vkContext->fOwnsInstanceAndDevice = false;
	vkContext->fProtectedContext = GrProtected::kNo;

	// so far i dont think we have any special needs,
	// but this might change in the future
	GrContextOptions options;

	skia_context = std::unique_ptr<GrDirectContext>(GrDirectContext::MakeVulkan(*vkContext, options).release());
	if (skia_context == nullptr)
		return false;

	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;

	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(context->physicalDevice, surface, &capabilities);

	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(context->physicalDevice, surface, &formatCount, nullptr);

	if (formatCount != 0) {
		formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(context->physicalDevice, surface, &formatCount, formats.data());
	}

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(context->physicalDevice, surface, &presentModeCount, nullptr);

	if (presentModeCount != 0) {
		presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(context->physicalDevice, surface, &presentModeCount, presentModes.data());
	}

	VkSurfaceFormatKHR format;

	// Pick our surface format.
	VkFormat surfaceFormat = VK_FORMAT_UNDEFINED;
	VkColorSpaceKHR colorSpace = VK_COLORSPACE_SRGB_NONLINEAR_KHR;
	for (uint32_t i = 0; i < formats.size(); ++i) {
		VkFormat localFormat = formats[i].format;
		bool is_format_supported = false;
		switch (localFormat) {
		case VK_FORMAT_R8G8B8A8_UNORM:
		case VK_FORMAT_B8G8R8A8_UNORM:
		case VK_FORMAT_R8G8B8A8_SRGB:
		case VK_FORMAT_R8G8B8_UNORM:
		case VK_FORMAT_R8G8_UNORM:
		case VK_FORMAT_A2B10G10R10_UNORM_PACK32:
		case VK_FORMAT_A2R10G10B10_UNORM_PACK32:
		case VK_FORMAT_R5G6B5_UNORM_PACK16:
		case VK_FORMAT_B4G4R4A4_UNORM_PACK16:
		case VK_FORMAT_R4G4B4A4_UNORM_PACK16:
		case VK_FORMAT_R8_UNORM:
		case VK_FORMAT_ETC2_R8G8B8_UNORM_BLOCK:
		case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
		case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
		case VK_FORMAT_R16G16B16A16_SFLOAT:
		case VK_FORMAT_R16_SFLOAT:
		case VK_FORMAT_R16_UNORM:
		case VK_FORMAT_R16G16_UNORM:
		case VK_FORMAT_G8_B8_R8_3PLANE_420_UNORM:
		case VK_FORMAT_G8_B8R8_2PLANE_420_UNORM:
		case VK_FORMAT_R16G16B16A16_UNORM:
		case VK_FORMAT_R16G16_SFLOAT:
		case VK_FORMAT_S8_UINT:
		case VK_FORMAT_D24_UNORM_S8_UINT:
		case VK_FORMAT_D32_SFLOAT_S8_UINT:
			is_format_supported = true;
			break;
		default:
			is_format_supported = false;
			break;
		}
		if (is_format_supported) {
			surfaceFormat = localFormat;
			colorSpace = formats[i].colorSpace;
			break;
		}
	}

	format.format = surfaceFormat;
	format.colorSpace = colorSpace;

	switch (surfaceFormat) {
	case VK_FORMAT_R8G8B8A8_UNORM: // fall through
	case VK_FORMAT_R8G8B8A8_SRGB:
		colorType = kRGBA_8888_SkColorType;
		break;
	case VK_FORMAT_B8G8R8A8_UNORM: // fall through
		colorType = kBGRA_8888_SkColorType;
		break;
	default:
		format.format = VK_FORMAT_UNDEFINED;
		format.colorSpace = VK_COLORSPACE_SRGB_NONLINEAR_KHR;
	}

	// check if the color space is undefined, and if yes terminate application
	VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
	for (const auto& availablePresentMode : presentModes) {
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
			presentMode = availablePresentMode;
		}
	}

	VkExtent2D extent;

	if (capabilities.currentExtent.width != UINT32_MAX) {
		extent = capabilities.currentExtent;
	} else {
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);

		VkExtent2D actualExtent = {
			static_cast<uint32_t>(width),
			static_cast<uint32_t>(height)
		};

		actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
		actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

		extent = actualExtent;
	}

	uint32_t imageCount = capabilities.minImageCount + 2;
	if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
		imageCount = capabilities.maxImageCount;
	}

	// google stuff

	VkImageUsageFlags usageFlags = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
		VK_IMAGE_USAGE_TRANSFER_DST_BIT;

	SkASSERT((capabilities.supportedUsageFlags & usageFlags) == usageFlags);

	if (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT) {
		usageFlags |= VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
	}
	if (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
		usageFlags |= VK_IMAGE_USAGE_SAMPLED_BIT;
	}

	SkASSERT(capabilities.supportedTransforms & capabilities.currentTransform);
	SkASSERT(capabilities.supportedCompositeAlpha & (VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR |
		VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR));

	VkCompositeAlphaFlagBitsKHR composite_alpha =
		(capabilities.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR) ?
		VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR :
		VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

	VkSwapchainCreateInfoKHR swapchainCreateInfo{};
	swapchainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	swapchainCreateInfo.surface = surface;
	swapchainCreateInfo.minImageCount = imageCount;
	swapchainCreateInfo.imageFormat = format.format;
	swapchainCreateInfo.imageColorSpace = format.colorSpace;
	swapchainCreateInfo.imageExtent = extent;
	swapchainCreateInfo.imageArrayLayers = 1;
	swapchainCreateInfo.imageUsage = usageFlags;

	uint32_t queueFamilyIndices[] = { context->indices.graphicsFamily.value(), context->indices.presentFamily.value() };

	if (context->indices.graphicsFamily != context->indices.presentFamily) {
		swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		swapchainCreateInfo.queueFamilyIndexCount = 2;
		swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
	} else {
		swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		swapchainCreateInfo.queueFamilyIndexCount = 0;
		swapchainCreateInfo.pQueueFamilyIndices = nullptr;
	}

	swapchainCreateInfo.preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
	swapchainCreateInfo.compositeAlpha = composite_alpha;
	swapchainCreateInfo.presentMode = presentMode;
	swapchainCreateInfo.clipped = VK_TRUE;
	swapchainCreateInfo.oldSwapchain = swapChain;
	if (vkCreateSwapchainKHR(device, &swapchainCreateInfo, nullptr, &swapChain) != VK_SUCCESS) {
		return false;
	}

	// destroy the old swapchain
	if (swapchainCreateInfo.oldSwapchain != VK_NULL_HANDLE) {
		vkDeviceWaitIdle(device);

		if (fBackbuffers) {
			for (uint32_t i = 0; i < swapSurface.size() + EXTRA_BACK_BUFFER; ++i) {
				fBackbuffers[i].fImageIndex = -1;
				vkDestroySemaphore(device, fBackbuffers[i].fRenderSemaphore, nullptr);
			}
		}
		if (fBackbuffers)
			fBackbuffers.reset();

		vkDestroySwapchainKHR(device, swapchainCreateInfo.oldSwapchain, nullptr);
	}
	swapChainImageFormat = format.format;
	swapChainExtent = extent;

	// create the backbuffers
	imageCount = 0;
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
	if (imageCount == 0)
		throw std::runtime_error("No images present in the swapchain");
	swapChainImages.resize(imageCount);
	swapChainImageLayout.resize(imageCount);
	swapSurface.resize(imageCount);

	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

	for (uint32_t i = 0; i < imageCount; ++i) {
		swapChainImageLayout[i] = VK_IMAGE_LAYOUT_UNDEFINED;

		GrVkImageInfo info;
		info.fImage = swapChainImages[i];
		info.fAlloc = GrVkAlloc();
		info.fImageLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		info.fImageTiling = VK_IMAGE_TILING_OPTIMAL;
		info.fFormat = swapChainImageFormat;
		info.fLevelCount = 1;
		info.fCurrentQueueFamily = context->indices.presentFamily.value();

		params.fColorSpace = SkColorSpace::MakeSRGBLinear();
		if (usageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
			GrBackendTexture backendTexture(swapChainExtent.width, swapChainExtent.height, info);
			auto localsksurface = SkSurface::MakeFromBackendTexture(
				skia_context.get(), backendTexture, kTopLeft_GrSurfaceOrigin,
				params.fMSAASampleCount,
				colorType, params.fColorSpace, &params.fSurfaceProps);
			swapSurface[i] = localsksurface.release();
		} else {
			if (params.fMSAASampleCount != 1) {
				throw std::runtime_error("Could not deal with input definitions");
			}
			GrBackendRenderTarget backendRT(swapChainExtent.width, swapChainExtent.height, params.fMSAASampleCount, info);
			swapSurface[i] = SkSurface::MakeFromBackendRenderTarget(
				skia_context.get(), backendRT, kTopLeft_GrSurfaceOrigin, colorType,
				params.fColorSpace, &params.fSurfaceProps).release();
		}
		if (!swapSurface[i]) {
			throw std::runtime_error("Surface could not be created for unknown reason");
		}
	}

	// set up the backbuffers
	VkSemaphoreCreateInfo semaphoreInfo;
	memset(&semaphoreInfo, 0, sizeof(VkSemaphoreCreateInfo));
	semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreInfo.pNext = nullptr;
	semaphoreInfo.flags = 0;

	// we create one additional backbuffer structure here, because we want to
	// give the command buffers they contain a chance to finish before we cycle back
	fBackbuffers = std::unique_ptr<BackbufferInfo[]>(new BackbufferInfo[imageCount + EXTRA_BACK_BUFFER]);
	for (uint32_t i = 0; i < imageCount + EXTRA_BACK_BUFFER; ++i) {
		fBackbuffers[i].fImageIndex = -1;
		if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &fBackbuffers[i].fRenderSemaphore) != VK_SUCCESS)
			throw std::runtime_error("failed to create synchronization objects for a frame!");
	}
	fCurrentBackbufferIndex = imageCount;
	return true;
}

void Window::destroy()
{
	vkDeviceWaitIdle(device);

	if (fBackbuffers) {
		for (uint32_t i = 0; i < swapSurface.size() + EXTRA_BACK_BUFFER; ++i) {
			fBackbuffers[i].fImageIndex = -1;
			vkDestroySemaphore(device, fBackbuffers[i].fRenderSemaphore, nullptr);
		}
	}

	for (auto& pointer_to_surface : swapSurface)
		if (pointer_to_surface != nullptr)
			delete pointer_to_surface;
	swapSurface.clear();

	if (skia_context) {
		skia_context->abandonContext();
		skia_context.reset();
	}
	utilities::cout << "destroying swapchain";
	vkDestroySwapchainKHR(device, swapChain, nullptr);
	vkDestroyDevice(device, nullptr);
	vkDestroySurfaceKHR(context->instance, surface, nullptr);
	utilities::cout << "destroyed swapchain";
	utilities::cout << "destroying window";
	glfwDestroyWindow(window);
	utilities::cout << "destroyed window";
	return;
}

std::vector<Signal> Window::process_pending_signals()
{
	std::vector<Signal> received_signals;
	int size = signal_queue.size();
	received_signals.reserve(size);
	for (int index = 0; index < size; ++index) {
		Signal signal;
		bool val = signal_queue.try_pop(signal);
		if(val)
			received_signals.push_back(signal);
	}
	return received_signals;
}

bool Window::recreateDisplay()
{
	glfwGetFramebufferSize(window, &width, &height);
	while (width == 0 || height == 0) {
		glfwGetFramebufferSize(window, &width, &height);
		glfwWaitEvents();
	}

	vkDeviceWaitIdle(device);

	if (fBackbuffers) {
		for (uint32_t i = 0; i < swapSurface.size() + EXTRA_BACK_BUFFER; ++i) {
			fBackbuffers[i].fImageIndex = -1;
			vkDestroySemaphore(device, fBackbuffers[i].fRenderSemaphore, nullptr);
		}
	}
	if (fBackbuffers)
		fBackbuffers.reset();

	for (auto pointer_to_surface : swapSurface)
		if (pointer_to_surface != nullptr)
			delete pointer_to_surface;
	swapSurface.clear();

	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;

	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(context->physicalDevice, surface, &capabilities);

	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(context->physicalDevice, surface, &formatCount, nullptr);

	if (formatCount != 0) {
		formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(context->physicalDevice, surface, &formatCount, formats.data());
	}

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(context->physicalDevice, surface, &presentModeCount, nullptr);

	if (presentModeCount != 0) {
		presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(context->physicalDevice, surface, &presentModeCount, presentModes.data());
	}

	VkSurfaceFormatKHR format;

	// Pick our surface format.
	VkFormat surfaceFormat = VK_FORMAT_UNDEFINED;
	VkColorSpaceKHR colorSpace = VK_COLORSPACE_SRGB_NONLINEAR_KHR;
	for (uint32_t i = 0; i < formats.size(); ++i) {
		VkFormat localFormat = formats[i].format;
		bool is_format_supported = false;
		switch (localFormat) {
		case VK_FORMAT_R8G8B8A8_UNORM:
		case VK_FORMAT_B8G8R8A8_UNORM:
		case VK_FORMAT_R8G8B8A8_SRGB:
		case VK_FORMAT_R8G8B8_UNORM:
		case VK_FORMAT_R8G8_UNORM:
		case VK_FORMAT_A2B10G10R10_UNORM_PACK32:
		case VK_FORMAT_A2R10G10B10_UNORM_PACK32:
		case VK_FORMAT_R5G6B5_UNORM_PACK16:
		case VK_FORMAT_B4G4R4A4_UNORM_PACK16:
		case VK_FORMAT_R4G4B4A4_UNORM_PACK16:
		case VK_FORMAT_R8_UNORM:
		case VK_FORMAT_ETC2_R8G8B8_UNORM_BLOCK:
		case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
		case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
		case VK_FORMAT_R16G16B16A16_SFLOAT:
		case VK_FORMAT_R16_SFLOAT:
		case VK_FORMAT_R16_UNORM:
		case VK_FORMAT_R16G16_UNORM:
		case VK_FORMAT_G8_B8_R8_3PLANE_420_UNORM:
		case VK_FORMAT_G8_B8R8_2PLANE_420_UNORM:
		case VK_FORMAT_R16G16B16A16_UNORM:
		case VK_FORMAT_R16G16_SFLOAT:
		case VK_FORMAT_S8_UINT:
		case VK_FORMAT_D24_UNORM_S8_UINT:
		case VK_FORMAT_D32_SFLOAT_S8_UINT:
			is_format_supported = true;
			break;
		default:
			is_format_supported = false;
			break;
		}
		if (is_format_supported) {
			surfaceFormat = localFormat;
			colorSpace = formats[i].colorSpace;
			break;
		}
	}

	format.format = surfaceFormat;
	format.colorSpace = colorSpace;

	switch (surfaceFormat) {
	case VK_FORMAT_R8G8B8A8_UNORM: // fall through
	case VK_FORMAT_R8G8B8A8_SRGB:
		colorType = kRGBA_8888_SkColorType;
		break;
	case VK_FORMAT_B8G8R8A8_UNORM: // fall through
		colorType = kBGRA_8888_SkColorType;
		break;
	default:
		format.format = VK_FORMAT_UNDEFINED;
		format.colorSpace = VK_COLORSPACE_SRGB_NONLINEAR_KHR;
		break;
	}

	// check if the color space is undefined, and if yes terminate application
	VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
	for (const auto& availablePresentMode : presentModes) {
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
			presentMode = availablePresentMode;
		}
	}

	VkExtent2D extent;

	if (capabilities.currentExtent.width != UINT32_MAX) {
		extent = capabilities.currentExtent;
	} else {
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);

		VkExtent2D actualExtent = {
			static_cast<uint32_t>(width),
			static_cast<uint32_t>(height)
		};

		actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
		actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));
		extent = actualExtent;
	}

	uint32_t imageCount = capabilities.minImageCount + 2;
	if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
		imageCount = capabilities.maxImageCount;
	}

	// google stuff

	VkImageUsageFlags usageFlags = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
		VK_IMAGE_USAGE_TRANSFER_DST_BIT;

	SkASSERT((capabilities.supportedUsageFlags & usageFlags) == usageFlags);

	if (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT) {
		usageFlags |= VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
	}
	if (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
		usageFlags |= VK_IMAGE_USAGE_SAMPLED_BIT;
	}

	SkASSERT(capabilities.supportedTransforms & capabilities.currentTransform);
	SkASSERT(capabilities.supportedCompositeAlpha & (VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR |
		VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR));

	VkCompositeAlphaFlagBitsKHR composite_alpha =
		(capabilities.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR) ?
			VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR : VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

	VkSwapchainCreateInfoKHR swapchainCreateInfo{};
	swapchainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	swapchainCreateInfo.surface = surface;
	swapchainCreateInfo.minImageCount = imageCount;
	swapchainCreateInfo.imageFormat = format.format;
	swapchainCreateInfo.imageColorSpace = format.colorSpace;
	swapchainCreateInfo.imageExtent = extent;
	swapchainCreateInfo.imageArrayLayers = 1;
	swapchainCreateInfo.imageUsage = usageFlags;

	uint32_t queueFamilyIndices[] = { context->indices.graphicsFamily.value(), context->indices.presentFamily.value() };

	if (context->indices.graphicsFamily != context->indices.presentFamily) {
		swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		swapchainCreateInfo.queueFamilyIndexCount = 2;
		swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
	} else {
		swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		swapchainCreateInfo.queueFamilyIndexCount = 0;
		swapchainCreateInfo.pQueueFamilyIndices = nullptr;
	}

	swapchainCreateInfo.preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
	swapchainCreateInfo.compositeAlpha = composite_alpha;
	swapchainCreateInfo.presentMode = presentMode;
	swapchainCreateInfo.clipped = VK_TRUE;
	swapchainCreateInfo.oldSwapchain = swapChain;
	if (vkCreateSwapchainKHR(device, &swapchainCreateInfo, nullptr, &swapChain) != VK_SUCCESS) {
		return false;
	}

	// destroy the old swapchain
	if (swapchainCreateInfo.oldSwapchain != VK_NULL_HANDLE) {
		vkDeviceWaitIdle(device);

		if (fBackbuffers) {
			for (uint32_t i = 0; i < swapSurface.size() + EXTRA_BACK_BUFFER; ++i) {
				fBackbuffers[i].fImageIndex = -1;
				vkDestroySemaphore(device, fBackbuffers[i].fRenderSemaphore, nullptr);
			}
		}
		if (fBackbuffers)
			fBackbuffers.reset();

		vkDestroySwapchainKHR(device, swapchainCreateInfo.oldSwapchain, nullptr);
	}
	swapChainImageFormat = format.format;
	swapChainExtent = extent;

	imageCount = 0;
	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
	if (imageCount == 0)
		throw std::runtime_error("No images present in the swapchain");
	swapChainImages.resize(imageCount);
	swapChainImageLayout.resize(imageCount);
	swapSurface.resize(imageCount);

	vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

	for (uint32_t i = 0; i < imageCount; ++i) {
		swapChainImageLayout[i] = VK_IMAGE_LAYOUT_UNDEFINED;

		GrVkImageInfo info;
		info.fImage = swapChainImages[i];
		info.fAlloc = GrVkAlloc();
		info.fImageLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		info.fImageTiling = VK_IMAGE_TILING_OPTIMAL;
		info.fFormat = swapChainImageFormat;
		info.fLevelCount = 1;
		info.fCurrentQueueFamily = context->indices.presentFamily.value();

		if (usageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
			GrBackendTexture backendTexture(swapChainExtent.width, swapChainExtent.height, info);
			swapSurface[i] = SkSurface::MakeFromBackendTexture(
				skia_context.get(), backendTexture, kTopLeft_GrSurfaceOrigin,
						params.fMSAASampleCount,
						colorType, params.fColorSpace, &params.fSurfaceProps).release();
		} else {
			if (params.fMSAASampleCount != 1) {
				throw std::runtime_error("Could not deal with input definitions");
			}
			GrBackendRenderTarget backendRT(swapChainExtent.width, swapChainExtent.height, params.fMSAASampleCount, info);
			swapSurface[i] = SkSurface::MakeFromBackendRenderTarget(
				skia_context.get(), backendRT, kTopLeft_GrSurfaceOrigin, colorType,
					params.fColorSpace, &params.fSurfaceProps).release();
		}
		if (!swapSurface[i]) {
			throw std::runtime_error("Surface could not be created for unknown reason");
		}
	}

	// set up the backbuffers
	VkSemaphoreCreateInfo semaphoreInfo;
	memset(&semaphoreInfo, 0, sizeof(VkSemaphoreCreateInfo));
	semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreInfo.pNext = nullptr;
	semaphoreInfo.flags = 0;

	// we create one additional backbuffer structure here, because we want to
	// give the command buffers they contain a chance to finish before we cycle back
	fBackbuffers = std::unique_ptr<BackbufferInfo[]>(new BackbufferInfo[imageCount + EXTRA_BACK_BUFFER]);
	for (uint32_t i = 0; i < imageCount + EXTRA_BACK_BUFFER; ++i) {
		fBackbuffers[i].fImageIndex = -1;
		if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &fBackbuffers[i].fRenderSemaphore) != VK_SUCCESS)
			throw std::runtime_error("failed to create synchronization objects for a frame!");
	}
	fCurrentBackbufferIndex = imageCount;
	return true;
}

BackbufferInfo* Window::getAvailableBackBuffer()
{
	if (fBackbuffers == nullptr)
		throw std::runtime_error("Pointer to back buffers is lost");
	++fCurrentBackbufferIndex;
	if (fCurrentBackbufferIndex > swapSurface.size()) {
		fCurrentBackbufferIndex = 0;
	}
	BackbufferInfo* backbuffer = fBackbuffers.get() + fCurrentBackbufferIndex;
	return backbuffer;
}

void Window::framebufferResizeCallback(GLFWwindow* window, int width, int height)
{
	auto app = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
	app->framebufferResized = true;
}

}
}