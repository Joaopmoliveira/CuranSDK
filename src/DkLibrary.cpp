#include "DkLibrary.h"

namespace curan
{
	namespace display {
		Layout::Layout(SkPaint in_paint_layout, Arrangement in_arrangment)
		{
			paint_layout = in_paint_layout;
			arrangment = in_arrangment;
		}

		void Layout::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
		{
			SkRect rec = SkRect::MakeLTRB(drawing_area.fLeft * window_size.fLeft,
				drawing_area.fTop * window_size.fTop,
				drawing_area.fRight * window_size.fRight,
				drawing_area.fBottom * window_size.fBottom);
			canvas_to_draw->drawRect(rec, paint_layout);
		}

		Page::Page(Info& info)
		{
			paint_page = info.paint_page;
			page_layout = info.page_layout;
			intialization_callback = info.initialize_page_callback;
		}

		void Page::draw(SkCanvas* canvas_to_draw, SkRect& window_size)
		{
			SkSurface* surf = canvas_to_draw->getSurface();
			SkRect total_size = SkRect::MakeIWH(surf->width(), surf->height());
			SkRect range = SkRect::MakeLTRB(window_size.fLeft/ surf->width(), window_size.fTop/ surf->height(), window_size.fRight/ surf->width(), window_size.fBottom/ surf->height());
			page_layout->draw(canvas_to_draw, range, total_size);
		}

		std::shared_ptr<Page> Page::make(Info& info)
		{
			return std::make_shared<Page>(info);
		}

		void Page::callback(Signal signal, bool* interacted)
		{
			for (auto widg : widgets_to_callback)
				widg->callback(signal,interacted);
		}

		void Page::monitor(std::shared_ptr<Widget> widg)
		{
			widgets_to_callback.push_back(widg);
		}

		void Page::started()
		{
			if (intialization_callback)
				intialization_callback();
		}

		LayoutLinearContainer::LayoutLinearContainer(Info& info) : Layout(info.paint_layout, info.arrangement)
		{
			contained_layouts = info.layouts;

			if (contained_layouts.size() == 0)
				return;
			SkScalar packet_width = 1.0f / contained_layouts.size();

			if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_layouts.size())) {
				switch (arrangment) {
				case Arrangement::HORIZONTAL:
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(info.divisions[i], 0, info.divisions[i + 1], 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				case Arrangement::VERTICAL:
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(0, info.divisions[i], 1, info.divisions[i + 1]);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				default:

					break;
				}
			}
			else {
				switch (arrangment) {
				case Arrangement::HORIZONTAL:
					for (SkScalar left_position = 0.0; left_position < 1.0; left_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(left_position, 0, left_position + packet_width, 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				case Arrangement::VERTICAL:
					for (SkScalar top_position = 0.0; top_position < 1.0; top_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(0, top_position, 1, top_position + packet_width);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				default:

					break;
				}
			}
		}

		void LayoutLinearContainer::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
		{
			SkRect rectangle = SkRect::MakeLTRB(drawing_area.fLeft * window_size.width(),
				drawing_area.fTop * window_size.height(),
				drawing_area.fRight * window_size.width(),
				drawing_area.fBottom * window_size.height());
			canvas_to_draw->drawRect(rectangle, paint_layout);
			for (int i = 0; i < rectangles_of_contained_layouts.size(); ++i) {
				SkRect rect = rectangles_of_contained_layouts[i];
				SkRect temp = rect.MakeXYWH(drawing_area.x() + rect.x() * drawing_area.width(),
					drawing_area.y() + rect.y() * drawing_area.height(),
					rect.width() * drawing_area.width(),
					rect.height() * drawing_area.height());

				SkRect temp2 = temp;
				temp2.fBottom = temp2.fBottom * window_size.height();
				temp2.fLeft = temp2.fLeft * window_size.width();
				temp2.fRight = temp2.fRight * window_size.width();
				temp2.fTop = temp2.fTop * window_size.height();
				contained_layouts[i]->draw(canvas_to_draw, temp, window_size);
			}
		}

		std::shared_ptr<LayoutLinearContainer> LayoutLinearContainer::make(Info& info)
		{
			return std::make_shared<LayoutLinearContainer>(info);
		}

		const std::vector<const char*> validationLayers = {
			"VK_LAYER_KHRONOS_validation"
		};

		const std::vector<const char*> deviceExtensions = {
			VK_KHR_SWAPCHAIN_EXTENSION_NAME
		};



		VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
			auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
			if (func != nullptr) {
				return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
			}
			else {
				return VK_ERROR_EXTENSION_NOT_PRESENT;
			}
		}

		void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
			auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
			if (func != nullptr) {
				func(instance, debugMessenger, pAllocator);
			}
		}

		Window::Window(DisplayParams& pars)
		{
			params = pars;
		};

		Window::~Window()
		{};

		void Window::swapBuffers()
		{
			BackbufferInfo* backbuffer = fBackbuffers + fCurrentBackbufferIndex;
			SkSurface* surface = swapSurface[backbuffer->fImageIndex];

			GrBackendSemaphore beSemaphore;
			beSemaphore.initVulkan(backbuffer->fRenderSemaphore);

			GrFlushInfo info;
			info.fNumSemaphores = 1;
			info.fSignalSemaphores = &beSemaphore;
			GrBackendSurfaceMutableState presentState(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, context->indices.presentFamily.value());
			if (surface->flush(info, &presentState) != GrSemaphoresSubmitted::kYes) {
				std::cerr << "The semaphores are no longer valid, need to recreate them";
			}
			if (surface->recordingContext()->asDirectContext()->submit() != true)
				std::cerr << "the semaphores are no longer valid";

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
				recreateDisplay();
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

		void Window::add_page(std::shared_ptr<Page> page)
		{
			contained_pages.push_back(page);
		}

		void Window::draw(SkCanvas* canvas, int page_index)
		{
			static int previews_page_index = 0;
			SkRect rect = SkRect::MakeLTRB(0, 0, swapChainExtent.width, swapChainExtent.height);
			if (previews_page_index != page_index) {
				contained_pages[page_index]->started();
				previews_page_index = page_index;
			}
			contained_pages[page_index]->draw(canvas, rect);
		}

		void Window::connect_handler() {
			glfwSetCursorPosCallback(this->window, cursor_position_callback);
			glfwSetMouseButtonCallback(this->window, cursor_position_click_callback);
			glfwSetScrollCallback(this->window, scroll_callback);
			glfwSetDropCallback(this->window, item_droped_callback);
		}

		bool Window::initialize(Context* in_context)
		{
			VkResult res;
			this->context = in_context;
			// create a glfw window
			window = glfwCreateWindow(WIDTH, HEIGHT, "Curan", nullptr, nullptr);
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

				},
				{
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
				vkContext = new GrVkBackendContext;

			vkContext->fInstance = context->instance;
			vkContext->fPhysicalDevice = context->physicalDevice;
			vkContext->fDevice = device;
			vkContext->fQueue = graphicsQueue;
			vkContext->fGraphicsQueueIndex = context->indices.graphicsFamily.value();
			vkContext->fMaxAPIVersion = VK_API_VERSION_1_0;
			vkContext->fVkExtensions = context->extensions;
			vkContext->fDeviceFeatures = nullptr;
			vkContext->fGetProc = context->vulkan_pointer;
			vkContext->fOwnsInstanceAndDevice = false;
			vkContext->fProtectedContext = GrProtected::kNo;

			// so far i dont think we have any special needs,
			// but this might change in the future
			GrContextOptions options;

			skia_context = GrDirectContext::MakeVulkan(*vkContext, options).release();
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
			}
			else {

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
			}
			else {
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
					delete[] fBackbuffers;
				fBackbuffers = nullptr;

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

				if (usageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
					GrBackendTexture backendTexture(swapChainExtent.width, swapChainExtent.height, info);
					swapSurface[i] = SkSurface::MakeFromBackendTexture(
						skia_context, backendTexture, kTopLeft_GrSurfaceOrigin,
						params.fMSAASampleCount,
						colorType, params.fColorSpace, &params.fSurfaceProps).release();
				}
				else {
					if (params.fMSAASampleCount != 1) {
						throw std::runtime_error("Could not deal with input definitions");
					}
					GrBackendRenderTarget backendRT(swapChainExtent.width, swapChainExtent.height, params.fMSAASampleCount, info);
					swapSurface[i] = SkSurface::MakeFromBackendRenderTarget(
						skia_context, backendRT, kTopLeft_GrSurfaceOrigin, colorType,
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
			fBackbuffers = new BackbufferInfo[imageCount + EXTRA_BACK_BUFFER];
			for (uint32_t i = 0; i < imageCount + EXTRA_BACK_BUFFER; ++i) {
				fBackbuffers[i].fImageIndex = -1;
				if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &fBackbuffers[i].fRenderSemaphore) != VK_SUCCESS)
					throw std::runtime_error("failed to create synchronization objects for a frame!");
			}
			fCurrentBackbufferIndex = imageCount;
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
			if (fBackbuffers)
				delete[] fBackbuffers;
			fBackbuffers = nullptr;

			for (auto pointer_to_surface : swapSurface)
				if (pointer_to_surface != nullptr)
					delete pointer_to_surface;
			swapSurface.clear();

			if (vkContext != nullptr) {
				delete vkContext;
				vkContext = nullptr;
			}

			if (skia_context != nullptr) {
				skia_context->abandonContext();
				delete skia_context;
				skia_context = nullptr;
			}

			vkDestroySwapchainKHR(device, swapChain, nullptr);
			vkDestroyDevice(device, nullptr);
			vkDestroySurfaceKHR(context->instance, surface, nullptr);
			glfwDestroyWindow(window);

			return;
		}

		void Window::process_pending_signals(int page_index)
		{
			int size = signal_queue.size();
			for (int index = 0; index < size; ++index) {
				Signal signal = signal_queue.try_pop();
				if (signal.signal_type == Signal::Type::EMPTY_SIGNAL)
					continue;
				bool interacted = false;
				contained_pages[page_index]->callback(signal, &interacted);
			}
		}

		bool Window::recreateDisplay()
		{
			int width = 0, height = 0;
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
				delete[] fBackbuffers;
			fBackbuffers = nullptr;

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
			}
			else {

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
			}
			else {
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
					delete[] fBackbuffers;
				fBackbuffers = nullptr;

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
						skia_context, backendTexture, kTopLeft_GrSurfaceOrigin,
						params.fMSAASampleCount,
						colorType, params.fColorSpace, &params.fSurfaceProps).release();
				}
				else {
					if (params.fMSAASampleCount != 1) {
						throw std::runtime_error("Could not deal with input definitions");
					}
					GrBackendRenderTarget backendRT(swapChainExtent.width, swapChainExtent.height, params.fMSAASampleCount, info);
					swapSurface[i] = SkSurface::MakeFromBackendRenderTarget(
						skia_context, backendRT, kTopLeft_GrSurfaceOrigin, colorType,
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
			fBackbuffers = new BackbufferInfo[imageCount + EXTRA_BACK_BUFFER];
			for (uint32_t i = 0; i < imageCount + EXTRA_BACK_BUFFER; ++i) {
				fBackbuffers[i].fImageIndex = -1;
				if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &fBackbuffers[i].fRenderSemaphore) != VK_SUCCESS)
					throw std::runtime_error("failed to create synchronization objects for a frame!");
			}
			fCurrentBackbufferIndex = imageCount;
		}

		BackbufferInfo* Window::getAvailableBackBuffer()
		{
			if (fBackbuffers == nullptr)
				throw std::runtime_error("Pointer to back buffers is lost");
			++fCurrentBackbufferIndex;
			if (fCurrentBackbufferIndex > swapSurface.size()) {
				fCurrentBackbufferIndex = 0;
			}

			BackbufferInfo* backbuffer = fBackbuffers + fCurrentBackbufferIndex;
			return backbuffer;
		}

		void Window::framebufferResizeCallback(GLFWwindow* window, int width, int height)
		{
			auto app = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			app->framebufferResized = true;
		}


		void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
			Signal::Data data;
			data.move_signal = MouseMoveSignal{ xpos,ypos };
			Signal received_signal{ Signal::Type::MOUSE_MOVE_SIGNAL,data };
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		};

		void cursor_position_click_callback(GLFWwindow* window, int button, int action, int mods)
		{
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			Signal::Type type_of_signal;
			switch (action)
			{
			case GLFW_PRESS:
				type_of_signal = Signal::Type::MOUSE_PRESS_SIGNAL;
				break;
			case GLFW_RELEASE:
				type_of_signal = Signal::Type::MOUSE_UNPRESS_SIGNAL;
				break;
			}
			Signal::Data data;
			data.press_signal = MousePressSignal{ xpos,ypos };
			Signal received_signal{ type_of_signal ,data };
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		};

		void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
		{
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			Signal::Data data;
			data.scroll_signal = ScrollSignal{ xpos,ypos,xoffset,yoffset };
			Signal received_signal{ Signal::Type::SCROLL_SIGNAL ,data };
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		};

		void item_droped_callback(GLFWwindow* window, int count, const char** paths)
		{
			char** deep_copy_paths = new char* [count];
			for (int index = 0; index < count; ++index) {
				size_t length = strlen(paths[index]);
				deep_copy_paths[index] = new char[length + 1];
				strcpy(deep_copy_paths[index], paths[index]);
			}
			Signal::Data data;
			data.dropped_signal.count = count;
			data.dropped_signal.paths = deep_copy_paths;
			Signal received_signal{ Signal::Type::ITEM_DROPED_SIGNAL ,data };
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		}

		LayoutLinearWidgetContainer::LayoutLinearWidgetContainer(Info& info) : Layout(info.paintLayout, info.arrangement)
		{
			contained_widgets = info.widgets;

			if (contained_widgets.size() == 0)
				return;
			SkScalar packet_width = 1.0 / contained_widgets.size();

			if ((info.divisions.size() != 0) && (info.divisions.size() - 1 == contained_widgets.size())) {
				switch (arrangment) {
				case Arrangement::HORIZONTAL:
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(info.divisions[i], 0, info.divisions[i + 1], 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				case Arrangement::VERTICAL:
					for (int i = 0; i < info.divisions.size() - 1; ++i) {
						SkRect widget_rect = SkRect::MakeLTRB(0, info.divisions[i], 1, info.divisions[i + 1]);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				default:

					break;
				}
			}
			else {
				switch (arrangment) {
				case Arrangement::HORIZONTAL:
					for (SkScalar left_position = 0.0; left_position < 1.0; left_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(left_position, 0, left_position + packet_width, 1);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				case Arrangement::VERTICAL:
					for (SkScalar top_position = 0.0; top_position < 1.0; top_position += packet_width) {
						SkRect widget_rect = SkRect::MakeLTRB(0, top_position, 1, top_position + packet_width);
						rectangles_of_contained_layouts.push_back(widget_rect);
					}
					break;
				default:

					break;
				}
			}
		}

		void LayoutLinearWidgetContainer::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
		{
			SkRect rectangle = SkRect::MakeLTRB(drawing_area.fLeft * window_size.width(),
				drawing_area.fTop * window_size.height(),
				drawing_area.fRight * window_size.width(),
				drawing_area.fBottom * window_size.height());
			canvas_to_draw->drawRect(rectangle, paint_layout);
			for (int i = 0; i < rectangles_of_contained_layouts.size(); ++i) {
				SkRect rect = rectangles_of_contained_layouts[i];
				SkRect temp = rect.MakeXYWH(drawing_area.x() + rect.x() * drawing_area.width(),
					drawing_area.y() + rect.y() * drawing_area.height(),
					rect.width() * drawing_area.width(),
					rect.height() * drawing_area.height());

				SkRect temp2 = temp;
				temp2.fBottom = temp2.fBottom * window_size.height();
				temp2.fLeft = temp2.fLeft * window_size.width();
				temp2.fRight = temp2.fRight * window_size.width();
				temp2.fTop = temp2.fTop * window_size.height();
				//canvas->drawRect(temp2, paint_square);
				contained_widgets[i]->draw(canvas_to_draw, temp2);
			}
		}

		std::shared_ptr<LayoutLinearWidgetContainer> LayoutLinearWidgetContainer::make(Info& info)
		{
			return std::make_shared<LayoutLinearWidgetContainer>(info);
		}

		Widget::Widget()
		{
		}


		void Widget::draw(SkCanvas* canvas, SkRect& widget_rect)
		{}

		void Widget::callback(Signal signal, bool* interacted)
		{
		}

		bool Widget::interacts(SkScalar x, SkScalar y)
		{
			return ((widget_position.fRight >= x) &&
				(widget_position.fLeft <= x) &&
				(widget_position.fBottom >= y) &&
				(widget_position.fTop <= y));
		}

		void Widget::set_position(SkRect& rect)
		{
			widget_position = rect;
		}

		void Widget::get_position(SkRect& rect)
		{
			rect = widget_position;
		}


		Button::Button(Info& info)
		{
			hover_color = info.hover_color;
			waiting_color = info.waiting_color;
			click_color = info.click_color;
			paint = info.paintButton;
			paint_text = info.paintText;
			size = info.size;
			text_font = info.textFont;
			text_font.measureText(info.button_text.data(), info.button_text.size(), SkTextEncoding::kUTF8, &widget_rect_text);
			text = SkTextBlob::MakeFromString(info.button_text.c_str(), text_font);

			IconResources* resources = IconResources::Get();

			sk_sp<SkImage> image;
			resources->GetIcon(image, info.icon_identifier);
			icon_data = image;
		}

		void Button::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			switch (current_state) {
			case ButtonStates::WAITING:
				paint.setColor(waiting_color);
				break;
			case ButtonStates::HOVER:
				paint.setColor(hover_color);
				break;
			case ButtonStates::PRESSED:
				paint.setColor(click_color);
				break;
			}
			float text_offset_x = widget_rect.centerX() - widget_rect_text.width() / 2.0f;
			float text_offset_y = widget_rect.centerY() + widget_rect_text.height() / 2.0f;
			float box_offset_x = widget_rect.centerX() - size.width() / 2.0f;
			float box_offset_y = widget_rect.centerY() - size.height() / 2.0f;
			SkRect current_area = SkRect::MakeXYWH(box_offset_x, box_offset_y, std::min(widget_rect.width(), size.width()), std::min(widget_rect.height(), size.height()));
			set_position(current_area);
			canvas->drawRect(current_area, paint);
			canvas->drawTextBlob(text, text_offset_x, current_area.fBottom, paint_text);

			if (icon_data.get() != nullptr) {
				float image_width = icon_data->width();
				float image_height = icon_data->height();

				float current_selected_width = current_area.width();
				float current_selected_height = current_area.height() - widget_rect_text.height();

				float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);

				float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + current_area.x();
				float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + current_area.y();

				SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

				SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
				canvas->drawImageRect(icon_data, current_selected_image_rectangle, opt);
			}
		}

		std::shared_ptr<Button> Button::make(Info& info)
		{
			return std::make_shared<Button>(info);
		}

		void Button::callback(Signal signal, bool* interacted)
		{
			switch (signal.signal_type) {
			case Signal::Type::MOUSE_MOVE_SIGNAL:
				if (interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos))
				{
					*interacted = true;
					current_state = ButtonStates::HOVER;
				}

				else
					current_state = ButtonStates::WAITING;
				break;
			case Signal::Type::MOUSE_PRESS_SIGNAL:
				if (interacts(signal.data.press_signal.xpos, signal.data.press_signal.ypos)) {
					signal_click(signal.data.press_signal);
					*interacted = true;
					current_state = ButtonStates::PRESSED;
				}
				else
					current_state = ButtonStates::WAITING;
				break;
			case Signal::Type::MOUSE_UNPRESS_SIGNAL:
				current_state = ButtonStates::WAITING;
				break;
			case Signal::Type::ITEM_DROPED_SIGNAL:
				*interacted = true;
				signal_dropped(signal.data.dropped_signal);
				break;
			default:
				break;
			}
		}

		Context::Context()
		{
		}

		bool Context::initialize_context()
		{
			glfwInit();
			glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

			PFN_vkGetInstanceProcAddr getInstanceProc = (PFN_vkGetInstanceProcAddr)glfwGetInstanceProcAddress(instance, "vkGetInstanceProcAddr");
			PFN_vkGetDeviceProcAddr getDeviceProc = (PFN_vkGetDeviceProcAddr)glfwGetInstanceProcAddress(instance, "vkGetDeviceProcAddr");
			
			vulkan_pointer = [getInstanceProc, getDeviceProc](const char* proc_name,
				VkInstance instance, VkDevice device) {
					if (device != VK_NULL_HANDLE) {
						return getDeviceProc(device, proc_name);
					}
					return getInstanceProc(instance, proc_name);
			};

			uint32_t instanceVersion = VK_MAKE_VERSION(1, 0, 0);;
			// Provided by VK_VERSION_1_1
			VkResult res = vkEnumerateInstanceVersion(&instanceVersion);

			uint32_t apiVersion = VK_MAKE_VERSION(1, 0, 0);
			if (instanceVersion >= VK_MAKE_VERSION(1, 1, 0)) {
				// If the instance version is 1.0 we must have the apiVersion also be 1.0. However, if the
				// instance version is 1.1 or higher, we can set the apiVersion to be whatever the highest
				// api we may use in skia (technically it can be arbitrary). So for now we set it to 1.1
				// since that is the highest vulkan version.
				apiVersion = VK_MAKE_VERSION(1, 1, 0);
			}

			instanceVersion = std::min(instanceVersion, apiVersion);

			const VkApplicationInfo app_info = {
				VK_STRUCTURE_TYPE_APPLICATION_INFO, // sType
				nullptr,                            // pNext
				"Curan",                           // pApplicationName
				0,                                  // applicationVersion
				"Curan",                           // pEngineName
				0,                                  // engineVerison
				apiVersion,                         // apiVersion
			};

			std::vector<VkLayerProperties> instanceLayers;
			std::vector<VkExtensionProperties> instanceExtensions;

			if (!init_instance_extensions_and_layers(
				instanceExtensions,
				instanceLayers)) {
				return false;
			}

			std::vector<const char*> instanceLayerNames;
			std::vector<const char*> instanceExtensionNames;
			for (int i = 0; i < instanceLayers.size(); ++i) {
				instanceLayerNames.push_back(instanceLayers[i].layerName);
			}
			for (int i = 0; i < instanceExtensions.size(); ++i) {
				if (strncmp(instanceExtensions[i].extensionName, "VK_KHX", 6) != 0) {
					instanceExtensionNames.push_back(instanceExtensions[i].extensionName);
				}
			}

			const VkInstanceCreateInfo instance_create = {
			   VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,    // sType
			   nullptr,                                   // pNext
			   0,                                         // flags
			   &app_info,                                 // pApplicationInfo
			   (uint32_t)instanceLayerNames.size(),     // enabledLayerNameCount
			   instanceLayerNames.data(),                // ppEnabledLayerNames
			   (uint32_t)instanceExtensionNames.size(), // enabledExtensionNameCount
			   instanceExtensionNames.data(),            // ppEnabledExtensionNames
			};

			res = vkCreateInstance(&instance_create, nullptr, &instance);

			if (res < 0) {
				return false;
			}

			uint32_t deviceCount = 0;
			vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

			if (deviceCount == 0) {
				throw std::runtime_error("failed to find GPUs with Vulkan support!");
			}

			std::vector<VkPhysicalDevice> devices(deviceCount);
			vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

			for (const auto& device : devices) {

				uint32_t queueFamilyCount = 0;
				vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

				std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
				vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

				int i = 0;
				for (const auto& queueFamily : queueFamilies) {
					if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
						indices.graphicsFamily = i;
					}

					bool presentSupport = false;

					if (glfwGetPhysicalDevicePresentationSupport(instance, device, i) != GLFW_FALSE) {
						presentSupport = true;
					}

					if (presentSupport) {
						indices.presentFamily = i;
					}

					if (indices.isComplete()) {
						physicalDevice = device;
						break;
					}

					i++;
				}
			}

			if (physicalDevice == VK_NULL_HANDLE) {
				return false;
			}

			std::vector<VkLayerProperties> deviceLayers;
			std::vector<VkExtensionProperties> deviceExtensions;
			if (!init_device_extensions_and_layers(deviceLayers, deviceExtensions))
			{
				destroy_context();
				return false;
			}

			for (int i = 0; i < deviceLayers.size(); ++i) {
				std::string temporary_string{ deviceLayers[i].layerName };
				char* temporary_c_string = new char[temporary_string.size()];
				std::memcpy(temporary_c_string, temporary_string.data(), temporary_string.size());
				deviceLayerNames.push_back(temporary_c_string);
			}

			// We can't have both VK_KHR_buffer_device_address and VK_EXT_buffer_device_address as
			// extensions. So see if we have the KHR version and if so don't push back the EXT version in
			// the next loop.
			bool hasKHRBufferDeviceAddress = false;
			for (int i = 0; i < deviceExtensions.size(); ++i) {
				if (!strcmp(deviceExtensions[i].extensionName, "VK_KHR_buffer_device_address")) {
					hasKHRBufferDeviceAddress = true;
					break;
				}
			}

			for (int i = 0; i < deviceExtensions.size(); ++i) {
				// Don't use experimental extensions since they typically don't work with debug layers and
				// often are missing dependecy requirements for other extensions. Additionally, these are
				// often left behind in the driver even after they've been promoted to real extensions.
				if (0 != strncmp(deviceExtensions[i].extensionName, "VK_KHX", 6) &&
					0 != strncmp(deviceExtensions[i].extensionName, "VK_NVX", 6)) {

					// This is an nvidia extension that isn't supported by the debug layers so we get lots
					// of warnings. We don't actually use it, so it is easiest to just not enable it.
					if (0 == strcmp(deviceExtensions[i].extensionName, "VK_NV_low_latency")) {
						continue;
					}

					if (!hasKHRBufferDeviceAddress ||
						0 != strcmp(deviceExtensions[i].extensionName, "VK_EXT_buffer_device_address")) {
						std::string temporary_string{ deviceExtensions[i].extensionName };
						char* temporary_c_string = new char[temporary_string.size() + 1];
						std::memcpy(temporary_c_string, temporary_string.c_str(), temporary_string.size() + 1);
						deviceExtensionNames.push_back(temporary_c_string);
					}
				}
			}

			if (extensions == nullptr)
				extensions = new GrVkExtensions();

			extensions->init(vulkan_pointer, instance, physicalDevice,
				(uint32_t)instanceExtensionNames.size(),
				instanceExtensionNames.data(),
				(uint32_t)deviceExtensionNames.size(),
				deviceExtensionNames.data());
		}

		bool Context::init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers)
		{
			VkResult res;

			// instance extensions
			// via Vulkan implementation and implicitly enabled layers
			uint32_t extensionCount = 0;
			// Provided by VK_VERSION_1_0
			std::vector<VkExtensionProperties> extensions;
			res = vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

			if (VK_SUCCESS != res) {
				return false;
			}

			extensions.resize(extensionCount);
			res = vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data());

			if (VK_SUCCESS != res) {
				return false;
			}

			for (uint32_t i = 0; i < extensionCount; ++i) {
				instanceExtensions.push_back(extensions[i]);
			}

			extensions.clear();

			// via explicitly enabled layers
			uint32_t layerCount = instanceLayers.size();
			for (uint32_t layerIndex = 0; layerIndex < layerCount; ++layerIndex) {
				uint32_t extensionCount = 0;
				res = vkEnumerateInstanceExtensionProperties(instanceLayers[layerIndex].layerName, &extensionCount, nullptr);
				if (VK_SUCCESS != res) {
					return false;
				}
				extensions.resize(extensionCount);
				res = vkEnumerateInstanceExtensionProperties(instanceLayers[layerIndex].layerName, &extensionCount, extensions.data());
				if (VK_SUCCESS != res) {
					return false;
				}
				for (uint32_t i = 0; i < extensionCount; ++i) {
					instanceExtensions.push_back(extensions[i]);
				}
			}

			return true;
		}

		bool Context::init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions)
		{
			VkResult res;

			// device extensions
			// via Vulkan implementation and implicitly enabled layers
			uint32_t extensionCount = 0;
			// Provided by VK_VERSION_1_0
			std::vector<VkExtensionProperties> extensions;
			res = vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, nullptr);
			if (VK_SUCCESS != res) {
				return false;
			}
			extensions.resize(extensionCount);
			res = vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, extensions.data());
			if (VK_SUCCESS != res) {
				return false;
			}
			for (uint32_t i = 0; i < extensionCount; ++i) {
				deviceExtensions.push_back(extensions[i]);
			}

			extensions.clear();

			// via explicitly enabled layers
			uint32_t layerCount = deviceLayers.size();
			for (uint32_t layerIndex = 0; layerIndex < layerCount; ++layerIndex) {
				uint32_t extensionCount = 0;
				res = vkEnumerateDeviceExtensionProperties(physicalDevice, deviceLayers[layerIndex].layerName, &extensionCount, nullptr);
				if (VK_SUCCESS != res) {
					return false;
				}
				extensions.resize(extensionCount);
				res = vkEnumerateDeviceExtensionProperties(physicalDevice, deviceLayers[layerIndex].layerName, &extensionCount, extensions.data());
				if (VK_SUCCESS != res) {
					return false;
				}
				for (uint32_t i = 0; i < extensionCount; ++i) {
					deviceExtensions.push_back(extensions[i]);
				}
			}

			return true;
		}

		void Context::destroy_context()
		{
			if (extensions != nullptr)
				delete extensions;
			if (instance != VK_NULL_HANDLE)
				vkDestroyInstance(instance, nullptr);

			glfwTerminate();
		}

		Context::~Context()
		{
		}

		VKAPI_ATTR VkBool32 VKAPI_CALL Context::debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData)
		{
			std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

			return VK_FALSE;
		}

		void Context::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo)
		{
			createInfo = {};
			createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
			createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
			createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
			createInfo.pfnUserCallback = debugCallback;
		}

		ItemPreview::ItemPreview(Info& info)
		{
			color_background = info.color_background_left;
			color_hover = info.color_hover;
			color_selected = info.color_selected;
			color_waiting = info.color_waiting;
			font = info.font;
			text_paint = info.text_paint;

			paint_image_background.setStyle(SkPaint::kFill_Style);
			paint_image_background.setAntiAlias(true);
			paint_image_background.setStrokeWidth(4);
			paint_image_background.setColor(color_waiting);


			paint_background.setStyle(SkPaint::kFill_Style);
			paint_background.setAntiAlias(true);
			paint_background.setStrokeWidth(4);
			paint_background.setColor(color_background);
		}

		void ItemPreview::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			set_position(widget_rect);

			SkAutoCanvasRestore restore(canvas, true);
			canvas->clipRect(widget_rect);

			paint_background.setColor(color_background);
			canvas->drawRect(widget_rect, paint_background);

			auto iterator = item_list.begin();
			auto size = item_list.size();

			SkScalar width = widget_rect.width();
			uint32_t number_of_previews_per_line = (width - 2 * DK_VOLUME_PREVIEW_EXTREMA_SPACING_LINE) / DK_VOLUME_PREVIEW_WIDTH - 1;
			if (number_of_previews_per_line == 0)
				number_of_previews_per_line = 1;
			SkScalar spacing_between_icons_line = (width - 2 * DK_VOLUME_PREVIEW_EXTREMA_SPACING_LINE - (number_of_previews_per_line + 1) * DK_VOLUME_PREVIEW_WIDTH) / number_of_previews_per_line;
			uint32_t number_of_vertical_previews = size / number_of_previews_per_line
				+ (((size < 0) ^ (number_of_previews_per_line > 0)) && (size % number_of_previews_per_line));

			maximum_height = number_of_vertical_previews * (DK_VOLUME_PREVIEW_HEIGHT + DK_VOLUME_PREVIEW_EXTREMA_SPACING_COLUNM);

			SkScalar x = DK_VOLUME_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
			SkScalar y = DK_VOLUME_PREVIEW_EXTREMA_SPACING_COLUNM + widget_rect.y() + current_height_offset;

			while ((iterator != item_list.end()) && (y < widget_rect.fBottom))
			{
				Item item_to_draw = iterator->second;
				int index_of_item = iterator->first;
				preview_rectangle.setXYWH(x, y, DK_VOLUME_PREVIEW_WIDTH, DK_VOLUME_PREVIEW_HEIGHT);

				auto item_found = std::find(current_selected_identifiers.begin(), current_selected_identifiers.end(), index_of_item);

				if (item_found != current_selected_identifiers.end()){
					paint_image_background.setColor(color_hover);
				}	else {
					paint_image_background.setColor(color_waiting);
				}

				float image_width = item_to_draw.image->width();
				float image_height = item_to_draw.image->height();

				float preview_width = preview_rectangle.width();
				float preview_height = preview_rectangle.height();

				float scale_factor = std::min((preview_width * 0.95f) / image_width, DK_VOLUME_IMAGE_HEIGHT / image_height);

				float init_x = (preview_width - image_width * scale_factor) / 2.0f + preview_rectangle.x();
				float init_y = preview_rectangle.y() + (DK_VOLUME_PREVIEW_HEIGHT - DK_VOLUME_IMAGE_HEIGHT);

				SkRect image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, DK_VOLUME_IMAGE_HEIGHT * 0.98f);
				canvas->drawRoundRect(preview_rectangle, 5, 5, paint_image_background);

				SkRect bound_individual_name = item_to_draw.text->bounds();

				canvas->drawTextBlob(item_to_draw.text.get(), x + (DK_VOLUME_PREVIEW_WIDTH - bound_individual_name.width()) / 2.0f, y + bound_individual_name.height(),text_paint);

				SkSamplingOptions options;
				canvas->drawImageRect(item_to_draw.image, image_rectangle, options, nullptr);

				iterator++;

				x += DK_VOLUME_PREVIEW_WIDTH + spacing_between_icons_line;

				if (x + DK_VOLUME_PREVIEW_WIDTH >= widget_rect.fRight) {
					y += DK_VOLUME_PREVIEW_HEIGHT + DK_VOLUME_PREVIEW_EXTREMA_SPACING_COLUNM;
					x = DK_VOLUME_PREVIEW_EXTREMA_SPACING_LINE + widget_rect.x();
				}
			}
		}

		std::shared_ptr<ItemPreview> ItemPreview::make(Info& info)
		{
			return  std::make_shared<ItemPreview>(info);
		}

		void ItemPreview::callback(Signal signal, bool* interacted)
		{
			switch (signal.signal_type) {
			case Signal::Type::MOUSE_MOVE_SIGNAL:
				*interacted = true;
				current_mouse_position = signal.data.move_signal;
				break;
			case Signal::Type::MOUSE_PRESS_SIGNAL:
				if (interacts(signal.data.press_signal.xpos, signal.data.press_signal.ypos)){
					*interacted = true;
					
				}

				break;
			case Signal::Type::SCROLL_SIGNAL:
				if (interacts(signal.data.scroll_signal.xpos, signal.data.scroll_signal.ypos)) {
					*interacted = true;
					float increment = -signal.data.scroll_signal.yoffset * 5;
					SkRect current_area;
					get_position(current_area);
					if ((current_height_offset + increment + maximum_height > current_area.height()) && (current_height_offset + increment <= 0))
						current_height_offset = current_height_offset + increment;
				}
				break;
			default:

				break;
			}
		}


		ThreadPollStatusDisplay::ThreadPollStatusDisplay(Info& info)
		{
			color_background = info.color_background;
			color_text = info.color_text;
			font = info.font;

			background.setStyle(SkPaint::kFill_Style);
			background.setAntiAlias(true);
			background.setStrokeWidth(4);
			background.setColor(color_background);

			text_paint.setStyle(SkPaint::kFill_Style);
			text_paint.setAntiAlias(true);
			text_paint.setStrokeWidth(2);
			text_paint.setColor(color_text);
		}

		void ThreadPollStatusDisplay::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			canvas->drawRect(widget_rect, background);

			curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();

			int number_of_tasks_running = 0;
			int number_of_tasks_queue = 0;
			pool->GetNumberTasks(number_of_tasks_running, number_of_tasks_queue);

			std::string text = "Number of pending tasks: " + std::to_string(number_of_tasks_queue) + " running tasks: " + std::to_string(number_of_tasks_running);;

			SkRect bound_text;
			font.measureText(text.c_str(), text.size(), SkTextEncoding::kUTF8, &bound_text);

			canvas->drawSimpleText(text.c_str(),
				text.size(),
				SkTextEncoding::kUTF8,
				widget_rect.x(),
				widget_rect.y() + bound_text.height(),
				font,
				text_paint);
		}

		std::shared_ptr<ThreadPollStatusDisplay> ThreadPollStatusDisplay::make(Info& info)
		{
			return std::make_shared<ThreadPollStatusDisplay>(info);
		}

		TaskPreviewer::TaskPreviewer(Info& info)
		{
			click_color = info.click_color;
			font = info.font;
			hover_color = info.hover_color;
			relative_dimensions = info.relative_dimensions;
			waiting_color = info.waiting_color;
			text_color = info.text_color;

			background.setStyle(SkPaint::kFill_Style);
			background.setAntiAlias(true);
			background.setStrokeWidth(1);
			background.setColor(waiting_color);

			text_paint.setStyle(SkPaint::kFill_Style);
			text_paint.setAntiAlias(true);
			text_paint.setStrokeWidth(2);
			text_paint.setColor(text_color);

			IconResources* resources = IconResources::Get();

			for (auto task : info.supported_tasks)
			{
				InternalTaskData task_data;
				task_data.description = SkTextBlob::MakeFromString(task.description.c_str(), font);
				task_data.name = SkTextBlob::MakeFromString(task.name.c_str(), font);
				task_data.task_data = task;
				resources->GetIcon(task_data.icon, task.icon_identifier);
				list_of_tasks.push_back(task_data);
			}
		}

		void TaskPreviewer::callback(Signal signal, bool* interacted)
		{
			switch (signal.signal_type) {
			case Signal::Type::MOUSE_MOVE_SIGNAL:
			{
				*interacted = true;
				current_mouse_position = signal.data.move_signal;
			}
				
				break;
			case Signal::Type::MOUSE_PRESS_SIGNAL:
				if (check_interaction(signal.data.press_signal.xpos, signal.data.press_signal.ypos))
				{
					*interacted = true;
					last_pressed_position = signal.data.press_signal;
				}
					
				break;
			case Signal::Type::SCROLL_SIGNAL:
				if (check_interaction(signal.data.scroll_signal.xpos, signal.data.scroll_signal.ypos)) {

				}
				break;
			default:

				break;
			}
		}

		bool TaskPreviewer::check_interaction(double xpos, double ypos)
		{
			return ((widget_real_position.fRight >= xpos) &&
				(widget_real_position.fLeft <= xpos) &&
				(widget_real_position.fBottom >= ypos) &&
				(widget_real_position.fTop <= ypos));
		}

		void TaskPreviewer::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			background.setColor(waiting_color);

			widget_real_position.fBottom = relative_dimensions.fBottom * widget_rect.height();
			widget_real_position.fLeft = relative_dimensions.fLeft * widget_rect.width();
			widget_real_position.fRight = relative_dimensions.fRight * widget_rect.width();
			widget_real_position.fTop = relative_dimensions.fTop * widget_rect.height();

			SkScalar vertical_position{ vertical_offset + widget_real_position.fTop };
			canvas->drawRect(widget_real_position, background);
			for (int index = 0; index < list_of_tasks.size() && vertical_position < widget_real_position.fBottom; index++, vertical_position += DK_TASK_PREVIEW_HEIGHT) {
				SkRect image_name_rect = SkRect::MakeLTRB(widget_real_position.fLeft, vertical_position, widget_real_position.fLeft + DK_TASK_PREVIEW_HEIGHT, vertical_position + DK_TASK_PREVIEW_HEIGHT);
				SkRect description_rect = SkRect::MakeLTRB(image_name_rect.fLeft, vertical_position, widget_real_position.fRight, vertical_position + DK_TASK_PREVIEW_HEIGHT);

				if (image_name_rect.fLeft < last_pressed_position.xpos &&
					description_rect.fRight > last_pressed_position.xpos &&
					image_name_rect.fTop < last_pressed_position.ypos &&
					image_name_rect.fBottom > last_pressed_position.ypos)
				{
					background.setColor(click_color);
					background.setStyle(SkPaint::kFill_Style);
					current_index = index;
				}
				else if (image_name_rect.fLeft < current_mouse_position.xpos &&
					description_rect.fRight > current_mouse_position.xpos &&
					image_name_rect.fTop < current_mouse_position.ypos &&
					image_name_rect.fBottom > current_mouse_position.ypos) {
					background.setColor(hover_color);
					background.setStyle(SkPaint::kFill_Style);
				}
				else {
					background.setColor(waiting_color);
					background.setStyle(SkPaint::kFill_Style);
				}

				canvas->drawRect(image_name_rect, background);
				canvas->drawRect(description_rect, background);

				background.setColor(SK_ColorBLACK);
				background.setStyle(SkPaint::kStroke_Style);
				canvas->drawRect(image_name_rect, background);
				canvas->drawRect(description_rect, background);

				auto task = list_of_tasks[index];
				float image_width = task.icon->width();
				float image_height = task.icon->height();

				SkRect text_bound = task.name->bounds();

				float current_selected_width = image_name_rect.width();
				float current_selected_height = image_name_rect.height() - text_bound.height();

				float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);


				float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + image_name_rect.x();
				float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + image_name_rect.y();

				SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

				SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
				canvas->drawImageRect(list_of_tasks[index].icon, current_selected_image_rectangle, opt);
				canvas->drawTextBlob(task.name, init_x, image_name_rect.fBottom - text_bound.height(), text_paint);
			}
		}

		std::shared_ptr<TaskPreviewer> TaskPreviewer::make(Info& info)
		{
			return std::make_shared<TaskPreviewer>(info);
		}

		ImageDisplayDinamicLayout::ImageDisplayDinamicLayout(Info& info)
		{
			interaction = info.initial_interaction_mode;
			current_layout = info.layout;
			m_study = info.vol;
			optional_option_box = info.option_box;

			MovableImageDisplay::Info info_display_1x1_1;
			info_display_1x1_1.alpha_channel = info.alpha_channel;
			info_display_1x1_1.color_type = info.color_type;
			info_display_1x1_1.initial_image = info.vol.image;
			info_display_1x1_1.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_1x1_1 = MovableImageDisplay::make(info_display_1x1_1);

			LayoutVariableWidgetContainer::Info info_1x1;
			info_1x1.arrangement = Layout::Arrangement::VARIABLE;
			info_1x1.rectangles_of_contained_layouts = { SkRect::MakeXYWH(0,0,1,1) };
			info_1x1.widgets = { std::static_pointer_cast<Widget>(display_1x1_1) };

			Tile tile_1x1;
			tile_1x1.layout = std::static_pointer_cast<Layout>(LayoutVariableWidgetContainer::make(info_1x1));
			tile_1x1.contained_views = { display_1x1_1 };
			contained_tiles.push_back(tile_1x1);

			MovableImageDisplay::Info info_display_1x2_1;
			info_display_1x2_1.alpha_channel = info.alpha_channel;
			info_display_1x2_1.color_type = info.color_type;
			info_display_1x2_1.waiting_color = SK_ColorYELLOW;
			info_display_1x2_1.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_1x2_1 = MovableImageDisplay::make(info_display_1x2_1);

			MovableImageDisplay::Info info_display_1x2_2;
			info_display_1x2_2.alpha_channel = info.alpha_channel;
			info_display_1x2_2.color_type = info.color_type;
			info_display_1x2_1.waiting_color = SK_ColorYELLOW;
			info_display_1x2_2.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_1x2_2 = MovableImageDisplay::make(info_display_1x2_2);

			LayoutVariableWidgetContainer::Info info_1x2;
			info_1x2.arrangement = Layout::Arrangement::VARIABLE;
			info_1x2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 0.5, 1));
			info_1x2.widgets.push_back(std::static_pointer_cast<Widget>(display_1x2_1));
			info_1x2.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0.5, 0, 0.5, 1));
			info_1x2.widgets.push_back(std::static_pointer_cast<Widget>(display_1x2_2));

			Tile tile_1x2;
			tile_1x2.layout = std::static_pointer_cast<Layout>(LayoutVariableWidgetContainer::make(info_1x2));
			tile_1x2.contained_views.push_back(display_1x2_1);
			tile_1x2.contained_views.push_back(display_1x2_2);
			contained_tiles.push_back(tile_1x2);

			MovableImageDisplay::Info info_display_1x3_1;
			info_display_1x3_1.alpha_channel = info.alpha_channel;
			info_display_1x3_1.color_type = info.color_type;
			info_display_1x3_1.initial_image = info.vol.image;
			info_display_1x3_1.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_1x3_1 = MovableImageDisplay::make(info_display_1x3_1);

			MovableImageDisplay::Info info_display_1x3_2;
			info_display_1x3_2.alpha_channel = info.alpha_channel;
			info_display_1x3_2.color_type = info.color_type;
			info_display_1x3_2.initial_image = info.vol.image;
			info_display_1x3_2.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_1x3_2 = MovableImageDisplay::make(info_display_1x3_2);

			MovableImageDisplay::Info info_display_1x3_3;
			info_display_1x3_3.alpha_channel = info.alpha_channel;
			info_display_1x3_3.color_type = info.color_type;
			info_display_1x3_3.initial_image = info.vol.image;
			info_display_1x3_3.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_1x3_3 = MovableImageDisplay::make(info_display_1x3_3);

			LayoutVariableWidgetContainer::Info info_1x3;
			info_1x3.arrangement = Layout::Arrangement::VARIABLE;
			info_1x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1.0 / 3, 1));
			info_1x3.widgets.push_back(std::static_pointer_cast<Widget>(display_1x3_1));
			info_1x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(1.0 / 3, 0, 1.0 / 3, 1));
			info_1x3.widgets.push_back(std::static_pointer_cast<Widget>(display_1x3_2));
			info_1x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(2.0 / 3, 0, 1.0 / 3, 1));
			info_1x3.widgets.push_back(std::static_pointer_cast<Widget>(display_1x3_3));

			Tile tile_1x3;
			tile_1x3.layout = std::static_pointer_cast<Layout>(LayoutVariableWidgetContainer::make(info_1x3));
			tile_1x3.contained_views.push_back(display_1x3_1);
			tile_1x3.contained_views.push_back(display_1x3_2);
			tile_1x3.contained_views.push_back(display_1x3_3);
			contained_tiles.push_back(tile_1x3);

			MovableImageDisplay::Info info_display_2x3_1;
			info_display_2x3_1.alpha_channel = info.alpha_channel;
			info_display_2x3_1.color_type = info.color_type;
			info_display_2x3_1.initial_image = info.vol.image;
			info_display_2x3_1.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_2x3_1 = MovableImageDisplay::make(info_display_2x3_1);

			MovableImageDisplay::Info info_display_2x3_2;
			info_display_2x3_2.alpha_channel = info.alpha_channel;
			info_display_2x3_2.color_type = info.color_type;
			info_display_2x3_2.initial_image = info.vol.image;
			info_display_2x3_2.waiting_color = SK_ColorYELLOW;
			std::shared_ptr<MovableImageDisplay> display_2x3_2 = MovableImageDisplay::make(info_display_2x3_2);

			MovableImageDisplay::Info info_display_2x3_3;
			info_display_2x3_3.alpha_channel = info.alpha_channel;
			info_display_2x3_3.color_type = info.color_type;
			info_display_2x3_3.waiting_color = SK_ColorYELLOW;
			info_display_2x3_3.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_2x3_3 = MovableImageDisplay::make(info_display_2x3_3);

			MovableImageDisplay::Info info_display_2x3_4;
			info_display_2x3_4.alpha_channel = info.alpha_channel;
			info_display_2x3_4.color_type = info.color_type;
			info_display_2x3_4.waiting_color = SK_ColorYELLOW;
			info_display_2x3_4.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_2x3_4 = MovableImageDisplay::make(info_display_2x3_4);

			MovableImageDisplay::Info info_display_2x3_5;
			info_display_2x3_5.alpha_channel = info.alpha_channel;
			info_display_2x3_5.color_type = info.color_type;
			info_display_2x3_5.waiting_color = SK_ColorYELLOW;
			info_display_2x3_5.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_2x3_5 = MovableImageDisplay::make(info_display_2x3_5);

			MovableImageDisplay::Info info_display_2x3_6;
			info_display_2x3_6.alpha_channel = info.alpha_channel;
			info_display_2x3_6.color_type = info.color_type;
			info_display_2x3_6.waiting_color = SK_ColorYELLOW;
			info_display_2x3_6.initial_image = info.vol.image;
			std::shared_ptr<MovableImageDisplay> display_2x3_6 = MovableImageDisplay::make(info_display_2x3_6);

			LayoutVariableWidgetContainer::Info info_2x3;
			info_2x3.arrangement = Layout::Arrangement::VARIABLE;
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_1));
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(1.0 / 3, 0, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_2));
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(2.0 / 3, 0, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_3));
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(0, 0.5, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_4));
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(1.0 / 3, 0.5, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_5));
			info_2x3.rectangles_of_contained_layouts.push_back(SkRect::MakeXYWH(2.0 / 3, 0.5, 1.0 / 3, 0.5));
			info_2x3.widgets.push_back(std::static_pointer_cast<Widget>(display_2x3_6));

			Tile tile_2x3;
			tile_2x3.layout = std::static_pointer_cast<Layout>(LayoutVariableWidgetContainer::make(info_2x3));
			tile_2x3.contained_views.push_back(display_2x3_1);
			tile_2x3.contained_views.push_back(display_2x3_2);
			tile_2x3.contained_views.push_back(display_2x3_3);
			tile_2x3.contained_views.push_back(display_2x3_4);
			tile_2x3.contained_views.push_back(display_2x3_5);
			tile_2x3.contained_views.push_back(display_2x3_6);
			contained_tiles.push_back(tile_2x3);
		}

		void ImageDisplayDinamicLayout::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			set_position(widget_rect);
			Tile tile = contained_tiles[current_layout];
			SkRect rect = SkRect::MakeWH(canvas->getSurface()->width(), canvas->getSurface()->height());
			SkRect normalized = SkRect::MakeLTRB(widget_rect.fLeft / rect.width(), widget_rect.fTop / rect.height(), widget_rect.fRight / rect.width(), widget_rect.fBottom / rect.height());
			tile.layout->draw(canvas, normalized, rect);
			if(show_optional_box && optional_option_box != nullptr)
				optional_option_box->draw(canvas,widget_rect);
		}

		std::shared_ptr<ImageDisplayDinamicLayout> ImageDisplayDinamicLayout::make(Info& info)
		{
			return std::make_shared<ImageDisplayDinamicLayout>(info);
		}

		void ImageDisplayDinamicLayout::callback(Signal signal, bool* interacted)
		{
			if (show_optional_box && optional_option_box != nullptr)
			{
				optional_option_box->callback(signal, interacted);
				if (*interacted)
					return;
			}

			Tile tile = contained_tiles[current_layout];
			switch (interaction) {
			case InteractionMode::CLICK:
			{
				// the click mode would be used for selecting points 
				//in the medical images, for now we have no specific 
				//use for this mode of interaction
				static bool click_state = false;
				switch (signal.signal_type) {
				case Signal::Type::MOUSE_MOVE_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							*interacted = true;
							v->hover(true);
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					current_mouse_position = signal.data.move_signal;
					break;
				case Signal::Type::MOUSE_PRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							*interacted = true;
							v->hover(true);
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					last_pressed_position = signal.data.press_signal;
					break;
				case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							*interacted = true;
							v->hover(true);
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					break;
				default:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							*interacted = true;
							v->hover(true);
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					break;
				}
			}
			break;

			case InteractionMode::PAN:
			{
				// the pan mode is used for to move the image around. 
				// I was thinking about recording the data of the previous
				// point, calculate the difference and modify the image 
				// translation vector
				static bool click_state = false;
				switch (signal.signal_type) {
				case Signal::Type::MOUSE_MOVE_SIGNAL:
					for (auto v : tile.contained_views)
					{
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos))
						{
							*interacted = true;
							v->hover(true);
							if (click_state) {
								double delta_x = signal.data.move_signal.xpos - current_mouse_position.xpos;
								double delta_y = signal.data.move_signal.ypos - current_mouse_position.ypos;
								v->translate(delta_x, delta_y);
							}
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					current_mouse_position = signal.data.move_signal;
					break;
				case Signal::Type::MOUSE_PRESS_SIGNAL:
				{
					click_state = true;
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							v->callback(signal, interacted);
							*interacted = true;
						}
						else
							v->hover(false);
					}
					last_pressed_position = signal.data.press_signal;
				}
				break;
				case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							*interacted = true;
							v->hover(true);
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					click_state = false;
					break;
				default:

					break;
				}
			}
			break;
			case InteractionMode::ROTATE:
			{
				// the rotate mode is used for to rotate the image around. 
				// to achieve this behaviour 
				static bool click_state = false;
				switch (signal.signal_type) {
				case Signal::Type::MOUSE_MOVE_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							if (click_state)
							{
								static Eigen::Vector3d vec_prev = { current_mouse_position.xpos - last_pressed_position.xpos,current_mouse_position.ypos - last_pressed_position.ypos, 0.0};
								Eigen::Vector3d vec_current = { signal.data.move_signal.xpos - last_pressed_position.xpos ,signal.data.move_signal.ypos - last_pressed_position.ypos, 0.0 };
								double angle = 0.0;
								double norm = vec_prev.dot(vec_current);
								if (norm != 0.0)
									angle = std::atan2(vec_prev.cross(vec_current)[2], norm);
								v->rotate(angle * (180 / 3.141562), last_pressed_position.xpos, last_pressed_position.ypos);
								vec_prev = vec_current;
							}
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					current_mouse_position = signal.data.move_signal;
					break;
				case Signal::Type::MOUSE_PRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					click_state = true;
					last_pressed_position = signal.data.press_signal;
					break;
				case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					click_state = false;
					break;
				default:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					break;
				}
			}
			break;
			case InteractionMode::ZOOM:
			{
				static bool click_state = false;
				switch (signal.signal_type) {
				case Signal::Type::MOUSE_MOVE_SIGNAL:
					for (auto v : tile.contained_views)
					{
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos))
						{
							*interacted = true;
							v->hover(true);
							if (click_state)
							{
								MouseMoveSignal current_delta_vector{ signal.data.move_signal.xpos - last_pressed_position.xpos ,signal.data.move_signal.ypos - last_pressed_position.ypos };
								double zoom_factor{ std::exp(-(current_delta_vector.ypos) * 0.005) };
								v->zoom(zoom_factor, last_pressed_position.xpos, last_pressed_position.ypos);
							}
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					current_mouse_position = signal.data.move_signal;
					break;
				case Signal::Type::MOUSE_PRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					click_state = true;
					last_pressed_position = signal.data.press_signal;
					break;
				case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					click_state = false;
					break;
				default:
					for (auto v : tile.contained_views) {
						if (v->interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos)) {
							v->hover(true);
							*interacted = true;
							v->callback(signal, interacted);
						}
						else
							v->hover(false);
					}
					break;
				}
			}
			break;
			default:

				break;
			}
		}

		void ImageDisplayDinamicLayout::pan_mode(MousePressSignal)
		{
			interaction = InteractionMode::PAN;
		}

		void ImageDisplayDinamicLayout::rotate_mode(MousePressSignal)
		{
			interaction = InteractionMode::ROTATE;
		}

		void ImageDisplayDinamicLayout::click_mode(MousePressSignal)
		{
			interaction = InteractionMode::CLICK;
		}

		void ImageDisplayDinamicLayout::zoom_mode(MousePressSignal)
		{
			interaction = InteractionMode::ZOOM;
		}

		void ImageDisplayDinamicLayout::change_option_box_presentation_state(MousePressSignal signal)
		{
			show_optional_box.store(!show_optional_box);
			display_paths(signal);
			interaction = InteractionMode::CLICK;
		}
		
		void ImageDisplayDinamicLayout::display_paths(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->display_paths();
		};

		void ImageDisplayDinamicLayout::show_all_paths(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->show_all_paths();
		};

		void ImageDisplayDinamicLayout::hide_path(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->hide_path();
		};

		void ImageDisplayDinamicLayout::change_path(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->change_path();
		};

		void ImageDisplayDinamicLayout::submit_path(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->submit_path();
		};

		void ImageDisplayDinamicLayout::eliminate_path(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->eliminate_path();
		};

		void ImageDisplayDinamicLayout::new_path(MousePressSignal)
		{
			Tile tile = contained_tiles[current_layout];
			for (auto views : tile.contained_views)
				views->new_path();
		};

		void ImageDisplayDinamicLayout::change_layout(LayoutState new_layout)
		{
			current_layout = new_layout;
		}

		LayoutVariableContainer::LayoutVariableContainer(Info& info) : Layout(info.paint_layout, info.arrangement)
		{
			contained_layouts = info.layouts;
			rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
		}

		void LayoutVariableContainer::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
		{
			SkRect rectangle = SkRect::MakeLTRB(drawing_area.fLeft * window_size.width(),
				drawing_area.fTop * window_size.height(),
				drawing_area.fRight * window_size.width(),
				drawing_area.fBottom * window_size.height());
			canvas_to_draw->drawRect(rectangle, paint_layout);
			for (int i = 0; i < rectangles_of_contained_layouts.size(); ++i) {
				SkRect rect = rectangles_of_contained_layouts[i];
				SkRect temp = rect.MakeXYWH(drawing_area.x() + rect.x() * drawing_area.width(),
					drawing_area.y() + rect.y() * drawing_area.height(),
					rect.width() * drawing_area.width(),
					rect.height() * drawing_area.height());

				SkRect temp2 = temp;
				temp2.fBottom = temp2.fBottom * window_size.height();
				temp2.fLeft = temp2.fLeft * window_size.width();
				temp2.fRight = temp2.fRight * window_size.width();
				temp2.fTop = temp2.fTop * window_size.height();
				contained_layouts[i]->draw(canvas_to_draw, temp, window_size);
			}
		}

		std::shared_ptr<LayoutVariableContainer> LayoutVariableContainer::make(Info& info)
		{
			return std::make_shared<LayoutVariableContainer>(info);
		}

		LayoutVariableWidgetContainer::LayoutVariableWidgetContainer(Info& info) : Layout(info.paintLayout, info.arrangement)
		{
			contained_widgets = info.widgets;
			rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
		}

		void LayoutVariableWidgetContainer::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
		{
			SkRect rectangle = SkRect::MakeLTRB(drawing_area.fLeft * window_size.width(),
				drawing_area.fTop * window_size.height(),
				drawing_area.fRight * window_size.width(),
				drawing_area.fBottom * window_size.height());
			canvas_to_draw->drawRect(rectangle, paint_layout);
			for (int i = 0; i < rectangles_of_contained_layouts.size(); ++i) {
				SkRect rect = rectangles_of_contained_layouts[i];
				SkRect temp = rect.MakeXYWH(drawing_area.x() + rect.x() * drawing_area.width(),
					drawing_area.y() + rect.y() * drawing_area.height(),
					rect.width() * drawing_area.width(),
					rect.height() * drawing_area.height());

				SkRect temp2 = temp;
				temp2.fBottom = temp2.fBottom * window_size.height();
				temp2.fLeft = temp2.fLeft * window_size.width();
				temp2.fRight = temp2.fRight * window_size.width();
				temp2.fTop = temp2.fTop * window_size.height();
				contained_widgets[i]->draw(canvas_to_draw, temp2);
			}
		}

		std::shared_ptr<LayoutVariableWidgetContainer> LayoutVariableWidgetContainer::make(Info& info)
		{
			return std::make_shared<LayoutVariableWidgetContainer>(info);
		}

		ThreeWayButton::ThreeWayButton(Info& info)
		{
			total_size = info.size;
			hover_color_side = info.hover_color_side;
			waiting_color_side = info.waiting_color_side;
			click_color_side = info.click_color_side;
			color_center = info.color_center;
			text_font = info.font;

			left_button_size = SkRect::MakeWH(total_size.width() * (1.0 / 3), total_size.height() * 0.75);
			center_button_size = SkRect::MakeWH(total_size.width() * (1.0 / 3), total_size.height());
			right_button_size = SkRect::MakeWH(total_size.width() * (1.0 / 3), total_size.height() * 0.75);

			SkColor coltext = { SK_ColorBLACK };

			paint_text.setStyle(SkPaint::kFill_Style);
			paint_text.setAntiAlias(true);
			paint_text.setStrokeWidth(4);
			paint_text.setColor(coltext);

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			text_font = SkFont(typeface, DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			for (auto s : info.stages_array) {
				DrawableButtonData data;
				data.area = SkRect::MakeWH(1, 1);
				text_font.measureText(s.data(), s.size(), SkTextEncoding::kUTF8, &data.widget_rect_text);
				data.text = SkTextBlob::MakeFromString(s.c_str(), text_font);
				data.paint_background.setStyle(SkPaint::kFill_Style);
				data.paint_background.setAntiAlias(true);
				data.paint_background.setStrokeWidth(4);
				button_vector_stuf.push_back(data);
			}

			DrawableButtonData data;
			data.area = SkRect::MakeWH(1, 1);
			text_font.measureText("", 0, SkTextEncoding::kUTF8, &data.widget_rect_text);
			data.paint_background.setStyle(SkPaint::kFill_Style);
			data.paint_background.setAntiAlias(true);
			data.paint_background.setStrokeWidth(4);
			data.text = SkTextBlob::MakeFromString("", text_font);
			button_vector_stuf.push_back(data);

			drawable_stuf[1].paint_background.setColor(color_center);

			drawable_stuf[0] = button_vector_stuf[button_vector_stuf.size() - 1];
			drawable_stuf[0].paint_background.setColor(waiting_color_side);
			drawable_stuf[1] = button_vector_stuf[index];
			drawable_stuf[1].paint_background.setColor(color_center);
			drawable_stuf[2] = button_vector_stuf[(long)index + 1];
			drawable_stuf[2].paint_background.setColor(waiting_color_side);
		}

		void ThreeWayButton::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			std::lock_guard<std::mutex> lk(mut);

			drawable_stuf[0].area = SkRect::MakeXYWH(widget_rect.centerX() - total_size.width() / 2, widget_rect.centerY() - left_button_size.height() / 2, left_button_size.width(), left_button_size.height());
			drawable_stuf[1].area = SkRect::MakeXYWH(drawable_stuf[0].area.fRight, widget_rect.centerY() - center_button_size.height() / 2, center_button_size.width(), center_button_size.height());
			drawable_stuf[2].area = SkRect::MakeXYWH(drawable_stuf[1].area.fRight, widget_rect.centerY() - right_button_size.height() / 2, right_button_size.width(), right_button_size.height());

			canvas->drawRoundRect(drawable_stuf[0].area, 10, 10, drawable_stuf[0].paint_background);
			float text_offset_x = drawable_stuf[0].area.centerX() - drawable_stuf[0].widget_rect_text.width() / 2.0f;
			float text_offset_y = drawable_stuf[0].area.centerY() + drawable_stuf[0].widget_rect_text.height() / 2.0f;
			canvas->drawTextBlob(drawable_stuf[0].text, text_offset_x, text_offset_y, paint_text);

			canvas->drawRoundRect(drawable_stuf[1].area, 10, 10, drawable_stuf[1].paint_background);
			text_offset_x = drawable_stuf[1].area.centerX() - drawable_stuf[1].widget_rect_text.width() / 2.0f;
			text_offset_y = drawable_stuf[1].area.centerY() + drawable_stuf[1].widget_rect_text.height() / 2.0f;
			canvas->drawTextBlob(drawable_stuf[1].text, text_offset_x, text_offset_y, paint_text);

			canvas->drawRoundRect(drawable_stuf[2].area, 10, 10, drawable_stuf[2].paint_background);
			text_offset_x = drawable_stuf[2].area.centerX() - drawable_stuf[2].widget_rect_text.width() / 2.0f;
			text_offset_y = drawable_stuf[2].area.centerY() + drawable_stuf[2].widget_rect_text.height() / 2.0f;
			canvas->drawTextBlob(drawable_stuf[2].text, text_offset_x, text_offset_y, paint_text);
		}

		std::shared_ptr<ThreeWayButton> ThreeWayButton::make(Info& info)
		{
			return std::make_shared<ThreeWayButton>(info);
		}

		void ThreeWayButton::callback(Signal signal, bool* interacted)
		{
			switch (signal.signal_type)
			{
			case Signal::Type::MOUSE_MOVE_SIGNAL:
				if (area_interacts(drawable_stuf[0].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos))
				{
					drawable_stuf[0].paint_background.setColor(hover_color_side);
					*interacted = true;
				}
				else
					drawable_stuf[0].paint_background.setColor(waiting_color_side);
				if (area_interacts(drawable_stuf[2].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos))
				{
					*interacted = true;
					drawable_stuf[2].paint_background.setColor(hover_color_side);
				}
				else
					drawable_stuf[2].paint_background.setColor(waiting_color_side);
				break;
			case Signal::Type::MOUSE_PRESS_SIGNAL:
				if (area_interacts(drawable_stuf[0].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos)) {
					drawable_stuf[0].paint_background.setColor(click_color_side);
					if (index != 0)
						signal_click(signal.data.press_signal, ButtonPressed::LEFT);
					index = (index - 1 < 0) ? 0 : index - 1;
					if (index == 0) {
						std::lock_guard<std::mutex> lk(mut);
						drawable_stuf[0] = button_vector_stuf[button_vector_stuf.size() - 1];
						drawable_stuf[0].paint_background.setColor(waiting_color_side);
						drawable_stuf[1] = button_vector_stuf[index];
						drawable_stuf[1].paint_background.setColor(color_center);
						drawable_stuf[2] = button_vector_stuf[(long)index + 1];
						drawable_stuf[2].paint_background.setColor(waiting_color_side);
					}
					else
					{
						std::lock_guard<std::mutex> lk(mut);
						drawable_stuf[0] = button_vector_stuf[index - 1l];
						drawable_stuf[1] = button_vector_stuf[index];
						drawable_stuf[1].paint_background.setColor(color_center);
						drawable_stuf[2] = button_vector_stuf[(long)index + 1];
						drawable_stuf[2].paint_background.setColor(waiting_color_side);
					}
					*interacted = true;
				}
				if (area_interacts(drawable_stuf[2].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos)) {
					drawable_stuf[2].paint_background.setColor(click_color_side);
					if (index != button_vector_stuf.size() - 2)
						signal_click(signal.data.press_signal, ButtonPressed::RIGHT);
					index = ((long)index + 1 > button_vector_stuf.size() - 2) ? button_vector_stuf.size() - 2 : (long)index + 1;
					if (index == button_vector_stuf.size() - 2)
					{
						std::lock_guard<std::mutex> lk(mut);
						drawable_stuf[0] = button_vector_stuf[(long)index - 1];
						drawable_stuf[0].paint_background.setColor(waiting_color_side);
						drawable_stuf[1] = button_vector_stuf[index];
						drawable_stuf[1].paint_background.setColor(color_center);
						drawable_stuf[2] = button_vector_stuf[button_vector_stuf.size() - 1];
						drawable_stuf[2].paint_background.setColor(waiting_color_side);
					}
					else
					{
						std::lock_guard<std::mutex> lk(mut);
						drawable_stuf[0] = button_vector_stuf[(long)index - 1];
						drawable_stuf[0].paint_background.setColor(waiting_color_side);
						drawable_stuf[1] = button_vector_stuf[index];
						drawable_stuf[1].paint_background.setColor(color_center);
						drawable_stuf[2] = button_vector_stuf[(long)index + 1];
						drawable_stuf[2].paint_background.setColor(waiting_color_side);

					}
					*interacted = true;
				}
				break;
			case Signal::Type::MOUSE_UNPRESS_SIGNAL:
				if (area_interacts(drawable_stuf[0].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos))
					drawable_stuf[0].paint_background.setColor(waiting_color_side);
				if (area_interacts(drawable_stuf[2].area, signal.data.press_signal.xpos, signal.data.press_signal.ypos))
					drawable_stuf[2].paint_background.setColor(waiting_color_side);
				break;
			default:

				break;
			}
		}

		bool ThreeWayButton::area_interacts(SkRect area, SkScalar x_pos, SkScalar y_pos)
		{
			return ((area.fRight >= x_pos) &&
				(area.fLeft <= x_pos) &&
				(area.fBottom >= y_pos) &&
				(area.fTop <= y_pos));
		}

		ManipulatorPosition::ManipulatorPosition(Info& info)
		{
			color_background = info.color_background;
			color_text = info.color_text;
			font = info.font;
			color_ranges = info.color_ranges;

			if (info.joint_names.size() != info.ranges.size())
				return;

			int number_of_partitions = info.joint_names.size();
			SkScalar normalized_height = 1.0 / number_of_partitions;
			range_text.resize(info.joint_names.size());
			rectangle.resize(info.joint_names.size());
			SkScalar top_height = 0.0f;
			for (int index = 0; index < info.joint_names.size(); ++index) {
				range_text[index].joint_name = SkTextBlob::MakeFromString(info.joint_names[index].c_str(), font);
				font.measureText(info.joint_names[index].c_str(), info.joint_names[index].size(), SkTextEncoding::kUTF8, &range_text[index].center_joint_size);
				range_text[index].left_range = SkTextBlob::MakeFromString(std::to_string(std::get<0>(info.ranges[index])).c_str(), font);
				font.measureText(std::to_string(std::get<0>(info.ranges[index])).c_str(), std::to_string(std::get<0>(info.ranges[index])).size(), SkTextEncoding::kUTF8, &range_text[index].left_range_size);
				range_text[index].right_range = SkTextBlob::MakeFromString(std::to_string(std::get<1>(info.ranges[index])).c_str(), font);
				font.measureText(std::to_string(std::get<1>(info.ranges[index])).c_str(), std::to_string(std::get<1>(info.ranges[index])).size(), SkTextEncoding::kUTF8, &range_text[index].right_range_size);
				rectangle[index] = SkRect::MakeLTRB(0, top_height, 1, top_height + normalized_height);
				top_height += normalized_height;
			}
		}

		void ManipulatorPosition::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			background.setColor(color_background);
			canvas->drawRect(widget_rect, background);
			for (int index = 0; index < rectangle.size(); ++index)
			{
				SkRect re = rectangle[index];
				SkRect range_drawable_area = SkRect::MakeXYWH(re.fLeft * widget_rect.width() + widget_rect.x(),
					re.fTop * widget_rect.height() + widget_rect.y(),
					re.width() * widget_rect.width(),
					re.height() * widget_rect.height());
				SkRect range_area = SkRect::MakeXYWH(range_drawable_area.fLeft,
					(range_drawable_area.height() - DK_MANIPULATOR_BAR_HEIGHT) / 2.0 + range_drawable_area.y(),
					range_drawable_area.width(),
					DK_MANIPULATOR_BAR_HEIGHT);

				SkRect range_area_left = SkRect::MakeXYWH(range_area.x(), range_area.y(), range_area.width() / 2, range_area.height());
				SkRect range_area_right = SkRect::MakeXYWH(range_area_left.fRight, range_area.y(), range_area.width() / 2, range_area.height());

				SkPoint points_left[2] = { SkPoint::Make(range_area_left.fLeft, range_area_left.centerY()), SkPoint::Make(range_area_left.fRight, range_area_left.centerY()) };
				SkPoint points_right[2] = { SkPoint::Make(range_area_right.fLeft, range_area_right.centerY()), SkPoint::Make(range_area_right.fRight, range_area_right.centerY()) };
				SkColor colors_left[2] = { SK_ColorRED, SK_ColorGREEN };
				SkColor colors_right[2] = { SK_ColorGREEN ,SK_ColorRED };

				SkPaint paint;
				paint.setShader(SkGradientShader::MakeLinear(points_left, colors_left, nullptr, 2, SkTileMode::kClamp,
					0, nullptr));

				canvas->drawRect(range_area_left, paint);

				paint.setShader(SkGradientShader::MakeLinear(points_right, colors_right, nullptr, 2, SkTileMode::kClamp,
					0, nullptr));

				canvas->drawRect(range_area_right, paint);

				SkScalar xoffset = range_drawable_area.centerX() - range_text[index].center_joint_size.width() / 2.0;
				SkScalar yoffset = range_drawable_area.fTop + range_text[index].center_joint_size.height();
				canvas->drawTextBlob(range_text[index].joint_name, xoffset, yoffset, text_paint);
				xoffset = range_area.fLeft;
				yoffset = range_area.centerY();
				canvas->drawTextBlob(range_text[index].left_range, xoffset, yoffset, text_paint);
				xoffset = range_area.fRight - range_text[index].right_range_size.width();
				yoffset = range_area.centerY();
				canvas->drawTextBlob(range_text[index].right_range, xoffset, yoffset, text_paint);
			}
		}

		std::shared_ptr<ManipulatorPosition> ManipulatorPosition::make(Info& info)
		{
			return std::make_shared<ManipulatorPosition>(info);
		}

		void ManipulatorPosition::callback(Signal signal, bool* interacted)
		{
		}

		SystemValidation::SystemValidation(Info& info)
		{
			const SkScalar intervals[] = { 10.0f, 5.0f, 2.0f, 5.0f };
			size_t count = sizeof(intervals) / sizeof(intervals[0]);

			path_validation_colors = info.path_validation_colors;

			path_paint.setPathEffect(
				SkPathEffect::MakeCompose(SkDashPathEffect::Make(intervals, count, 0.0f),
					SkDiscretePathEffect::Make(10.0f, 4.0f)));
			path_paint.setStyle(SkPaint::kStroke_Style);
			path_paint.setStrokeWidth(5.0f);
			path_paint.setAntiAlias(true);
			path_paint.setColor(std::get<1>(path_validation_colors));

			for (auto img : info.icons_of_devices) {
				StaticImageDisplay::Info disp_info;
				disp_info.alpha_channel = kOpaque_SkAlphaType;
				disp_info.color_type = kGray_8_SkColorType;
				disp_info.initial_image = img;
				disp_info.waiting_color = SkColorSetARGB(0, 0, 0, 0);
				std::shared_ptr<StaticImageDisplay> display = StaticImageDisplay::make(disp_info);
				data.push_back(display);
			}
		}

		void SystemValidation::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			int size = data.size();
			SkScalar box_width = widget_rect.width() / (size - 1);
			SkScalar box_height = widget_rect.height() / 2;

			SkScalar box_real_width = box_width * 0.5;
			SkScalar box_real_height = box_height * 0.5;

			SkScalar top = widget_rect.y();
			SkScalar left = widget_rect.x();

			SkRect main_icon = SkRect::MakeXYWH(widget_rect.centerX() - box_real_width / 2, top + (box_height - box_real_height) / 2, box_real_width, box_real_height);
			data[0]->draw(canvas, main_icon);

			for (int index = 1; index < data.size(); ++index)
			{
				SkRect area_of_box_display = SkRect::MakeXYWH(left + (box_width - box_real_width) / 2, top + box_height + (box_height - box_real_height) / 2, box_real_width, box_real_height);
				data[index]->draw(canvas, area_of_box_display);
				left += box_width;
			}

			static double previous_size = 0.0;
			if (previous_size != widget_rect.height() * 0.2)
			{
				previous_size = widget_rect.height() * 0.2;
				buffering.resize(previous_size);
			}

			switch (current_status)
			{
			case curan::communication::ProcessorOpenIGTLink::Status::CK_CLOSED:
				buffering.paint.setColor(SK_ColorRED);
				break;
			case curan::communication::ProcessorOpenIGTLink::Status::CK_OPEN:
				buffering.paint.setColor(SK_ColorGREEN);
				break;
			default:

				break;
			}

			buffering.draw(canvas, { widget_rect.centerX(),widget_rect.centerY() });

		}

		std::shared_ptr<SystemValidation> SystemValidation::make(Info& info)
		{
			return std::make_shared<SystemValidation>(info);
		}

		void SystemValidation::callback(Signal signal, bool* interacted)
		{
		}

		void SystemValidation::update_connection_status(curan::communication::ProcessorOpenIGTLink::Status in_status)
		{
			current_status = in_status;
		}

		RadioButton::RadioButton(Info& info)
		{
			color = info.color;
			text_font = info.text_font;
			size = info.size;
			background_color = info.background_color;
			is_exclusive = info.is_exclusive;

			paint.setStyle(SkPaint::kStroke_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(1);
			paint.setColor(color);

			paint_text.setStyle(SkPaint::kFill_Style);
			paint_text.setAntiAlias(true);
			paint_text.setStrokeWidth(0.5);
			paint_text.setColor(color);

			switch (info.layout){
			case RadioButtonLayout::HORIZONTAL:
			{
				double normalized_dimensions = 1.0 / info.options.size();
				double left_coordinate = 0.0;
				for (int index = 0; index < info.options.size(); ++index) {
					RadioItem item;
					item.normalized_position = SkRect::MakeXYWH(left_coordinate, 0, normalized_dimensions, 1);
					item.real_item_position = SkRect::MakeXYWH(0, 0, 0.01, 0.01);
					item.text = SkTextBlob::MakeFromString(info.options[index].c_str(), text_font);;
					text_font.measureText(info.options[index].data(), info.options[index].size(), SkTextEncoding::kUTF8, &item.text_size);;
					radio_components.push_back(item);
					left_coordinate += normalized_dimensions;
				}
				break;
			}
			case RadioButtonLayout::VERTICAL:
			{
				double normalized_dimensions = 1.0 / info.options.size();
				double left_coordinate = 0.0;
				for (int index = 0; index < info.options.size(); ++index) {
					RadioItem item;
					item.normalized_position = SkRect::MakeXYWH(0, left_coordinate, 1, normalized_dimensions);
					item.real_item_position = SkRect::MakeXYWH(0, 0, 0.01, 0.01);
					item.text = SkTextBlob::MakeFromString(info.options[index].c_str(), text_font);;
					text_font.measureText(info.options[index].data(), info.options[index].size(), SkTextEncoding::kUTF8, &item.text_size);;
					radio_components.push_back(item);
					left_coordinate += normalized_dimensions;
				}
				break;
			}
			default:

				break;
			}


		}

		void RadioButton::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			SkRect sized_rectangle = SkRect::MakeXYWH(widget_rect.centerX() - size.width() / 2, widget_rect.centerY() - size.height() / 2, size.width(), size.height());
			paint.setStyle(SkPaint::kFill_Style);
			paint.setColor(background_color);
			canvas->drawRect(sized_rectangle, paint);
			paint.setStyle(SkPaint::kStroke_Style);
			paint.setColor(color);

			std::lock_guard<std::mutex> lk(mut);

			for (int index = 0; index < radio_components.size(); ++index)
			{
				RadioItem* item = radio_components.data()+index;

				SkRect temp = SkRect::MakeXYWH(sized_rectangle.width() * item->normalized_position.x() + sized_rectangle.x(),
												sized_rectangle.height() * item->normalized_position.y() + sized_rectangle.y(),
												sized_rectangle.width() * item->normalized_position.width(),
												sized_rectangle.height() * item->normalized_position.height());

				item->real_item_position = SkRect::MakeXYWH(temp.fLeft + DK_RADIO_BUTTON_DIMENSION,
															temp.centerY() - (DK_RADIO_BUTTON_DIMENSION) / 2.0, 
															DK_RADIO_BUTTON_DIMENSION,
															DK_RADIO_BUTTON_DIMENSION);

				canvas->drawRect(item->real_item_position, paint);
				
				if (item->is_selected){
					canvas->drawLine({ item->real_item_position.fLeft,item->real_item_position.fTop }, { item->real_item_position.fRight,item->real_item_position.fBottom }, paint);
					canvas->drawLine({ item->real_item_position.fRight,item->real_item_position.fTop }, { item->real_item_position.fLeft,item->real_item_position.fBottom }, paint);
				}

				canvas->drawTextBlob(item->text, item->real_item_position.fRight+ DK_RADIO_BUTTON_DIMENSION, item->real_item_position.centerY() + item->text_size.height() / 2, paint_text);
			}
		}

		std::shared_ptr<RadioButton> RadioButton::make(Info& info)
		{
			return std::make_shared<RadioButton>(info);
		}

		void RadioButton::callback(Signal signal, bool* interacted)
		{
			switch (signal.signal_type) {
			case Signal::Type::MOUSE_MOVE_SIGNAL:
				//	if (interacts(signal.data.move_signal.xpos, signal.data.move_signal.ypos))
				break;

			case Signal::Type::MOUSE_PRESS_SIGNAL:
			{
				std::lock_guard<std::mutex> lk(mut);
				for (int index = 0; index < radio_components.size(); ++index) {
					RadioItem* item = radio_components.data() + index;
					if (((signal.data.press_signal.xpos > item->real_item_position.fLeft) && (signal.data.press_signal.xpos < item->real_item_position.fRight))
						&& ((signal.data.press_signal.ypos > item->real_item_position.fTop) && (signal.data.press_signal.ypos < item->real_item_position.fBottom)))
					{
						if (current_selected_index >= 0 && current_selected_index!=index && is_exclusive){
							RadioItem* selected_item = radio_components.data() + current_selected_index;
							selected_item->is_selected = false;
						}
						item->is_selected = !item->is_selected;
						current_selected_index = index;
					}
						
				};
				break;
			}
			default:
				break;
			}
		}

		StaticImageDisplay::StaticImageDisplay(Info& info)
		{
			options = SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
			alpha_channel = info.alpha_channel;
			color_type = info.color_type;
			image_to_display = info.initial_image;

			paint.setStyle(SkPaint::kFill_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(4);
			paint.setColor(info.waiting_color);
		}

		void StaticImageDisplay::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			set_position(widget_rect);
			SkAutoCanvasRestore restore(canvas, true);
			std::lock_guard<std::mutex> lk(mut);
			canvas->drawRect(widget_rect, paint);
			if (image_to_display.get() != nullptr)
			{
				float image_width = image_to_display->width();
				float image_height = image_to_display->height();

				float current_selected_width = widget_rect.width();
				float current_selected_height = widget_rect.height();

				float scale_factor = std::min(current_selected_width / image_width, current_selected_height / image_height);

				float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + widget_rect.x();
				float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + widget_rect.y();

				SkRect current_selected_image_rectangle = SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);

				SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0 / 3, 1.0 / 3 });
				canvas->drawImageRect(image_to_display, current_selected_image_rectangle, opt);
			}
		}

		std::shared_ptr<StaticImageDisplay> StaticImageDisplay::make(Info& info)
		{
			return std::make_shared<StaticImageDisplay>(info);
		}

		void StaticImageDisplay::update_image(sk_sp<SkImage> new_image)
		{
			std::lock_guard<std::mutex> lk(mut);
			image_to_display = new_image;
		}

		void MovableImageDisplay::update_color()
		{
			current_hsv_value_h = std::fmod(current_hsv_value_h + 35.0 ,360.0);
			SkScalar ksv[3] = { current_hsv_value_h , 1 , 1};
			path_color = SkHSVToColor(255, ksv);
		}

		MovableImageDisplay::MovableImageDisplay(Info& info)
		{
			options = SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
			current_transform = SkMatrix::SkMatrix();
			alpha_channel = info.alpha_channel;
			color_type = info.color_type;
			hover_color = info.hover_color;
			waiting_color = info.waiting_color;
			image_to_display = info.initial_image;

			paint.setStyle(SkPaint::kStroke_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(4);
			paint.setColor(waiting_color);

			update_color();

			path_paint.setStyle(SkPaint::kStroke_Style);
			path_paint.setAntiAlias(true);
			path_paint.setStrokeWidth(2);
			path_paint.setColor(path_color);
		}

		void MovableImageDisplay::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			set_position(widget_rect);
			SkAutoCanvasRestore restore(canvas, true);
			std::lock_guard<std::mutex> lk(mut);
			canvas->clipRect(widget_rect);
			canvas->drawRect(widget_rect, paint);
			canvas->setMatrix(current_transform);
			canvas->drawImage(image_to_display.get(), widget_rect.x(), widget_rect.y(), options);
			if (path_display_active)
			{
				for (auto path_info : paths_contained)
					if (path_info.show_path){
						path_paint.setColor(path_info.path_color);
						if(path_info.highlighted)
							path_paint.setStrokeWidth(7);
						else
							path_paint.setStrokeWidth(2);
						canvas->drawPath(path_info.path_contained, path_paint);
					}
						
				for (auto point_in_list : current_path_list){
					path_paint.setColor(path_color);
					path_paint.setStrokeWidth(3);
					canvas->drawCircle(point_in_list.fX, point_in_list.fY, 2, path_paint);
				}

				if (purge_resources)
				{
					purge_resources = false;
					canvas->getSurface()->recordingContext()->asDirectContext()->freeGpuResources();
				}
			}
		}

		void MovableImageDisplay::callback(Signal signal, bool* interacted)
		{
			//check if we are changing any path, if we are we need
			//to perform the path corresponding logic, i.e. calculate 
			//the current path in selection, change paths if 
			if (path_display_active) {
				switch (current_path_interaction){
				case PathInteration::CHANGE_PATH:
				{
					static bool is_pressed = false;
					SkPoint point;
					static std::list<PathsInfo>::iterator smallest_possible = paths_contained.end();
					static uint32_t point_index = 0;
					static std::vector<SkPoint> points;

					switch (signal.signal_type) {
					case Signal::Type::MOUSE_MOVE_SIGNAL:
					{
						*interacted = true;
						SkPath path;
						SkMatrix inverse_current_transform = SkMatrix();
						if (current_transform.invert(&inverse_current_transform))
							point = inverse_current_transform.mapXY(signal.data.move_signal.xpos, signal.data.move_signal.ypos);
						if (is_pressed && smallest_possible!= paths_contained.end()) {
							std::vector<SkPoint>::iterator itpoints = points.begin();
							uint32_t index = 0;
							if (index == point_index)
							{
								path.moveTo(point);
								++itpoints;
								++index;
								while (itpoints != points.end()) {
									path.lineTo(*itpoints);
									++itpoints;
									++index;
								}
							}
							else 
							{
								path.moveTo(*itpoints);
								++itpoints;
								++index;
								while (itpoints != points.end()) {
									if (index == point_index)
										path.lineTo(point);
									else
										path.lineTo(*itpoints);
									++itpoints;
									++index;
								}
							}
							std::lock_guard<std::mutex> guard{mut};
							path.close();
							smallest_possible->path_contained.reset();
							smallest_possible->path_contained = path;
						}
					}
						break;
					case Signal::Type::MOUSE_PRESS_SIGNAL:
					{
						*interacted = true;
						SkMatrix inverse_current_transform = SkMatrix();
						if (current_transform.invert(&inverse_current_transform))
							point = inverse_current_transform.mapXY(signal.data.press_signal.xpos, signal.data.press_signal.ypos);
						is_pressed = true;
						std::list<PathsInfo>::iterator it = paths_contained.begin();
						double current_smallest = std::numeric_limits<double>::max();
						SkScalar distance = 0.0f;
						std::vector<SkPoint> potential_points_to_modify;
						for (auto path : paths_contained)
						{
							if (path.show_path)
							{
								int n = path.path_contained.countPoints();
								potential_points_to_modify.resize(n);
								path.path_contained.getPoints(potential_points_to_modify.data(), n);
								path.highlighted = true;
								std::vector<SkPoint>::iterator itpoints = potential_points_to_modify.begin();
								uint32_t index = 0;
								while (itpoints != potential_points_to_modify.end())
								{
									distance = SkPoint::Distance(*itpoints, point);
									if (distance < current_smallest && distance < 50.0)
									{
										point_index = index;
										smallest_possible = it;
									}
									++itpoints;
									++index;
								}
								++it;
							}
						}
						if (smallest_possible != paths_contained.end()){
							std::lock_guard<std::mutex> lk(mut);
							points = std::move(potential_points_to_modify);
							smallest_possible->highlighted = true;
						}

					}
						break;
					case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					{
						is_pressed = false;
						point_index = 0;
						if (smallest_possible != paths_contained.end()){
							purge_resources = true;
							smallest_possible->highlighted = false;
						}
						smallest_possible = paths_contained.end();
					}
						break;
					default:

						break;
					}
				}
					break;
				case PathInteration::ELIMINATE_PATH:
				{
					if (Signal::Type::MOUSE_PRESS_SIGNAL == signal.signal_type)
					{
						*interacted = true;
						SkMatrix inverse_current_transform = SkMatrix();
						SkPoint point;
						if (current_transform.invert(&inverse_current_transform))
							point = inverse_current_transform.mapXY(signal.data.move_signal.xpos, signal.data.move_signal.ypos);
						std::list<PathsInfo>::iterator it = paths_contained.begin();
						std::list<PathsInfo>::iterator smallest_possible = paths_contained.end();
						uint32_t point_index = 0;
						double current_smallest = std::numeric_limits<double>::max();
						SkScalar distance = 0.0f;
						for (auto path : paths_contained)
						{
							if (path.show_path)
							{
								int n = path.path_contained.countPoints();
								std::vector<SkPoint> points(n);
								path.path_contained.getPoints(points.data(), n);
								std::vector<SkPoint>::iterator itpoints = points.begin();
								while (itpoints != points.end())
								{
									distance = SkPoint::Distance(*itpoints, point);
									if (distance < current_smallest && distance < 50.0)
										smallest_possible = it;
									++itpoints;
								}
								++it;
							}
						}
						if (smallest_possible != paths_contained.end()) {
							std::lock_guard<std::mutex> lk(mut);
							paths_contained.erase(smallest_possible);
						}
					}
				}
					break;
				case PathInteration::HIDE_PATH:
				{
					if (signal.signal_type== Signal::Type::MOUSE_PRESS_SIGNAL)
					{
						*interacted = true;
						SkMatrix inverse_current_transform = SkMatrix();
						SkPoint point;
						if (current_transform.invert(&inverse_current_transform))
							point = inverse_current_transform.mapXY(signal.data.press_signal.xpos, signal.data.press_signal.ypos);
						std::list<PathsInfo>::iterator it = paths_contained.begin();
						std::list<PathsInfo>::iterator smallest_possible = paths_contained.end();
						uint32_t point_index = 0;
						double current_smallest = std::numeric_limits<double>::max();
						SkScalar distance = 0.0f;
						for (auto path : paths_contained)
						{
							if (path.show_path)
							{
								int n = path.path_contained.countPoints();
								std::vector<SkPoint> points(n);
								path.path_contained.getPoints(points.data(), n);
								std::vector<SkPoint>::iterator itpoints = points.begin();
								while (itpoints != points.end())
								{
									distance = SkPoint::Distance(*itpoints, point);
									if (distance < current_smallest && distance < 50.0)
										smallest_possible = it;
									++itpoints;
								}
								++it;
							}
						}
						if (smallest_possible != paths_contained.end()) {
							std::lock_guard<std::mutex> lk(mut);
							smallest_possible->show_path = false;
						}
					}
				}
					break;
				case PathInteration::ADD_PATH:
				{
					if (Signal::Type::MOUSE_PRESS_SIGNAL == signal.signal_type)
					{
						*interacted = true;
						SkMatrix inverse_current_transform = SkMatrix();
						if (current_transform.invert(&inverse_current_transform))
						{
							auto point = inverse_current_transform.mapXY(signal.data.press_signal.xpos, signal.data.press_signal.ypos);
							std::lock_guard<std::mutex> lk(mut);
							current_path_list.push_back(point);
						}	
					}
				}
				break;
				default:

					break;
				}
			}
		}

		std::shared_ptr<MovableImageDisplay> MovableImageDisplay::make(Info& info)
		{
			return std::make_shared<MovableImageDisplay>(info);
		}

		void MovableImageDisplay::rotate(SkScalar delta_theta, SkScalar x, SkScalar y)
		{
			SkMatrix rotation_matrix = SkMatrix();
			rotation_matrix.setRotate(delta_theta, x, y);
			std::lock_guard<std::mutex> lk(mut);
			current_transform.postConcat(rotation_matrix);
		}

		void MovableImageDisplay::translate(SkScalar delta_x, SkScalar delta_y)
		{
			std::lock_guard<std::mutex> lk(mut);
			current_transform.postTranslate(delta_x, delta_y);
		}

		void MovableImageDisplay::zoom(SkScalar scale_factor, SkScalar pivot_x, SkScalar pivot_y)
		{
			double x = pivot_x * scale_factor - pivot_x;
			double y = pivot_y * scale_factor - pivot_y;
			std::lock_guard<std::mutex> lk(mut);
			current_transform.preScale(scale_factor, scale_factor);
			current_transform.postTranslate(-x, -y);
		}

		void MovableImageDisplay::update_image(sk_sp<SkImage> new_image)
		{
			std::lock_guard<std::mutex> lk(mut);
			image_to_display = new_image;
		}

		void MovableImageDisplay::hover(bool is_hovering)
		{
			std::lock_guard<std::mutex> lk(mut);
			if (is_hovering)
				paint.setColor(hover_color);
			else
				paint.setColor(waiting_color);
		}

		void MovableImageDisplay::display_paths()
		{
			std::lock_guard<std::mutex> lk(mut);
			path_display_active = !path_display_active;
		}

		void MovableImageDisplay::hide_path()
		{
			std::lock_guard<std::mutex> lk(mut);
			current_path_interaction = PathInteration::HIDE_PATH;
		}

		void MovableImageDisplay::new_path()
		{
			std::lock_guard<std::mutex> lk(mut);
			current_path_interaction = PathInteration::ADD_PATH;
		}

		void MovableImageDisplay::show_all_paths()
		{
			std::lock_guard<std::mutex> lk(mut);
			
			for (auto details : paths_contained)
				details.show_path = true;
		}

		void MovableImageDisplay::change_path()
		{
			std::lock_guard<std::mutex> lk(mut);
			current_path_interaction = PathInteration::CHANGE_PATH;
		}

		void MovableImageDisplay::submit_path()
		{
			std::lock_guard<std::mutex> lk(mut);
			if (!(current_path_list.size() > 2))
				return;
			PathsInfo info;
			info.path_color = path_color;
			std::list<SkPoint>::iterator it = current_path_list.begin();

			SkPath new_path;
			new_path.moveTo(*it);
			++it;
			while (it!= current_path_list.end()){
				new_path.lineTo(*it);
				++it;
			}
			new_path.close();
			info.path_contained = new_path;
			info.path_name = "";
			info.show_path = true;
			paths_contained.push_back(info);
			update_color();
			current_path_list.clear();
		}

		void MovableImageDisplay::eliminate_path()
		{
			std::lock_guard<std::mutex> lk(mut);
			current_path_interaction = PathInteration::ELIMINATE_PATH;
		}

		CommunicationDisplay::CommunicationDisplay(Info& info)
		{
			text_font = info.text_font;

			std::string type = "Type";
			sk_sp<SkTextBlob> text_type = SkTextBlob::MakeFromString(type.c_str(), text_font);
			table_headers.push_back(text_type);

			std::string name = "Name";
			sk_sp<SkTextBlob> text_name = SkTextBlob::MakeFromString(name.c_str(), text_font);
			table_headers.push_back(text_name);

			std::string timestamp = "Timestamp (ms)";
			sk_sp<SkTextBlob> text_timestamp = SkTextBlob::MakeFromString(timestamp.c_str(), text_font);
			table_headers.push_back(text_timestamp);

			std::string freq = "Frequency (Hz)";
			sk_sp<SkTextBlob> text_freq = SkTextBlob::MakeFromString(freq.c_str(), text_font);
			table_headers.push_back(text_freq);

			paint.setStyle(SkPaint::kStroke_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(2);
			paint.setColor(SK_ColorWHITE);


			paint_text.setStyle(SkPaint::kFill_Style);
			paint_text.setAntiAlias(true);
			paint_text.setStrokeWidth(1);
			paint_text.setColor(SK_ColorWHITE);

			std::string information = "i";
			debug_glyph = SkTextBlob::MakeFromString(information.c_str(), text_font);
		}

		void CommunicationDisplay::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			switch (current_status)
			{
			case curan::communication::ProcessorOpenIGTLink::Status::CK_CLOSED:
				paint.setColor(SK_ColorRED);
				paint.setStrokeWidth(40);
				canvas->drawRect(widget_rect, paint);
				break;
			case curan::communication::ProcessorOpenIGTLink::Status::CK_OPEN:
				paint.setColor(SK_ColorGREEN);
				paint.setStrokeWidth(40);
				canvas->drawRect(widget_rect, paint);
				break;
			default:

				break;
			}
			paint.setColor(SK_ColorWHITE);
			paint.setStrokeWidth(1);
			set_position(widget_rect);
			SkAutoCanvasRestore restore(canvas, true);
			canvas->clipRect(widget_rect);
			canvas->clear(SK_ColorBLACK);
			center_debug_mode = { widget_rect.fRight - debug_mode_radius,widget_rect.fTop + debug_mode_radius };
			std::lock_guard guard{ mut };

			if (in_debug_mode) {

				paint.setColor(SK_ColorGRAY);
				paint.setStyle(SkPaint::kFill_Style);
				canvas->drawCircle(center_debug_mode, debug_mode_radius, paint);
				paint.setColor(SK_ColorWHITE);
				paint.setStyle(SkPaint::kStroke_Style);
				canvas->drawTextBlob(debug_glyph, center_debug_mode.fX, center_debug_mode.fY - debug_glyph->bounds().centerY(), paint_text);

				SkScalar height = widget_rect.height() * 0.95;
				SkScalar width = widget_rect.width() * 0.95;
				double x_init = widget_rect.x() + (widget_rect.width() - width) / 2;
				double y_init = widget_rect.y() + (widget_rect.height() - height) / 2;

				int number_of_cells = (height / 2.0) / (double)(DK_DEFAULT_TEXT_SIZE + 5);

				SkRect renctangle = SkRect::MakeXYWH(x_init, y_init, width / 4, DK_DEFAULT_TEXT_SIZE + 5);

				for (int index = { 0 }; index < 4; ++index) {
					canvas->drawRect(renctangle, paint);
					SkRect text_blob_rect = table_headers[index]->bounds();
					canvas->drawTextBlob(table_headers[index], renctangle.x() + (renctangle.width() - text_blob_rect.width()) / 2, renctangle.y() + renctangle.height() - text_blob_rect.bottom(), paint_text);
					renctangle.offsetTo(renctangle.x() + renctangle.width(), renctangle.y());
				}

				renctangle.offsetTo(x_init, y_init + renctangle.height());

				std::map<std::string, MessageInfo>::iterator it = table_conversion.begin();
				std::map<std::string, MessageInfo>::iterator clicked_it = table_conversion.end();
				for (int index = 0; index < number_of_cells; ++index) {
					if (renctangle.fBottom > last_pressed_position.ypos && renctangle.fTop < last_pressed_position.ypos) {
						paint.setStyle(SkPaint::kFill_Style);
						paint.setColor(SK_ColorDKGRAY);
						clicked_it = it;
					}
					else {
						paint.setStyle(SkPaint::kStroke_Style);
						paint.setColor(SK_ColorWHITE);
					}

					if (it == table_conversion.end()) {
						SkRect rect = renctangle;
						canvas->drawRect(rect, paint);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
					}
					else {
						MessageInfo& info = it->second;
						SkRect rect = renctangle;
						canvas->drawRect(rect, paint);
						std::string type = info.message_type;
						canvas->drawSimpleText(type.data(), type.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						std::string device_name = info.message_devide_name;
						canvas->drawSimpleText(device_name.data(), device_name.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(info.timestamp.time_since_epoch()).count());
						canvas->drawSimpleText(timestamp.data(), timestamp.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
						rect.offsetTo(rect.x() + rect.width(), rect.y());
						canvas->drawRect(rect, paint);
						std::string frequency = std::to_string(info.frequency);
						canvas->drawSimpleText(frequency.data(), frequency.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
						renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
						++it;
					}

				}
				std::string preview = "Preview: ";
				canvas->drawSimpleText(preview.data(), preview.size(), SkTextEncoding::kUTF8, x_init, y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5), text_font, paint_text);

				if (clicked_it != table_conversion.end()) {
					MessageInfo& selected_message = clicked_it->second;
					switch (selected_message.getType()) {
					case MessageType::IMAGE:
					{
						SkScalar y = y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5);
						SkSamplingOptions options;
						sk_sp<SkImage> surf;
						igtl::ImageMessage::Pointer img;
						if (selected_message.getImage(img, surf)) {
							canvas->drawImage(surf, widget_rect.x() + (widget_rect.width() - surf->width()) / 2.0, y);
						}
					}
					break;
					case MessageType::TRANSFORM:
					{
						SkScalar y = y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5);
						// Retrive the transform data
						igtl::TransformMessage::Pointer transf;
						if (selected_message.getTransform(transf)) {
							igtl::Matrix4x4 matrix;
							transf->GetMatrix(matrix);
							for (int i = 0; i < 4; ++i) {
								std::string row = "[";
								for (int j = 0; j < 4; ++j)
									row += curan::utils::to_string_with_precision(matrix[i][j], 2) + " , ";
								row += "]";
								SkRect bound_individual_name;
								text_font.measureText(row.c_str(), row.size(), SkTextEncoding::kUTF8, &bound_individual_name);
								canvas->drawSimpleText(row.c_str(), row.size(), SkTextEncoding::kUTF8, (widget_rect.width() - bound_individual_name.width()) / 2, y + bound_individual_name.height(), text_font, paint_text);
								y += bound_individual_name.height() + 5;
							}
						}
					}
					break;
					default:

						break;
					}
				}
			}
			else {
				canvas->drawCircle(center_debug_mode, debug_mode_radius, paint);
				canvas->drawTextBlob(debug_glyph, center_debug_mode.fX, center_debug_mode.fY - debug_glyph->bounds().centerY(), paint_text);
				std::map<std::string, MessageInfo>::iterator it = table_conversion.begin();
				while (it != table_conversion.end())
				{
					if (it->second.getType() == MessageType::IMAGE) {
						SkSamplingOptions options;
						sk_sp<SkImage> surf;
						igtl::ImageMessage::Pointer img;
						it->second.getImage(img, surf);
						canvas->drawImage(surf, widget_rect.x() + (widget_rect.width() - surf->width()) / 2.0, widget_rect.y() + (widget_rect.height() - surf->height()) / 2.0);
						return;
					}
					++it;
				}

			}
		}

		std::shared_ptr<CommunicationDisplay> CommunicationDisplay::make(Info& info)
		{
			return std::make_shared<CommunicationDisplay>(info);
		}

		void CommunicationDisplay::callback(Signal signal, bool* interacted) {

			switch (signal.signal_type)
			{
			case Signal::Type::MOUSE_MOVE_SIGNAL:

				break;
			case Signal::Type::MOUSE_PRESS_SIGNAL:
			{
				*interacted = true;
				last_pressed_position = signal.data.press_signal;
				float x = signal.data.press_signal.xpos - center_debug_mode.fX;
				float y = signal.data.press_signal.ypos - center_debug_mode.fY;
				if (x * x + y * y < debug_mode_radius * debug_mode_radius) {
					std::lock_guard guard{ mut };
					in_debug_mode = !in_debug_mode;
				}
			}
			break;
			case Signal::Type::MOUSE_UNPRESS_SIGNAL:

				break;
			default:

				break;
			}
		}

		void CommunicationDisplay::process_message(igtl::MessageBase::Pointer pointer)
		{
			if (pointer->GetMessageType() == "TRANSFORM") {
				std::string s = pointer->GetDeviceName();
				std::string identifier = "TRANSFORM" + s;

				std::map<std::string, MessageInfo>::iterator it;
				{
					std::lock_guard guard{ mut };
					it = table_conversion.find(identifier);
				}

				igtl::TransformMessage::Pointer message_body = igtl::TransformMessage::New();
				//We copy the contents of the received message into the local type and then we unpack it
				if (message_body->Copy(pointer) && message_body->Unpack(1) == igtl::MessageBase::UNPACK_BODY)
				{
					if (it == table_conversion.end()) {

						auto current_time_point = std::chrono::steady_clock::now();

						MessageInfo info{ MessageType::TRANSFORM };
						info.frequency = 1.0 / std::chrono::duration<double>(current_time_point - info.timestamp).count();
						info.setTransform(message_body);
						info.timestamp = std::chrono::steady_clock::now();
						info.message_devide_name = message_body->GetDeviceName();
						info.message_type = message_body->GetDeviceType();
						{
							std::lock_guard guard{ mut };
							table_conversion.emplace(identifier, info);
						}
					}
					else {
						auto current_time_point = std::chrono::steady_clock::now();
						it->second.frequency = 1.0 / std::chrono::duration<double>(current_time_point - it->second.timestamp).count();
						it->second.setTransform(message_body);
						it->second.timestamp = std::chrono::steady_clock::now();
						it->second.message_devide_name = message_body->GetDeviceName();
						it->second.message_type = message_body->GetDeviceType();
					}
				}


				return;
			}

			if (pointer->GetMessageType() == "IMAGE") {
				std::string s = pointer->GetDeviceName();
				std::string identifier = "IMAGE" + s;
				std::map<std::string, MessageInfo>::iterator it;
				{
					std::lock_guard guard{ mut };
					it = table_conversion.find(identifier);
				}

				igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
				//We copy the contents of the received message into the local type and then we unpack it
				if (message_body->Copy(pointer) && message_body->Unpack(1) == igtl::MessageBase::UNPACK_BODY)
				{
					if (it == table_conversion.end()) {

						auto current_time_point = std::chrono::steady_clock::now();

						MessageInfo info{ MessageType::IMAGE };
						info.frequency = 1.0 / std::chrono::duration<double>(current_time_point - info.timestamp).count();
						{
							std::lock_guard guard{ mut };
							info.createSurface(message_body);
							info.setImage(message_body);
						}
						info.timestamp = std::chrono::steady_clock::now();
						info.message_devide_name = message_body->GetDeviceName();
						info.message_type = message_body->GetDeviceType();
						{
							std::lock_guard guard{ mut };
							table_conversion.emplace(identifier, info);
						}
					}
					else {
						auto current_time_point = std::chrono::steady_clock::now();

						it->second.frequency = 1.0 / std::chrono::duration<double>(current_time_point - it->second.timestamp).count();
						{
							std::lock_guard guard{ mut };
							it->second.setImage(message_body);
						}
						it->second.timestamp = std::chrono::steady_clock::now();
						it->second.message_devide_name = message_body->GetDeviceName();
						it->second.message_type = message_body->GetDeviceType();
					}
				}
				return;
			}

		}

		void CommunicationDisplay::connection_status(curan::communication::ProcessorOpenIGTLink::Status status)
		{
			current_status = status;
		}

		CommunicationDisplay::MessageInfo::MessageInfo(MessageType in_type) : type{ in_type }
		{

		}

		CommunicationDisplay::MessageInfo::~MessageInfo()
		{

		}

		void CommunicationDisplay::MessageInfo::setTransform(igtl::TransformMessage::Pointer p)
		{
			pointer.transform = p;
		}



		void CommunicationDisplay::MessageInfo::setImage(igtl::ImageMessage::Pointer p)
		{
			if (p->GetImageSize() == information.bytesPerPixel() * information.width() * information.height()) {
				pointer.image = p;
				SkPixmap pixel_array = SkPixmap(information, p->GetScalarPointer(), information.bytesPerPixel() * information.width());
				trial = SkImage::MakeRasterCopy(pixel_array);
			}
			else
			{
				curan::utils::console->info("Buffer size: " + std::to_string(p->GetImageSize()));
				curan::utils::console->info("Expected buffer size (width x height x bytes per pixel): (" + std::to_string(information.width()) + "," + std::to_string(information.height()) + "," + std::to_string(information.bytesPerPixel()) + ")" + "=" + std::to_string(information.bytesPerPixel() * information.width() * information.height()));
			}


		}

		void CommunicationDisplay::MessageInfo::createSurface(igtl::ImageMessage::Pointer p)
		{
			int width, height, depth = 0;
			p->GetDimensions(width, height, depth);
			curan::utils::console->info("Image size (x,y,h):" + std::to_string(width) + "," + std::to_string(height) + "," + std::to_string(depth));

			if (p->GetNumComponents() != 1)
				return;

			switch (p->GetScalarType())
			{
			case igtl::ImageMessage::TYPE_UINT16:
			case igtl::ImageMessage::TYPE_UINT32:
			case igtl::ImageMessage::TYPE_FLOAT32:
			case igtl::ImageMessage::TYPE_FLOAT64:
			case igtl::ImageMessage::TYPE_INT16:
			case igtl::ImageMessage::TYPE_INT32:
				return;
			case igtl::ImageMessage::TYPE_INT8:
				information = SkImageInfo::Make(width, height, SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
				break;
			case igtl::ImageMessage::TYPE_UINT8:
				information = SkImageInfo::Make(width, height, SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType);
				break;
			default:

				return;
			}
		}

		bool CommunicationDisplay::MessageInfo::getTransform(igtl::TransformMessage::Pointer& p)
		{
			if (type != MessageType::TRANSFORM)
				return false;
			p = pointer.transform;
			return true;
		}

		bool CommunicationDisplay::MessageInfo::getImage(igtl::ImageMessage::Pointer& p, sk_sp<SkImage>& in_surface)
		{
			if (type != MessageType::IMAGE)
				return false;
			p = pointer.image;
			in_surface = trial;
			return true;
		}

		CommunicationDisplay::MessageType CommunicationDisplay::MessageInfo::getType()
		{
			return type;
		}

		void Bufferring::draw(SkCanvas* canvas, SkPoint center)
		{
			auto current = std::chrono::steady_clock::now();
			if (std::chrono::duration<double>(current - previous_timepoint).count() > 0.015) {
				previous_timepoint = current;
				rotation += 2.5;
			}

			SkMatrix matrix = SkMatrix();
			matrix.setRotate(rotation);

			SkMatrix matrix2 = SkMatrix();
			matrix2.setTranslate(center.fX, center.fY);

			SkMatrix final = matrix.postConcat(matrix2);
			std::lock_guard<std::mutex> lk(mut);
			SkAutoCanvasRestore restore(canvas, true);
			canvas->setMatrix(final);
			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, points.size(), points.data(), paint);
		}

		void Bufferring::resize(double new_radius)
		{
			std::lock_guard<std::mutex> lk(mut);
			const double pi = std::acos(-1);
			points.resize(16);
			int index = 0;
			for (double angle = 0.0; angle < 2 * pi; angle += pi * 22.5 / 180)
			{
				SkPoint point = { std::cos(angle) * new_radius, std::sin(angle) * new_radius };
				points[index] = point;
				++index;
			}
		}

		Bufferring::Bufferring(double radius)
		{
			std::lock_guard<std::mutex> lk(mut);
			const double pi = std::acos(-1);
			points.resize(16);
			int index = 0;
			for (double angle = 0.0; angle < 2 * pi; angle += pi * 22.5 / 180)
			{
				SkPoint point = { std::cos(angle) * radius, std::sin(angle) * radius };
				points[index] = point;
				++index;
			}

			color = SK_ColorCYAN;

			paint.setStyle(SkPaint::kStroke_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(6);
			paint.setColor(color);
		}


		Signal::Signal() : Signal(Signal::Type::EMPTY_SIGNAL, Data())
		{

		}

		Signal::Signal(Signal::Type in_type, Data in_data)
		{
			signal_type = in_type;
			data = in_data;
		}

		Icon Icon::read(const char* s)
		{
			Icon icon;
			icon.pixels = stbi_load(s, &icon.texWidth, &icon.texHeight, &icon.texChannels, STBI_rgb_alpha);
			if (!icon.pixels) {
				throw std::runtime_error("failed to load texture image!");
			}
			SkImageInfo information = SkImageInfo::Make(icon.texWidth, icon.texHeight, kBGRA_8888_SkColorType, kUnpremul_SkAlphaType);
			icon.pixmap = SkPixmap(information, icon.pixels, icon.texWidth * NUMBER_BYTES_PER_PIXEL);
			icon.image_to_display = SkImage::MakeFromRaster(icon.pixmap, nullptr, nullptr);
			return icon;
		}

		IconResources::IconResources(std::string path_to_resources)
		{
			is_initialized = true;
			for (auto& p : std::filesystem::directory_iterator(path_to_resources))
			{
				icon_map.emplace(p.path().filename().u8string(), Icon::read(p.path().u8string().c_str()));
			}
		}

		bool IconResources::is_initialized = false;

		IconResources* IconResources::Get()
		{
			if (is_initialized)
				return Load("");
			else
				return nullptr;
		}

		IconResources* IconResources::Load(std::string path_to_resources)
		{
			static IconResources icon_loader{ path_to_resources };
			return &icon_loader;
		}

		void IconResources::GetIcon(sk_sp<SkImage>& image, std::string icon_string)
		{
			auto item_in_map = icon_map.find(icon_string);
			if (item_in_map != icon_map.end()) {
				image = item_in_map->second.image_to_display;
			}
		}

		OptionBox::OptionBox(Info& info)
		{
			movable = info.movable;
			contained_page = info.page_to_draw;
			option_box_size = info.option_box_size;
		};

		void OptionBox::draw(SkCanvas* canvas, SkRect& widget_rect)
		{
			SkRect position_in_space = SkRect::MakeXYWH(origin.fX+ widget_rect.fLeft, origin.fY+ widget_rect.fTop, option_box_size.fWidth, option_box_size.fHeight);
			set_position(position_in_space);
			contained_page->draw(canvas, position_in_space);
		};

		std::shared_ptr<OptionBox> OptionBox::make(Info& info)
		{
			return std::make_shared<OptionBox>(info);
		}

		void OptionBox::callback(Signal signal, bool* interacted)
		{
			contained_page->callback(signal, interacted);
			if (!(*interacted) && movable)
			{
				static bool is_pressed = false;
				static SkPoint previous;
				switch (signal.signal_type)
				{
				case Signal::Type::MOUSE_PRESS_SIGNAL:
					if (interacts(signal.data.press_signal.xpos, signal.data.press_signal.ypos)){
						previous = SkPoint::Make((float)signal.data.press_signal.xpos, (float)signal.data.press_signal.ypos);
						is_pressed = true;
					}
					break;
				case Signal::Type::MOUSE_MOVE_SIGNAL:
					if (is_pressed)
					{
						SkPoint current_point = SkPoint::Make(signal.data.move_signal.xpos, signal.data.move_signal.ypos);
						origin = SkPoint::Make(origin.fX+ current_point.fX- previous.fX,origin.fY+ current_point.fY- previous.fY);
						previous = current_point;
					}

					break;
				case Signal::Type::MOUSE_UNPRESS_SIGNAL:
					is_pressed = false;
					break;
				default:

					break;
				}
			}

		};

		void OptionBox::change_page(std::shared_ptr<Page> page)
		{
			contained_page = page;
		};
}
}