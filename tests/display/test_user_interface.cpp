
#define GLFW_INCLUDE_VULKAN

#include <GLFW/glfw3.h>

#include <vulkan/vulkan.h>

#include "include/core/SkTypes.h"
#include "include/core/SkAlphaType.h"
#include "include/core/SkAnnotation.h"
#include "include/core/SkBBHFactory.h"
#include "include/core/SkBitmap.h"
#include "include/core/SkBlendMode.h"
#include "include/core/SkBlender.h"
#include "include/core/SkBlurTypes.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkCanvasVirtualEnforcer.h"
#include "include/core/SkCapabilities.h"
#include "include/core/SkClipOp.h"
#include "include/core/SkColor.h"
#include "include/core/SkColorFilter.h"
#include "include/core/SkColorPriv.h"
#include "include/core/SkColorSpace.h"
#include "include/core/SkColorType.h"
#include "include/core/SkContourMeasure.h"
#include "include/core/SkCoverageMode.h"
#include "include/core/SkCubicMap.h"
#include "include/core/SkData.h"
#include "include/core/SkDataTable.h"
#include "include/core/SkDocument.h"
#include "include/core/SkDrawable.h"
#include "include/core/SkExecutor.h"
#include "include/core/SkFlattenable.h"
#include "include/core/SkFont.h"
#include "include/core/SkFontArguments.h"
#include "include/core/SkFontMetrics.h"
#include "include/core/SkFontMgr.h"
#include "include/core/SkFontParameters.h"
#include "include/core/SkFontStyle.h"
#include "include/core/SkFontTypes.h"
#include "include/core/SkGraphics.h"
#include "include/core/SkImage.h"
#include "include/core/SkImageFilter.h"
#include "include/core/SkImageGenerator.h"
#include "include/core/SkImageInfo.h"
#include "include/core/SkM44.h"
#include "include/core/SkMallocPixelRef.h"
#include "include/core/SkMaskFilter.h"
#include "include/core/SkMatrix.h"
#include "include/core/SkMesh.h"
#include "include/core/SkMilestone.h"
#include "include/core/SkOpenTypeSVGDecoder.h"
#include "include/core/SkOverdrawCanvas.h"
#include "include/core/SkPaint.h"
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/core/SkPathEffect.h"
#include "include/core/SkPathMeasure.h"
#include "include/core/SkPathTypes.h"
#include "include/core/SkPicture.h"
#include "include/core/SkPictureRecorder.h"
#include "include/core/SkPixelRef.h"
#include "include/core/SkPixmap.h"
#include "include/core/SkPoint.h"
#include "include/core/SkPoint3.h"
#include "include/core/SkRRect.h"
#include "include/core/SkRSXform.h"
#include "include/core/SkRasterHandleAllocator.h"
#include "include/core/SkRect.h"
#include "include/core/SkRefCnt.h"
#include "include/core/SkRegion.h"
#include "include/core/SkSamplingOptions.h"
#include "include/core/SkScalar.h"
#include "include/core/SkSerialProcs.h"
#include "include/core/SkShader.h"
#include "include/core/SkSize.h"
#include "include/core/SkSpan.h"
#include "include/core/SkStream.h"
#include "include/core/SkString.h"
#include "include/core/SkStrokeRec.h"
#include "include/core/SkSurface.h"
#include "include/core/SkSurfaceProps.h"
#include "include/core/SkSwizzle.h"
#include "include/core/SkTextBlob.h"
#include "include/core/SkTileMode.h"
#include "include/core/SkTraceMemoryDump.h"
#include "include/core/SkTypeface.h"
#include "include/core/SkUnPreMultiply.h"
#include "include/core/SkVertices.h"
#include "include/core/SkYUVAInfo.h"
#include "include/core/SkYUVAPixmaps.h"
#include "include/docs/SkPDFDocument.h"
#include "include/docs/SkXPSDocument.h"
#include "include/effects/Sk1DPathEffect.h"
#include "include/effects/Sk2DPathEffect.h"
#include "include/effects/SkBlenders.h"
#include "include/effects/SkBlurMaskFilter.h"
#include "include/effects/SkColorMatrix.h"
#include "include/effects/SkColorMatrixFilter.h"
#include "include/effects/SkCornerPathEffect.h"
#include "include/effects/SkDashPathEffect.h"
#include "include/effects/SkDiscretePathEffect.h"
#include "include/effects/SkGradientShader.h"
#include "include/effects/SkHighContrastFilter.h"
#include "include/effects/SkImageFilters.h"
#include "include/effects/SkLumaColorFilter.h"
#include "include/effects/SkOverdrawColorFilter.h"
#include "include/effects/SkPerlinNoiseShader.h"
#include "include/effects/SkRuntimeEffect.h"
#include "include/effects/SkShaderMaskFilter.h"
#include "include/effects/SkTableMaskFilter.h"
#include "include/effects/SkTrimPathEffect.h"
#include "include/encode/SkJpegEncoder.h"
#include "include/gpu/ganesh/SkSurfaceGanesh.h"
#include "include/gpu/vk/VulkanExtensions.h"
#include "include/gpu/GpuTypes.h"
#include "include/gpu/GrBackendSemaphore.h"
#include "include/gpu/GrBackendSurface.h"
#include "include/gpu/GrContextOptions.h"
#include "include/gpu/GrContextThreadSafeProxy.h"
#include "include/gpu/GrDirectContext.h"
#include "include/gpu/GrDriverBugWorkarounds.h"
#include "include/gpu/GrDriverBugWorkaroundsAutogen.h"
#include "include/gpu/GrRecordingContext.h"
#include "include/gpu/GrTypes.h"
#include "include/gpu/GrYUVABackendTextures.h"
#include "include/gpu/ShaderErrorHandler.h"
#include "include/gpu/mock/GrMockTypes.h"
#include "include/gpu/vk/VulkanBackendContext.h"
#include "include/gpu/vk/VulkanExtensions.h"
#include "include/gpu/vk/VulkanMemoryAllocator.h"
#include "include/gpu/vk/GrVkTypes.h"
#include "include/pathops/SkPathOps.h"
#include "include/ports/SkTypeface_win.h"
#include "include/utils/SkCamera.h"
#include "include/utils/SkCanvasStateUtils.h"
#include "include/utils/SkCustomTypeface.h"
#include "include/utils/SkEventTracer.h"
#include "include/utils/SkNWayCanvas.h"
#include "include/utils/SkNoDrawCanvas.h"
#include "include/utils/SkNullCanvas.h"
#include "include/utils/SkOrderedFontMgr.h"
#include "include/utils/SkPaintFilterCanvas.h"
#include "include/utils/SkParse.h"
#include "include/utils/SkParsePath.h"
#include "include/utils/SkShadowUtils.h"
#include "include/utils/SkTextUtils.h"
#include "include/utils/SkTraceEventPhase.h"
#include "include/utils/mac/SkCGUtils.h"
#include "include/gpu/vk/VulkanMutableTextureState.h"
#include "include/gpu/ganesh/vk/GrVkDirectContext.h"
#include "include/gpu/ganesh/vk/GrVkBackendSemaphore.h"
#include "include/gpu/ganesh/vk/GrVkBackendSurface.h"
#include "include/gpu/ganesh/vk/GrBackendDrawableInfo.h"

#if defined(SK_FONTMGR_FONTCONFIG_AVAILABLE)
#include "include/ports/SkFontMgr_fontconfig.h"
#endif

#include <string>
#include <variant>
#include "utils/SafeQueue.h"
#include <iostream>

/*
Initial width of the window on the screen.
*/
const uint32_t WIDTH = 1200;

/*
Initial height of the window on the screen.
*/
const uint32_t HEIGHT = 1000;

const int MAX_FRAMES_IN_FLIGHT = 2;
const int EXTRA_BACK_BUFFER = 1;

struct Empty {

};

struct Move
{
	double xpos;
	double ypos;
};

struct Press
{
	double xpos;
	double ypos;
};

struct Scroll
{
	double xpos;
	double ypos;
	double xoffset;
	double yoffset;
};

struct Unpress
{
	double xpos;
	double ypos;
};

struct ItemDropped
{
	int count;
	std::vector<std::string> paths;
};

using Signal = std::variant<Move, Press, Scroll, Unpress, ItemDropped, Empty>;

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);

void cursor_position_click_callback(GLFWwindow* window, int button, int action, int mods);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void item_droped_callback(GLFWwindow* window, int count, const char** paths);

using init_callback = std::function<void(void)>;

struct QueueFamilyIndices {
	std::optional<uint32_t> graphicsFamily;
	std::optional<uint32_t> presentFamily;

	bool is_complete() {
		return graphicsFamily.has_value() && presentFamily.has_value();
	}
};

class Context {
	VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;

public:
	
	VkInstance instance = VK_NULL_HANDLE;
	VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
	std::unique_ptr<skgpu::VulkanExtensions> extensions = nullptr;
	std::vector<const char*> deviceLayerNames;
	std::vector<const char*> deviceExtensionNames;
	QueueFamilyIndices indices;
	skgpu::VulkanGetProc vulkan_pointer = VK_NULL_HANDLE;

	Context() {
		initialize_context();
	}
	~Context() {
		destroy_context();
	}

private:

	bool initialize_context() {
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
		if (res != VkResult::VK_SUCCESS) {
			return false;
		}

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

		if (res != VkResult::VK_SUCCESS) {
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

				if (indices.is_complete()) {
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
			extensions = std::make_unique<skgpu::VulkanExtensions>();

		extensions->init(vulkan_pointer, instance, physicalDevice,
			(uint32_t)instanceExtensionNames.size(),
			instanceExtensionNames.data(),
			(uint32_t)deviceExtensionNames.size(),
			deviceExtensionNames.data());
		return true;
	}
	bool init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers) {
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
	bool init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions) {
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
	void destroy_context() {
		if (instance != VK_NULL_HANDLE)
			vkDestroyInstance(instance, nullptr);

		glfwTerminate();
	}
	static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
		std::cout << "validation layer: " << pCallbackData->pMessage << std::endl;

		return VK_FALSE;
	}
	void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
		createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
		createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
		createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
		createInfo.pfnUserCallback = debugCallback;
	}
};

struct DisplayParams {
	DisplayParams(std::unique_ptr<Context> cxt) 
		: fColorType(kN32_SkColorType)
		, fColorSpace(nullptr)
		, fMSAASampleCount(1)
		, fSurfaceProps(0, kRGB_H_SkPixelGeometry)
		, fDisableVsync(false)
		, fDelayDrawableAcquisition(false)
		, fEnableBinaryArchive(false)
		, cxt{std::move(cxt)}
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
};

struct BackbufferInfo {
	uint32_t        fImageIndex;
	VkSemaphore     fRenderSemaphore;
};

class Window {
private:
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
	DisplayParams params;
	VkSwapchainKHR swapChain = VK_NULL_HANDLE;
	std::vector<VkImage> swapChainImages;
	std::vector<VkImageLayout> swapChainImageLayout;
	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;
	VkImageUsageFlags usageFlags;
	size_t currentFrame = 0;
	bool framebufferResized = false;

public:
	GLFWwindow* window = nullptr;
	curan::utilities::SafeQueue<Signal> signal_queue;

	Window(DisplayParams&& pars) : params{ std::move(pars) }{
		context = std::move(params.cxt);
		params.cxt = nullptr;
		initialize();
		connect_handler();
	};

	~Window()
	{
		destroy();
	};

	void swapBuffers()
	{
		BackbufferInfo* backbuffer = fBackbuffers.get() + fCurrentBackbufferIndex;
	SkSurface* surface = swapSurface[backbuffer->fImageIndex];

	GrBackendSemaphore beSemaphore = GrBackendSemaphores::MakeVk(backbuffer->fRenderSemaphore);

	GrFlushInfo info;
	info.fNumSemaphores = 1;
	info.fSignalSemaphores = &beSemaphore;
	skgpu::MutableTextureState presentState = skgpu::MutableTextureStates::MakeVulkan(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, context->indices.presentFamily.value());

	auto dContext = surface->recordingContext()->asDirectContext();
    dContext->flush(surface, info, &presentState);
    dContext->submit();

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

	SkSurface* getBackbufferSurface()
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
	GrBackendSemaphore beSemaphore = GrBackendSemaphores::MakeVk(semaphore);

	surface->wait(1, &beSemaphore);
	return surface;
	}

	void connect_handler() {
		glfwSetCursorPosCallback(this->window, cursor_position_callback);
		glfwSetMouseButtonCallback(this->window, cursor_position_click_callback);
		glfwSetScrollCallback(this->window, scroll_callback);
		glfwSetDropCallback(this->window, item_droped_callback);
	}

	bool initialize()
	{
		VkResult res;
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
			vkContext = std::make_unique<skgpu::VulkanBackendContext>();

		vkContext->fInstance = context->instance;
		vkContext->fPhysicalDevice = context->physicalDevice;
		vkContext->fDevice = device;
		vkContext->fQueue = graphicsQueue;
		vkContext->fGraphicsQueueIndex = context->indices.graphicsFamily.value();
		vkContext->fMaxAPIVersion = VK_API_VERSION_1_0;
		vkContext->fVkExtensions = context->extensions.get();
		vkContext->fDeviceFeatures = nullptr;
		vkContext->fGetProc = context->vulkan_pointer;
		vkContext->fProtectedContext = GrProtected::kNo;

		// so far i dont think we have any special needs,
		// but this might change in the future
		GrContextOptions options;

		skia_context = std::unique_ptr<GrDirectContext>(GrDirectContexts::MakeVulkan(*vkContext, options).release());
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
			info.fAlloc = skgpu::VulkanAlloc();
			info.fImageLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			info.fImageTiling = VK_IMAGE_TILING_OPTIMAL;
			info.fFormat = swapChainImageFormat;
			info.fLevelCount = 1;
			info.fCurrentQueueFamily = context->indices.presentFamily.value();

		params.fColorSpace = SkColorSpace::MakeSRGBLinear();

		if (usageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
			GrBackendTexture backendTexture = GrBackendTextures::MakeVk(swapChainExtent.width, swapChainExtent.height, info);
            swapSurface[i] = SkSurfaces::WrapBackendTexture(skia_context.get(),
                                                          backendTexture,
                                                          kTopLeft_GrSurfaceOrigin,
                                                          params.fMSAASampleCount,
                                                          colorType,
                                                          params.fColorSpace,
                                                          &params.fSurfaceProps).release();
		} else {
			if (params.fMSAASampleCount > 1) {
				throw std::runtime_error("Could not deal with input definitions");
			}
			info.fSampleCount = std::max(1, params.fMSAASampleCount);;
            GrBackendRenderTarget backendRT = GrBackendRenderTargets::MakeVk(swapChainExtent.width, swapChainExtent.height, info);
            swapSurface[i] = SkSurfaces::WrapBackendRenderTarget(skia_context.get(),
                                                               backendRT,
                                                               kTopLeft_GrSurfaceOrigin,
                                                               colorType,
                                                               params.fColorSpace,
                                                               &params.fSurfaceProps).release();
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

	void destroy()
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

		vkDestroySwapchainKHR(device, swapChain, nullptr);
		vkDestroyDevice(device, nullptr);
		vkDestroySurfaceKHR(context->instance, surface, nullptr);
		glfwDestroyWindow(window);

		return;
	}

	void process_pending_signals()
	{
		size_t size = signal_queue.size();
		for (size_t index = 0; index < size; ++index) {
			if(auto signal = signal_queue.try_pop(); signal){
				std::cout << "coule not pop because something went wrong\n";
			}
		}
		std::cout << "signals processed" << size << "\n";
	}
	bool recreateDisplay()
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
			info.fAlloc = skgpu::VulkanAlloc();
			info.fImageLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			info.fImageTiling = VK_IMAGE_TILING_OPTIMAL;
			info.fFormat = swapChainImageFormat;
			info.fLevelCount = 1;
			info.fCurrentQueueFamily = context->indices.presentFamily.value();

		params.fColorSpace = SkColorSpace::MakeSRGBLinear();

		if (usageFlags & VK_IMAGE_USAGE_SAMPLED_BIT) {
			GrBackendTexture backendTexture = GrBackendTextures::MakeVk(swapChainExtent.width, swapChainExtent.height, info);
            swapSurface[i] = SkSurfaces::WrapBackendTexture(skia_context.get(),
                                                          backendTexture,
                                                          kTopLeft_GrSurfaceOrigin,
                                                          params.fMSAASampleCount,
                                                          colorType,
                                                          params.fColorSpace,
                                                          &params.fSurfaceProps).release();
		} else {
			if (params.fMSAASampleCount > 1) {
				throw std::runtime_error("Could not deal with input definitions");
			}
			info.fSampleCount = std::max(1, params.fMSAASampleCount);;
            GrBackendRenderTarget backendRT = GrBackendRenderTargets::MakeVk(swapChainExtent.width, swapChainExtent.height, info);
            swapSurface[i] = SkSurfaces::WrapBackendRenderTarget(skia_context.get(),
                                                               backendRT,
                                                               kTopLeft_GrSurfaceOrigin,
                                                               colorType,
                                                               params.fColorSpace,
                                                               &params.fSurfaceProps).release();
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
	
	BackbufferInfo* getAvailableBackBuffer()
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
	static void framebufferResizeCallback(GLFWwindow* window, int width, int height)
	{
		auto app = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
		app->framebufferResized = true;
	}
};

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	Move data{ xpos,ypos };
	Signal received_signal = data;
	auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
	window_pointer->signal_queue.push(received_signal);
	std::cout << "signal received" << "/n";
}

void cursor_position_click_callback(GLFWwindow* window, int button, int action, int mods) {
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	Signal received_signal;
	switch (action) {
	case GLFW_PRESS:
	{
		Press val{ xpos,ypos };
		received_signal = val;
	}
	break;
	case GLFW_RELEASE:
	{
		Unpress val{ xpos,ypos };
		received_signal = val;

	}
	break;
	}
	auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
	window_pointer->signal_queue.push(received_signal);
	std::cout << "signal received" << "/n";
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	Signal received_signal;
	Scroll val{ xpos,ypos,xoffset,yoffset };
	received_signal = val;
	auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
	window_pointer->signal_queue.push(received_signal);
	std::cout << "signal received" << "/n";
}

void item_droped_callback(GLFWwindow* window, int count, const char** paths) {
	ItemDropped dropped;
	dropped.count = count;
	dropped.paths.resize(dropped.count);
	for (int index = 0; index < count; ++index) {
		std::string path_i{ paths[index] };
		dropped.paths[index] = path_i;
	}
	Signal received_signal = dropped;
	auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
	window_pointer->signal_queue.push(received_signal);
	std::cout << "signal received" << "/n";
}

int main() {
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context) };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		canvas->drawColor(SK_ColorWHITE);

		SkPaint paint;
		paint.setStyle(SkPaint::kFill_Style);
		paint.setAntiAlias(true);
		paint.setStrokeWidth(4);
		paint.setColor(0xff4285F4);

		SkRect rect = SkRect::MakeXYWH(10, 10, 100, 160);
		canvas->drawRect(rect, paint);

		SkRRect oval;
		oval.setOval(rect);
		oval.offset(40, 80);
		paint.setColor(0xffDB4437);
		canvas->drawRRect(oval, paint);

		paint.setColor(0xff0F9D58);
		canvas->drawCircle(180, 50, 25, paint);

		rect.offset(80, 50);
		paint.setColor(0xffF4B400);
		paint.setStyle(SkPaint::kStroke_Style);
		canvas->drawRoundRect(rect, 10, 10, paint);
		glfwPollEvents();
		viewer->process_pending_signals();
		viewer->swapBuffers();
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	return 0;
}