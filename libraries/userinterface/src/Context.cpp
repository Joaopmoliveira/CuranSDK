#include "userinterface/Context.h"

namespace curan {
	namespace ui {

		Context::Context() {
			initialize_context();
		}
		Context::~Context() {
			destroy_context();
		}

		bool Context::initialize_context() {
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
				extensions = std::unique_ptr<GrVkExtensions>(new GrVkExtensions());

			extensions->init(vulkan_pointer, instance, physicalDevice,
				(uint32_t)instanceExtensionNames.size(),
				instanceExtensionNames.data(),
				(uint32_t)deviceExtensionNames.size(),
				deviceExtensionNames.data());
		}
		bool Context::init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers) {
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
		bool Context::init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions) {
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
		void Context::destroy_context() {
			if (instance != VK_NULL_HANDLE)
				vkDestroyInstance(instance, nullptr);

			glfwTerminate();
		}
		VKAPI_ATTR VkBool32 VKAPI_CALL Context::debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
			//TODO: std::cout << "validation layer: " << pCallbackData->pMessage << std::endl;

			return VK_FALSE;
		}
		void Context::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
			createInfo = {};
			createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
			createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
			createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
			createInfo.pfnUserCallback = debugCallback;
		}

	}
}