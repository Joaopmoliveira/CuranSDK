#ifndef CURAN_CONTEXT_HEADER_FILE_
#define CURAN_CONTEXT_HEADER_FILE_

#include <vector>
#include <memory>
#include <optional>
#include "UIdefinitions.h"

namespace curan {
	namespace ui {

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
			std::unique_ptr<GrVkExtensions> extensions = nullptr;
			std::vector<const char*> deviceLayerNames;
			std::vector<const char*> deviceExtensionNames;
			QueueFamilyIndices indices;
			GrVkGetProc vulkan_pointer = VK_NULL_HANDLE;

			Context();
			~Context();

		private:

			bool initialize_context();
			bool init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers);
			bool init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions);
			void destroy_context();
			static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData);
			void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
		};

	}
}

#endif