#ifndef CURAN_CONTEXT_HEADER_FILE_
#define CURAN_CONTEXT_HEADER_FILE_

#include <vector>
#include <memory>
#include <optional>
#include "widgets/definitions/UIdefinitions.h"

namespace curan {
namespace ui {

/*
Because we want to use SKIA, 
we need to manage the VULKAN context directly, 
which is bothersome but a necessity if 
we want to manipulate things in real time.

Vulkan works through explicitly creating an Instance, 
which defines an entry point to manipulate the driver 
which communicates with the GPU. 
Once this entry point
is defined we can select the queues to which we will 
communicate and submit our rendering request to the GPU, 
e.g., graphics and present family. 

Althoug it's possible
to create multiple windows that are managed by a single context, 
we were not able to create the necessary guarantees 
which make sure that we have sycronous acess to resources.
The context allocates the necessary things uppon creation and destruction, 
following RAII principles. 
*/

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