namespace curan {
	namespace display {
		/*
The Context class is responsible for initializing the GLFW library,
so that surface compatilibity can be queried and to initialize the
vulkan instance, the debug callbacks and selecting an appropriate
physical device compatible with the GPU api used, currently only
vulkan is supported. When the Context is eliminated the GLFW library
is also terminated.
*/
		class Context {
			VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;

			/*
			The output operator should transform into text all the information related with
			the context used in the execution of the program. This allows us to debug certain
			features of the execution enviorment which were used to lauch the application.
			*/
			friend std::ostream& operator<<(std::ostream& out, const Context& o);

		public:
			VkInstance instance = VK_NULL_HANDLE;
			VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
			GrVkExtensions* extensions = nullptr;
			std::vector<const char*> deviceLayerNames;
			std::vector<const char*> deviceExtensionNames;
			QueueFamilyIndices indices;
			GrVkGetProc vulkan_pointer = VK_NULL_HANDLE;

			Context();

			~Context();

			bool initialize_context();

			bool init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers);

			bool init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions);

			void destroy_context();

			static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData);

			void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
		};
	}
}