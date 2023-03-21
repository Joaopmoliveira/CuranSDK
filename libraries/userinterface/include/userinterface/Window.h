#ifndef CURAN_WINDOW_HEADER_FILE_
#define CURAN_WINDOW_HEADER_FILE_

#include "widgets/Signal.h"
#include <memory>
#include "utils/SafeQueue.h"
#include "Context.h"

namespace curan {
	namespace ui {
		struct DisplayParams {
			DisplayParams(std::unique_ptr<Context> cxt, int width, int height)
				: fColorType(kN32_SkColorType)
				, fColorSpace(nullptr)
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
		private:
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
			DisplayParams params;
			VkSwapchainKHR swapChain = VK_NULL_HANDLE;
			std::vector<VkImage> swapChainImages;
			std::vector<VkImageLayout> swapChainImageLayout;
			VkFormat swapChainImageFormat;
			VkExtent2D swapChainExtent;
			VkImageUsageFlags usageFlags;
			size_t currentFrame = 0;
			bool framebufferResized = false;
			std::string windowName;
		public:
			GLFWwindow* window = nullptr;
			curan::utils::SafeQueue<Signal> signal_queue;

			Window(DisplayParams&& pars);

			~Window();

			[[nodiscard]] bool swapBuffers();
			[[nodiscard]] SkSurface* getBackbufferSurface();
			void connect_handler();
			[[nodiscard]] bool initialize();
			void destroy();

			inline SkRect get_size() {
				SkRect rec = SkRect::MakeXYWH(0,0,width,height);
				return rec;
			}

			std::vector<Signal> process_pending_signals();
			[[nodiscard]] bool recreateDisplay();
			BackbufferInfo* getAvailableBackBuffer();
			static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
		};
	}
}

#endif