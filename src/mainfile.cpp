#define GLFW_INCLUDE_VULKAN

#include <GLFW/glfw3.h>

#include <sigslot/signal.hpp>

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <chrono>
#include <cstring>
#include <string>
#include <cstdlib>
#include <cstdint>
#include <optional>
#include <set>
#include <vulkan/vulkan.h>
#include <filesystem>
#include <limits>

#define SK_VULKAN
#include "include\core\SkBitmap.h"
#include "include\core\SkBlendMode.h"
#include "include\core\SkBlurTypes.h"
#include "include\core\SkCanvas.h"
#include "include\core\SkCanvasVirtualEnforcer.h"
#include "include\core\SkColor.h"
#include "include\core\SkColorFilter.h"
#include "include\core\SkEncodedImageFormat.h"
#include "include\core\SkFont.h"
#include "include\core\SkFontArguments.h"
#include "include\core\SkFontMetrics.h"
#include "include\core\SkFontMgr.h"
#include "include\core\SkFontParameters.h"
#include "include\core\SkFontStyle.h"
#include "include\core\SkFontTypes.h"
#include "include\core\SkGraphics.h"
#include "include\core\SkImage.h"
#include "include\core\SkImageEncoder.h"
#include "include\core\SkImageFilter.h"
#include "include\core\SkImageGenerator.h"
#include "include\core\SkImageInfo.h"
#include "include\core\SkM44.h"
#include "include\core\SkMallocPixelRef.h"
#include "include\core\SkMaskFilter.h"
#include "include\core\SkMath.h"
#include "include\core\SkMatrix.h"
#include "include\core\SkMilestone.h"
#include "include\core\SkOverdrawCanvas.h"
#include "include\core\SkPaint.h"
#include "include\core\SkPath.h"
#include "include\core\SkPathBuilder.h"
#include "include\core\SkPathEffect.h"
#include "include\core\SkPathMeasure.h"
#include "include\core\SkPicture.h"
#include "include\core\SkPictureRecorder.h"
#include "include\core\SkPixelRef.h"
#include "include\core\SkPixmap.h"
#include "include\core\SkPngChunkReader.h"
#include "include\core\SkPoint.h"
#include "include\core\SkPoint3.h"
#include "include\core\SkPromiseImageTexture.h"
#include "include\core\SkRRect.h"
#include "include\core\SkRSXform.h"
#include "include\core\SkRasterHandleAllocator.h"
#include "include\core\SkRect.h"
#include "include\core\SkRefCnt.h"
#include "include\core\SkRegion.h"
#include "include\core\SkScalar.h"
#include "include\core\SkSerialProcs.h"
#include "include\core\SkShader.h"
#include "include\core\SkSize.h"
#include "include\core\SkStream.h"
#include "include\core\SkString.h"
#include "include\core\SkStrokeRec.h"
#include "include\core\SkSurface.h"
#include "include\core\SkSurfaceCharacterization.h"
#include "include\core\SkSurfaceProps.h"
#include "include\core\SkSwizzle.h"
#include "include\core\SkTextBlob.h"
#include "include\core\SkTime.h"
#include "include\core\SkTraceMemoryDump.h"
#include "include\core\SkTypeface.h"
#include "include\core\SkTypes.h"
#include "include\core\SkUnPreMultiply.h"
#include "include\core\SkVertices.h"
#include "include\core\SkYUVAInfo.h"
#include "include\core\SkYUVAPixmaps.h"
#include "include\docs\SkPDFDocument.h"
#include "include\docs\SkXPSDocument.h"
#include "include\effects\Sk1DPathEffect.h"
#include "include\effects\Sk2DPathEffect.h"
#include "include\effects\SkBlurMaskFilter.h"
#include "include\effects\SkColorMatrix.h"
#include "include\effects\SkColorMatrixFilter.h"
#include "include\effects\SkCornerPathEffect.h"
#include "include\effects\SkDashPathEffect.h"
#include "include\effects\SkDiscretePathEffect.h"
#include "include\effects\SkGradientShader.h"
#include "include\effects\SkHighContrastFilter.h"
#include "include\effects\SkImageFilters.h"
#include "include\effects\SkOverdrawColorFilter.h"
#include "include\effects\SkRuntimeEffect.h"
#include "include\effects\SkShaderMaskFilter.h"
#include "include\effects\SkTableColorFilter.h"
#include "include\effects\SkTableMaskFilter.h"
#include "include\encode\SkJpegEncoder.h"
#include "include\gpu\GrBackendDrawableInfo.h"
#include "include\gpu\GrBackendSemaphore.h"
#include "include\gpu\GrBackendSurface.h"
#include "include\gpu\GrBackendSurfaceMutableState.h"
#include "include\gpu\GrConfig.h"
#include "include\gpu\GrContextOptions.h"
#include "include\gpu\GrContextThreadSafeProxy.h"
#include "include\gpu\GrDirectContext.h"
#include "include\gpu\GrDriverBugWorkarounds.h"
#include "include\gpu\GrRecordingContext.h"
#include "include\gpu\GrTypes.h"
#include "include\gpu\GrYUVABackendTextures.h"
#include "include\gpu\vk\GrVkBackendContext.h"
#include "include\gpu\vk\GrVkExtensions.h"
#include "include\gpu\vk\GrVkMemoryAllocator.h"
#include "include\gpu\vk\GrVkTypes.h"
#include "include\gpu\vk\GrVkVulkan.h"
#include "include\pathops\SkPathOps.h"
#include "include\ports\SkTypeface_win.h"

#include "DkUtilities.h"

namespace curan {
	namespace display {

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

#ifdef NDEBUG
		const bool enableValidationLayers = false;
#else
		const bool enableValidationLayers = true;
#endif

		/*
		The QueueFamilyIndices is used to select the
		graphics and present pipeline used to communicate
		with the GPU of the machine. Internally the struct
		contains an index which corresponds to the queue
		used to send information to the graphics portions
		of the physical device and the queue compatible
		with the presentation queue used by the operating
		system.
		*/
		struct QueueFamilyIndices {
			std::optional<uint32_t> graphicsFamily;
			std::optional<uint32_t> presentFamily;

			bool isComplete() {
				return graphicsFamily.has_value() && presentFamily.has_value();
			}
		};

		/*
		The display parameters are used by skia to
		achieve sub pixel acuracy. Read the skia
		tutorials to understand the implications
		of this structucture.
		*/
		struct DisplayParams {
			DisplayParams()
				: fColorType(kN32_SkColorType)
				, fColorSpace(nullptr)
				, fMSAASampleCount(1)
				, fSurfaceProps(0, kRGB_H_SkPixelGeometry)
				, fDisableVsync(false)
				, fDelayDrawableAcquisition(false)
				, fEnableBinaryArchive(false)
			{}

			SkColorType         fColorType;
			sk_sp<SkColorSpace> fColorSpace;
			int                 fMSAASampleCount;
			GrContextOptions    fGrContextOptions;
			SkSurfaceProps      fSurfaceProps;
			bool                fDisableVsync;
			bool                fDelayDrawableAcquisition;
			bool                fEnableBinaryArchive;
		};

		constexpr auto DK_DEFAULT_TEXT_SIZE = 15.0;
		constexpr auto DK_DEFAULT_FRAME_DELAY = std::chrono::milliseconds(15);
		constexpr SkScalar DK_VOLUME_PREVIEW_HEIGHT = 200.0f;
		constexpr SkScalar DK_VOLUME_PREVIEW_WIDTH = 150.0f;
		constexpr SkScalar DK_VOLUME_IMAGE_HEIGHT = 170.0f;
		constexpr SkScalar DK_VOLUME_PREVIEW_EXTREMA_SPACING_COLUNM = 20.f;
		constexpr SkScalar DK_VOLUME_PREVIEW_EXTREMA_SPACING_LINE = 20.0f;
		constexpr int NUMBER_BYTES_PER_PIXEL = 4;
		constexpr SkScalar DK_TASK_PREVIEW_HEIGHT = 150.0f;
		constexpr SkScalar DK_MANIPULATOR_BAR_HEIGHT = 50;
		constexpr SkScalar DK_RADIO_BUTTON_DIMENSION = 20;

		/*
		The MouseMoveSignal contains the position
		of the cursor in relation to the window
		of the GLFW library.
		*/
		struct MouseMoveSignal
		{
			double xpos;
			double ypos;
		};

		/*
		The MousePressSignal contains the position
		of the cursor in relation to the window
		of the GLFW library.
		*/
		struct MousePressSignal
		{
			double xpos;
			double ypos;
			//Eigen::Vector2d position = { 0.0, 0.0 };
		};

		/*
		The ScrollSignal contains the position
		of the cursor in relation to the window
		of the GLFW library and the scroll introduced by
		the user with the peripheral device.
		*/
		struct ScrollSignal
		{
			double xpos;
			double ypos;
			double xoffset;
			double yoffset;
		};

		/*
		The MouseUnpressSignal contains the position
		of the cursor in relation to the window of
		the GLFW library when the user releases the
		mouse peripheral.
		*/
		struct MouseUnpressSignal
		{
			double xpos;
			double ypos;
		};

		/*
		The ItemDropppedSignal contains the information of
		the clip contents provided by the operating system when
		the user drags and drops any item into the window.
		*/
		struct ItemDroppedSignal
		{
			int count;
			char** paths;
		};


		/*
		A signal contains an enumeration of the possible types
		of the contained signaland an union of all information
		related with each signal. The developer should enumerate
		over the possible signal types and extract information
		accordingly. It is important to note that accessing information
		which does not correspond to the type of signal is undefined
		behaviour which will crash a program
		*/
		struct Signal
		{
			union Data
			{
				MouseMoveSignal move_signal;
				MousePressSignal press_signal;
				ScrollSignal scroll_signal;
				MouseUnpressSignal unpress_signal;
				ItemDroppedSignal dropped_signal;
			};

			enum class Type {
				MOUSE_MOVE_SIGNAL,
				MOUSE_PRESS_SIGNAL,
				SCROLL_SIGNAL,
				MOUSE_UNPRESS_SIGNAL,
				ITEM_DROPED_SIGNAL,
				EMPTY_SIGNAL,
				TERMINATE_SIGNAL
			};

			Data data;
			Type signal_type;

			Signal();

			Signal(Type type, Data data);
		};

		/*
		Callback called when a page is shown the first time on screen.
		This is usefull when the developer whishes to start a background
		task without requiring the user to start a proccess.
		*/
		using init_callback = std::function<void(void)>;

		/*
The BackbufferInfo structure is used by skia
to syncronize data submited to the GPU associated
with a given swapchain image.
*/
		struct BackbufferInfo {
			uint32_t        fImageIndex;
			VkSemaphore     fRenderSemaphore;
		};

		/*
The Widget class is the parent class of all widgets
that can be displayed on the screen. All widgets
should override the implementation of the draw and
callback methods. These are called to change the
aperance of the widget on the screen draw the actual
pixels unto the screen.
*/
		class Widget {
			SkRect widget_position = SkRect::MakeWH(1, 1);
			Page* parent;
		public:
			Widget();
			virtual void draw(SkCanvas* canvas, SkRect& widget_rect);
			virtual void callback(Signal signal, bool* interacted);
			bool interacts(SkScalar x, SkScalar y);
			void set_position(SkRect& rect);
			void get_position(SkRect& rect);
		};

		/*
		The Layout class is the parent class that wrapps
		how visual objects are layout in the drawing
		surface. The coordinates of layouts are always
		normalized. This in turn means that any object
		contained in the layout will scale according
		to the size of the window. The children of the Layout
		can show different behaviours on how they are created
		depending on the specified Arrangment
		*/
		class Layout {
		public:

			enum Arrangement {
				/*
				* This tells the size computation unit that when resizing
				windows the vertical dimension needs to be recomputed
				*/
				VERTICAL,
				/*
				*/
				HORIZONTAL,
				/*
				*/
				VARIABLE
			};


			Layout(SkPaint in_paint_layout, Arrangement in_arrangment)
			{
				paint_layout = in_paint_layout;
				arrangment = in_arrangment;
			}

			virtual void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size)
			{
				SkRect rec = SkRect::MakeLTRB(drawing_area.fLeft * window_size.fLeft,
					drawing_area.fTop * window_size.fTop,
					drawing_area.fRight * window_size.fRight,
					drawing_area.fBottom * window_size.fBottom);
				canvas_to_draw->drawRect(rec, paint_layout);
			}

		protected:
			SkPaint paint_layout;
			Arrangement arrangment;
		};

		/*
		The LayoutLinearContainer class is a specific type of layout
		which takes the contained layouts and it generates the
		normalized coordinates of the layouts by itself, either
		vertically or horizontally depending on the arrangment
		specified by the developer. If they wish, they can specify
		the spacing between the contained layouts. This class allows
		nesting layouts inside layouts, thus providing flexiblility
		to the developer.
		*/
		class LayoutLinearContainer : public Layout
		{
		public:
			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Layout>> layouts;
				std::vector<SkScalar> divisions;
				SkPaint paint_layout;
			};

			LayoutLinearContainer(Info& info);
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override;
			static std::shared_ptr<LayoutLinearContainer> make(Info& info);


		protected:
			std::vector<std::shared_ptr<Layout>> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;

		};

		/*
		The LayoutVariableContainer class is a specific type of layout
		which requires the developer to specify the normalized coordinates
		of each layout contained inside it. This allows the developer to
		specify non-orthodox arranjments of layout. If they wish, they can specify
		the spacing between the contained layouts. This class allows
		nesting layouts inside layouts, thus providing flexiblility
		to the developer.
		*/
		class LayoutVariableContainer : public Layout
		{
		public:
			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Layout>> layouts;
				std::vector<SkRect> rectangles_of_contained_layouts;
				SkPaint paint_layout;
			};

			LayoutVariableContainer(Info& info);
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override;
			static std::shared_ptr<LayoutVariableContainer> make(Info& info);


		protected:
			std::vector<std::shared_ptr<Layout>> contained_layouts;
			std::vector<SkRect> rectangles_of_contained_layouts;

		};

		/*
		The LayoutLinearWidgetContainer class is a specific type of layout
		which takes the contained widgets and it generates the
		normalized coordinates of the widgets by itself, either
		vertically or horizontally depending on the arrangment
		specified by the developer. If they wish, they can specify
		the spacing between the contained widgets.
		*/
		class LayoutLinearWidgetContainer : public Layout
		{
		public:
			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Widget>> widgets;
				std::vector<SkScalar> divisions;
				SkPaint paintLayout;
			};

			LayoutLinearWidgetContainer(Info& info);
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override;
			static std::shared_ptr<LayoutLinearWidgetContainer> make(Info& info);

		protected:
			std::vector<std::shared_ptr<Widget>> contained_widgets;
			std::vector<SkRect> rectangles_of_contained_layouts;

		};

		/*
		The LayoutVariableWidgetContainer class is a specific type of layout
		which requires the developer to specify the normalized coordinates
		of each widget contained inside it. This allows the developer to
		specify non-orthodox arranjments of widgets.
		*/
		class LayoutVariableWidgetContainer : public Layout
		{
		protected:
			std::vector<std::shared_ptr<Widget>> contained_widgets;
			std::vector<SkRect> rectangles_of_contained_layouts;
		public:
			struct Info {
				Arrangement arrangement;
				std::vector<std::shared_ptr<Widget>> widgets;
				std::vector<SkRect> rectangles_of_contained_layouts;
				SkPaint paintLayout;
			};

			LayoutVariableWidgetContainer(Info& info);
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override;
			static std::shared_ptr<LayoutVariableWidgetContainer> make(Info& info);
		};

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

		/*
The Page class is specific in its name. It represents a portion
of a surface which the developer can draw on. The page has two
jobs, to propagate the signals to each widget, so that the widget
can decide if it wishes to deal with said signal and to position
in space the layouts which in turn they contain widgets.
*/
		class Page {
		protected:
			SkColor paint_page;
			std::atomic<bool> is_dirty;
			std::shared_ptr<Layout> page_layout;
			std::vector<std::shared_ptr<Widget>> widgets_to_callback;
			init_callback intialization_callback;
		public:

			struct Info {
				SkColor paint_page;
				std::shared_ptr<Layout> page_layout;
				init_callback initialize_page_callback;
			};

			Page(Info& info);
			void draw(SkCanvas* canvas_to_draw, SkRect& window_region);
			static std::shared_ptr<Page> make(Info& info);
			void callback(Signal signal, bool* interacted);
			void monitor(std::shared_ptr<Widget> widg);
			void started();
		};

		/*
	The Window class allows the developer to obtain a
	surface, draw on it and replace this surface to be
	shown on screen at a latter point in time. Internally
	the Window class contains pages which it can display.
	*/
		class Window {
		private:
			std::vector<std::shared_ptr<Page>> contained_pages;

		public:
			GLFWwindow* window = nullptr;
			curan::utils::ThreadSafeQueue<Signal> signal_queue;

			Window(DisplayParams& pars);
			~Window();

			void swapBuffers();
			SkSurface* getBackbufferSurface();
			void add_page(std::shared_ptr<Page> page);
			void draw(SkCanvas* canvas, int page_index);
			void connect_handler();
			bool initialize(Context* in_context);
			void destroy();
			void process_pending_signals(int page_index);
		private:

			bool recreateDisplay();
			BackbufferInfo* getAvailableBackBuffer();
			static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

			Context* context = nullptr;
			VkSurfaceKHR surface = VK_NULL_HANDLE;
			VkDevice device = VK_NULL_HANDLE;
			VkQueue graphicsQueue = VK_NULL_HANDLE;
			VkQueue presentQueue = VK_NULL_HANDLE;
			GrDirectContext* skia_context = nullptr;
			BackbufferInfo* fBackbuffers = nullptr;
			GrVkBackendContext* vkContext = nullptr;
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
		};
	}
}


int main() {
	
	return 0;
}