#ifndef DkLibrary_h_DEFINED
#define DkLibrary_h_DEFINED

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
#include "PkLibrary.h"
#include "CkLibrary.h"

namespace curan
{
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
		The Widget class is the parent class of all widgets
		that can be displayed on the screen. All widgets
		should override the implementation of the draw and
		callback methods. These are called to change the 
		aperance of the widget on the screen draw the actual 
		pixels unto the screen.
		*/
		class Widget {
			SkRect widget_position = SkRect::MakeWH(1,1);
			Page* parent;
		public:
			Widget();
			virtual void draw(SkCanvas* canvas, SkRect& widget_rect);
			virtual void callback(Signal signal,bool* interacted);
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
*/
				VERTICAL,
				/*
				*/
				HORIZONTAL,
				/*
				*/
				VARIABLE
			};


			Layout(SkPaint in_paint_layout, Arrangement in_arrangment);
			virtual void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size);

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
		The BackbufferInfo structure is used by skia
		to syncronize data submited to the GPU associated
		with a given swapchain image.
		*/
		struct BackbufferInfo {
			uint32_t        fImageIndex;      
			VkSemaphore     fRenderSemaphore;
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

		/*
		The OptionBox class is a widget which should be
		integrated with other widgets which require user
		feedback which is too complex to be contained in
		a small number of buttons, thus this visual element hovers
		over other components and should be directly connected to it.
		To be flexible its construction should be similar
		to a page (but with fixed width and height).
		*/
		class OptionBox : public Widget {
			SkPaint path_paint;
			std::shared_ptr<Page> contained_page;
			bool movable = false;
			SkPoint origin = SkPoint{0,0};
			SkSize option_box_size = {0,0};
		public:

			struct Info {
				std::shared_ptr<Page> page_to_draw;
				bool movable = false;
				SkSize option_box_size;
			};

			OptionBox(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<OptionBox> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			void change_page(std::shared_ptr<Page> page);

		};

		/*
		An Icon is an image which can be used by widgets to
		be displayed in their allocated area. It contains an
		SkImage which can be used to display on screen. 
		*/
		struct Icon
		{
			int texWidth = 0;
			int texHeight = 0;
			int texChannels = 0;
			stbi_uc* pixels = nullptr;
			SkPixmap pixmap;
			sk_sp<SkImage> image_to_display;

			static Icon read(const char* s);
		};

		struct IconResources
		{
			static IconResources* Get();
			static IconResources* Load(std::string path_to_resources);
			void GetIcon(sk_sp<SkImage>& image, std::string icon_string);
		private:
			static bool is_initialized;
			std::map<std::string, Icon> icon_map;
			IconResources(std::string path_to_resources);
		};

		/*
		The Buffering structure contains a circular ring composed
		of 16 points equally divided which can rotate. Its
		intended use is to provide feedback to the user once an
		operation is taking place in the background. Its a simple
		helper class for other widgets.
		*/
		struct Bufferring {
			SkPaint paint;
			SkColor color;
			std::vector<SkPoint> points;
			double rotation = 0.0;
			std::chrono::time_point<std::chrono::steady_clock> previous_timepoint = std::chrono::steady_clock::now();
			mutable std::mutex mut;

			Bufferring(double radius);

			void draw(SkCanvas* canvas, SkPoint center);

			void resize(double new_radius);
		};

		/*
		The Button class is one of the main widgets which can be used in the 
		application. It can display an image, it has states, depending on user 
		input and it contains signals which can be sent to slots associated with 
		the button when the constructor of the class is called.
		*/
		class Button : public Widget {
			/*
			Possible states that a button finds itself in.
			Either the button is currently PRESSED (the mouse
			device and sent a signal saying that the user whishs to
			interact with this specific widget), it can be in the
			HOVER state (the user has their mouse over the widget)
			and the button uses this information to provide some
			kind of feedback.
			*/
			enum class ButtonStates {
				WAITING,
				PRESSED,
				HOVER,
			};

		protected:
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkRect size;
			SkFont text_font;
			sk_sp<SkTextBlob> text;
			sk_sp<SkImage> icon_data;
			ButtonStates current_state = ButtonStates::WAITING;

		public:
			struct Info {
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkPaint paintButton;
				SkPaint paintText;
				SkFont textFont;
				SkRect size;
				std::string button_text;
				std::string icon_identifier;
			};

			Button(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<Button> make(Info& info);
			void callback(Signal signal, bool* interacted) override;


			sigslot::signal<MousePressSignal> signal_click;
			sigslot::signal<ItemDroppedSignal> signal_dropped;
		};

		/*
		* The ThreeWayButton class is a widget which basically contains possible states 
		* that a given portion of the application can circle through. As an example
		* image that you have a specific sequence of states that must be followed and you want the user to control when
		* to change between states. Lets call it A<->B<->C<->D
		* 
		* When on the first state the user can only move to the state B thus,
		*  <  >  <     >  < B >
		* 
		* After pressing B the callbacks associated with the button are called with information about either 
		* the left (in this case empty) or right button have been clicked. In the next state we should see
		* 
		* < A >  <   B   > < C >
		* 
		* And the user can procced advancing through the states as the application requires.
		*/
		class ThreeWayButton : public Widget
		{
			std::vector<std::string> string_vector;
		protected:

			struct DrawableButtonData
			{
				sk_sp<SkTextBlob> text;
				SkRect area;
				SkRect widget_rect_text;
				SkPaint paint_background;
			};

			/*
			Possible buttons that can appear on screen
			*/
			std::vector<DrawableButtonData> button_vector_stuf;
			/*
			Buttons which are currently displayed on screen
			*/
			std::array<DrawableButtonData, 3> drawable_stuf;

			SkColor hover_color_side;
			SkColor waiting_color_side;
			SkColor click_color_side;

			SkColor color_center;
			SkPaint paint_text;

			SkFont text_font;

			int index = 0;
			SkRect total_size;
			SkRect left_button_size;
			SkRect center_button_size;
			SkRect right_button_size;
			mutable std::mutex mut;

		public:
			struct Info {
				std::vector<std::string> stages_array;
				SkRect size;
				SkFont font;
				SkColor hover_color_side;
				SkColor waiting_color_side;
				SkColor click_color_side;
				SkColor color_center;
			};

			enum ButtonPressed {
				LEFT,
				RIGHT,
			};

			ThreeWayButton(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ThreeWayButton> make(Info& info);
			void callback(Signal signal, bool* interacted) override;

			sigslot::signal<MousePressSignal, ButtonPressed> signal_click;

		private:
			bool area_interacts(SkRect area, SkScalar x_pos, SkScalar y_pos);
		};

		/*
		The RadioButton class is a widget which
		*/
		class RadioButton : public Widget
		{
		protected:

			struct RadioItem {
				SkRect normalized_position = SkRect::MakeXYWH(0, 0, 1, 1);
				SkRect real_item_position = SkRect::MakeXYWH(0, 0, 1, 1);
				SkRect text_size = SkRect::MakeXYWH(0, 0, 1, 1);
				sk_sp<SkTextBlob> text;
				bool is_selected = false;
			};

			std::vector<RadioItem> radio_components;
			SkColor color;
			SkColor background_color;
			SkPaint paint;
			SkPaint paint_text;
			SkRect widget_rect_text;
			SkFont text_font;
			SkRect size;
			bool is_exclusive = false;
			int current_selected_index = -1;
			mutable std::mutex mut;

		public:

			enum RadioButtonLayout {
				VERTICAL,
				HORIZONTAL
			};
		
			struct Info {
				RadioButtonLayout layout;
				std::vector<std::string> options;
				SkFont text_font;
				SkColor color;
				SkColor background_color;
				SkRect size;
				bool is_exclusive = false;
			};

			RadioButton(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<RadioButton> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
		};

		/*
		The CommunicationDisplay class is a widget which shows 
		information recevied through the OpenIGTLink protocol. 
		This class can be submited to a functor (read the 
		documentation of the communication namespace) which
		will provide information when a message is received. 
		This in turn will be shown on the display area of the
		widget.
		*/
		class CommunicationDisplay : public Widget
		{

		public:
			struct Info {
				SkFont text_font;
			};

			enum MessageType {
				TRANSFORM,
				IMAGE,
			};

			/*
			Each message has some information which must be associated with it.
			The sksurface property is only ocupied by the Image types
			*/
			class MessageInfo {

				MessageType type;
				SkImageInfo information;
				sk_sp<SkImage> trial;
				struct {
					igtl::TransformMessage::Pointer transform;
					igtl::ImageMessage::Pointer image;
				} pointer;

			public:

				//
				MessageInfo(const MessageType type);
				~MessageInfo();

				std::chrono::time_point<std::chrono::steady_clock> timestamp;
				double frequency = 0.0;
				std::string message_type;
				std::string message_devide_name;

				void setTransform(igtl::TransformMessage::Pointer);
				void setImage(igtl::ImageMessage::Pointer);
				void createSurface(igtl::ImageMessage::Pointer);

				bool getTransform(igtl::TransformMessage::Pointer& p);
				bool getImage(igtl::ImageMessage::Pointer& p, sk_sp<SkImage>& surface);

				MessageType getType();
			};

		protected:
			SkFont text_font;
			SkPaint paint;
			SkPaint paint_text;
			std::atomic<curan::communication::ProcessorOpenIGTLink::Status> current_status;
			std::vector<sk_sp<SkTextBlob>> table_headers;

			/*
			The table conversion map contains a conversion
			between the string associated with the message
			received and an internal representation of
			the supported messages.
			*/
			std::map<std::string, MessageInfo> table_conversion;
			bool in_debug_mode = false;

			SkPoint center_debug_mode = { 0.0f,0.0f };
			float debug_mode_radius = 10.0f;
			sk_sp<SkTextBlob> debug_glyph;

			MousePressSignal last_pressed_position = { -1.0 , -1.0 };

			//std::vector<std::string> transform_string;

			mutable std::mutex mut;
		public:

			CommunicationDisplay(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<CommunicationDisplay> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			void process_message(igtl::MessageBase::Pointer pointer);
			void connection_status(curan::communication::ProcessorOpenIGTLink::Status status);
		};


		/*
		The StaticImageDisplay class is a widget which 
		can be used to show an image which can be updated
		but it cannot be moved thought user input. 
		This is a simpler version of a MovableImageDisplay 
		widget.
		*/
		class StaticImageDisplay : public Widget {
			sk_sp<SkImage> image_to_display;
			SkAlphaType alpha_channel;
			SkColorType color_type;
			SkPaint paint;
			std::mutex mut;
			SkSamplingOptions options;
		public:
			struct Info {
				SkAlphaType alpha_channel;
				SkColorType color_type;
				sk_sp<SkImage> initial_image;
				SkColor waiting_color = SK_ColorYELLOW;
				SkColor hover_color = SK_ColorRED;
			};

			StaticImageDisplay(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<StaticImageDisplay> make(Info& info);
			void update_image(sk_sp<SkImage> new_image);
		};

		/*
		A primitive is a drawable entity, unto a DrawPane. This is usefull when one wishes to add drawing behaviour to a given widget, without the need to equip the widget with custom code
		*/
		class Primitive
		{
			virtual void draw(SkCanvas* canvas);
		};

		/*
		The drawPane class is a instance which allows the user to draw over its area. This means that we decople the drawing from the underlying other drawings already present in the canvas.
		The drawPane should allow the most possible arrangements of drawings, it should probably contain information about the current path which is being
		drawn and the ability to change the path which one wishes to alter or redraw. 
		*/
		class DrawPane
		{
			struct PrimitivesInfo {
				SkPath path_contained;
				std::string path_name;
				bool show_path = false;
				bool highlighted = false;
				SkColor path_color;
			};

			enum PathInteration {
				HIDE_PRIMITIVE,
				CHANGE_PRIMITIVE,
				ELIMINATE_PRIMITIVE,
				INACTIVE_PRIMITIVE
			};

			std::list<SkPoint> current_path_list;
			std::list<PrimitivesInfo> paths_contained;

			std::list<Primitive> primitives_drawn;

			std::mutex mut;
		public:
			void draw(SkCanvas* canvas, SkRect& widget_rect);
			void callback(Signal signal, bool* interacted);
		};


		/*
		The MovableImageDisplay class is a widget displays an
		image on the screen. This widget is farly complex, in 
		the sence that is prepared to receive different types 
		of interactions from the user. It is also prepared to
		display curves over the image that the user can interact
		with. See the samples related with this widget to 
		understand its capabilities.
		*/
		class MovableImageDisplay : public Widget //this also need to be changed, the drawing dynamic should not de internal to the 
		{

			struct PathsInfo{
				SkPath path_contained;
				std::string path_name;
				bool show_path = false;
				bool highlighted = false;
				SkColor path_color;
			};

			enum PathInteration {
				HIDE_PATH,
				CHANGE_PATH,
				ELIMINATE_PATH,
				ADD_PATH,
				INACTIVE
			};

			SkMatrix current_transform;
			sk_sp<SkImage> image_to_display;
			SkAlphaType alpha_channel;
			SkColorType color_type;
			SkColor hover_color;
			SkColor waiting_color;
			SkPaint paint;
			SkSamplingOptions options;

			SkPaint path_paint;
			SkColor path_color;
			double current_hsv_value_h = 0.0;

			PathInteration current_path_interaction = PathInteration::INACTIVE;
			std::list<SkPoint> current_path_list;
			std::list<PathsInfo> paths_contained;
			bool path_display_active = false;
			std::atomic<bool> purge_resources = false;
			std::mutex mut;

			void update_color();

		public:

			struct Info {
				SkAlphaType alpha_channel;
				SkColorType color_type;
				sk_sp<SkImage> initial_image;
				SkColor waiting_color = SK_ColorYELLOW;
				SkColor hover_color = SK_ColorRED;
			};

			MovableImageDisplay(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			void callback(Signal signal, bool* interacted) override;
			static std::shared_ptr<MovableImageDisplay> make(Info& info);
			void rotate(SkScalar delta_theta, SkScalar pivot_x, SkScalar pivot_y);
			void translate(SkScalar delta_x, SkScalar delta_y);
			void zoom(SkScalar scale_factor, SkScalar pivot_x, SkScalar pivot_y);
			void update_image(sk_sp<SkImage> new_image);
			void hover(bool is_hovering);

			void display_paths();
			void show_all_paths();
			void hide_path();
			void new_path();
			void change_path();
			void submit_path();
			void eliminate_path();
		};



		/*
		The ImageDisplayDinamicLayout class is a widget which 
		contains a structured MovableImageDisplay widgets. This 
		allows the user to change the number of images being 
		shown and it allows for user interaction, both to 
		rotate, zoom, translate the images, but also to draw
		over the images for different purposes, i.e. manual 
		segmentation, etc..
		*/
		class ImageDisplayDinamicLayout : public Widget //this needs to be changed, i dont like that there is a class so complex
		{
		public:

			enum InteractionMode {
				CLICK,
				ROTATE,
				ZOOM,
				PAN
			};

			enum LayoutState {
				ONE_BY_ONE,
				ONE_BY_TWO,
				ONE_BY_THREE,
				TWO_BY_THREE,
			};


		protected:

			struct Tile {
				std::shared_ptr<Layout> layout;
				std::vector<std::shared_ptr<MovableImageDisplay>> contained_views;
			};

			std::vector<Tile> contained_tiles;
			curan::image::Study m_study;
			MouseMoveSignal current_mouse_position = { -1.0 , -1.0 };
			MousePressSignal last_pressed_position = { -1.0 , -1.0 };
			std::atomic<InteractionMode> interaction = InteractionMode::PAN;
			LayoutState current_layout = LayoutState::ONE_BY_ONE;

			std::shared_ptr<OptionBox> optional_option_box = nullptr;
			std::atomic<bool> show_optional_box = false;
		public:

			struct Info {
				SkAlphaType alpha_channel;
				SkColorType color_type;
				LayoutState layout;
				InteractionMode initial_interaction_mode;
				curan::image::Study vol;
				std::shared_ptr<OptionBox> option_box = nullptr;
			};

			ImageDisplayDinamicLayout(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ImageDisplayDinamicLayout> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			void pan_mode(MousePressSignal);
			void rotate_mode(MousePressSignal);
			void click_mode(MousePressSignal);
			void zoom_mode(MousePressSignal);
			void change_option_box_presentation_state(MousePressSignal);
			void change_layout(LayoutState new_layout);

			void display_paths(MousePressSignal);
			void show_all_paths(MousePressSignal);
			void hide_path(MousePressSignal);
			void change_path(MousePressSignal);
			void submit_path(MousePressSignal);
			void eliminate_path(MousePressSignal);
			void new_path(MousePressSignal);
		};

		/*
		The ItemPreview class is a widget that displays
		the current dicom images which have been imported into 
		the application. This widget can be used whenever the 
		developer whishes to provide the user with the tools 
		to change between images, between different registrations,
		etc..
		*/
		class ItemPreview : public Widget //the item preview makes sense, but we need to provide a feedback mechanism, so that listeners can be warned whenever the user selects a given image
		{
		public:
			sigslot::signal<int> preview_selected_call;

			struct Info {
				SkColor color_background_left;
				SkColor color_hover;
				SkColor color_selected;
				SkColor color_waiting;
				SkColor color_background_right;
				SkPaint text_paint;
				SkFont font;
			};

			struct Item {
				sk_sp<SkImage> image;
				SkRect current_pos;
				bool is_selected = false;
				sk_sp<SkTextBlob> text;
			};

		protected:

			std::list<int> current_selected_identifiers;

			int selected_image_identifier = 0;
			SkRect preview_rectangle;
			bool is_exclusive = true;

			SkPaint paint_image_background;
			SkPaint paint_background;
			SkPaint text_paint;
			SkFont font;

			SkColor color_background;
			
			SkColor color_waiting;
			SkColor color_hover;
			SkColor color_selected;

			SkScalar vertical_scroll = 0.0;

			MouseMoveSignal current_mouse_position = { -1.0 , -1.0 };

			std::atomic<float> maximum_height{ 0 };
			std::atomic<float> current_height_offset{ 0 };

			std::map<int, Item> item_list;

		public:

			ItemPreview(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ItemPreview> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			
			/*
			* The method returns the current index which is selected by the user
			*/
			int getselectedindex(std::list<int>& list_selected_indexes);
			
			/*
			* The method updates the current listing by adding a new item to the listing
			*/
			bool update(Item item_to_add,int identifier);

			/*
			* The remove method removes a listing from an external providir
			*/
			void remove(int identifier);
		};

		/*
		The ThreadPollStatusDisplay class is a widget that 
		displays the current background tasks and allows 
		the user to obtain information about any potential 
		problems.
		*/
		class ThreadPollStatusDisplay : public Widget //<not used but it is important
		{
		protected:
			SkPaint background;
			SkColor color_background;
			SkPaint text_paint;
			SkColor color_text;
			SkFont font;
		public:
			struct Info {
				SkFont font;
				SkColor color_background;
				SkColor color_text;
			};

			ThreadPollStatusDisplay(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ThreadPollStatusDisplay> make(Info& info);
		};

		/*
		The ManipulatorPosition class is a widget 
		*/
		class ManipulatorPosition : public Widget //this is stupid, the robot itself should do the job by itself
		{
		protected:

			struct ManipulatorTextRanges {
				sk_sp<SkTextBlob> left_range;
				SkRect left_range_size;
				sk_sp<SkTextBlob> right_range;
				SkRect right_range_size;
				sk_sp<SkTextBlob> joint_name;
				SkRect center_joint_size;
			};

			SkPaint background;
			SkColor color_background;
			SkColor color_ranges;
			SkPaint text_paint;
			SkColor color_text;
			SkFont font;
			std::vector<ManipulatorTextRanges> range_text;
			std::vector<SkRect> rectangle;

		public:
			struct Info {
				std::vector<std::tuple<int, int>> ranges;
				std::vector<std::string> joint_names;
				SkColor color_background;
				SkColor color_ranges;
				SkColor color_text;
				SkFont font;
			};

			ManipulatorPosition(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ManipulatorPosition> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
		private:

		};

		/*
		The SystemValidation class is a widget that 
		*/
		class SystemValidation : public Widget  //TODO:: requires changing - dont know a suitable name
		{
			std::vector<std::shared_ptr<StaticImageDisplay>> data;
			std::tuple<SkColor, SkColor> path_validation_colors;
			SkPaint path_paint;
			curan::communication::ProcessorOpenIGTLink::Status current_status = curan::communication::ProcessorOpenIGTLink::Status::CK_CLOSED;
			Bufferring buffering = Bufferring{ 30 };

		public:
			struct Info {
				std::tuple<SkColor, SkColor> path_validation_colors;
				std::vector<sk_sp<SkImage>> icons_of_devices;
			};

			SystemValidation(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<SystemValidation> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			void update_connection_status(curan::communication::ProcessorOpenIGTLink::Status status);
		private:
		};

		/*
		*/
		enum TasksEnumeration {
			MEDICAL_VIEWER,
			BIOPSY_VIEWER,
		};

		/*
		DkTask is a struct with contains information about the possible
		tasks which the application can perform. It has a name, i.e. Biopsy viewer,
		a small description about the task at hand and an icon identifier which should
		correspond to the
		*/
		struct Task
		{
			std::string name;
			std::string description;
			std::string icon_identifier;
			TasksEnumeration task_type;
		};


		/*
		The task previewer is an enumeration of the possible
		procedures that the viewer currently implements.
		At the moment there are two types of tasks that the viewer should
		implement, namelly:

		1 - The medical viewer

		2 - The Biopsy viewer

		*/
		class TaskPreviewer : public Widget //TODO:: requires changing - I would choose ItemSelector
		{
			struct InternalTaskData {
				sk_sp<SkImage> icon;
				Task task_data;
				sk_sp<SkTextBlob> name;
				sk_sp<SkTextBlob> description;
			};

		protected:
			std::vector<InternalTaskData> list_of_tasks;
			SkPaint background;
			SkPaint text_paint;
			SkColor hover_color;
			SkColor waiting_color;
			SkColor click_color;
			SkColor text_color;
			SkFont font;
			SkRect relative_dimensions;
			SkRect widget_real_position;
			SkScalar vertical_offset = 0.0f;

			MouseMoveSignal current_mouse_position = { -1.0 , -1.0 };
			MousePressSignal last_pressed_position = { -1.0 , -1.0 };

			int current_index = -1;
		public:

			struct Info {
				std::vector<Task> supported_tasks;
				SkColor hover_color;
				SkColor waiting_color;
				SkColor click_color;
				SkColor text_color;
				SkFont font;
				SkRect relative_dimensions;
			};

			TaskPreviewer(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<TaskPreviewer> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			bool check_interaction(double xpos, double ypos);
		};




		/*
		* The GLFW library is a c library, thus it cannot deal
		* with callbacks in C++ style. Thus the cursos_position_callback
		* free function is created. This function is submited
		* to GLFW library for a given window and internally it
		* creates a signal which is compatible with our code base.
		* The signal is then appended to the windows signal queue,
		* to be processed at a latter point in time.
		*/
		void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);

		/*
		* The GLFW library is a c library, thus it cannot deal
		* with callbacks in C++ style. Thus the cursor_position_click_callback
		* free function is created. This function is submited
		* to GLFW library for a given window and internally it
		* creates a signal which is compatible with our code base.
		* The signal is then appended to the windows signal queue,
		* to be processed at a latter point in time.
		*/
		void cursor_position_click_callback(GLFWwindow* window, int button, int action, int mods);

		/*
		* The GLFW library is a c library, thus it cannot deal
		* with callbacks in C++ style. Thus the scroll_callback
		* free function is created. This function is submited
		* to GLFW library for a given window and internally it
		* creates a signal which is compatible with our code base.
		* The signal is then appended to the windows signal queue,
		* to be processed at a latter point in time.
		*/
		void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

		/*
		* The GLFW library is a c library, thus it cannot deal
		* with callbacks in C++ style. Thus the item_droped_callback
		* free function is created. This function is submited
		* to GLFW library for a given window and internally it
		* creates a signal which is compatible with our code base.
		* The signal is then appended to the windows signal queue,
		* to be processed at a latter point in time.
		*/
		void item_droped_callback(GLFWwindow* window, int count, const char** paths);
	}
}
#endif

