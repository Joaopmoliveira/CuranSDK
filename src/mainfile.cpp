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

			Signal() : Signal(Signal::Type::EMPTY_SIGNAL, Data())
			{

			}

			Signal(Type in_type, Data in_data)
			{
				signal_type = in_type;
				data = in_data;
			}
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

		class normalizedcoordinates {
			double x;
			double y;
		};

		class pixelcoordinates {
			double x;
			double y;
		};


		template<typename T1,typename T2>
		void convertCoordinates(const T1& source, T2& destination, double scalllingx,double scallingy)
		{
			T2.x = T1.x* scalllingx;
			T2.y = T1.y* scalllingy;
		};

		class Page;

		/*
		Drawable is something that can be displayed on screen. If 
		it can be displayed, i.e. think layout, widgets, then it 
		must implement two important methods, i.e. how should it 
		react to envirorment changes (user interaction). If some interaction
		does happen, then drawable has an internal pointer to the page
		where it is being drawen in and it tells the parent that the 
		page has been dirtied and it needs to be redrawn. The second
		method that it must implement is the draw method.
		*/
		class Drawable {
			std::shared_ptr<Page> rendering_scene =  nullptr;
			SkRect drawabe_area; //pixel coordinates

		public:
			/*
			* The user_interaction function should be implemented in all children class.
			* It takes the signal received from the enviorment and depending on the implementation
			* it can change the information provided to the Parent, to let him known that the scene 
			* has changed and it needs to be updated. There should always be two kinds of warnings
			* The scene needs to be repainted (therefore all the sizes stay the same) or the scene has been
			* resized, therefore we need to recompute the dimensions of the pixelcoordinates for the 
			* widgets, layouts, etc...
			*/
			virtual void user_interaction(Signal signal) = 0;
			/*
			* The draw method implemented by the children classes should take the pixel coordiantes 
			* where the widget,layout,etc... should be placed and it prints itself unto the SKCanvas 
			*/
			virtual void draw() = 0;
		};

		/*
The Widget class is the parent class of all widgets
that can be displayed on the screen. All widgets
should override the implementation of the draw and
callback methods. These are called to change the
aperance of the widget on the screen draw the actual
pixels unto the screen.
*/
		class Widget : public Drawable
		{
		private:
			Page* parent;
			SkRect widget_position = SkRect::MakeWH(1, 1);
			
		public:
			Widget()
			{
			};

			virtual void draw(SkCanvas* canvas, SkRect& widget_rect) {
			
			}

			virtual void callback(Signal signal, bool* interacted) {
			}

			bool interacts(SkScalar x, SkScalar y) {
				return ((widget_position.fRight >= x) &&
					(widget_position.fLeft <= x) &&
					(widget_position.fBottom >= y) &&
					(widget_position.fTop <= y));
			}
			void set_position(SkRect& rect) {
				widget_position = rect;
			}
			void get_position(SkRect& rect) {
				rect = widget_position;
			}
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
		class Layout : public Drawable {
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

			LayoutLinearContainer(Info& info) : Layout(info.paint_layout, info.arrangement)
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
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override
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
			static std::shared_ptr<LayoutLinearContainer> make(Info& info)
			{
				return std::make_shared<LayoutLinearContainer>(info);
			}


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

			LayoutVariableContainer(Info& info) : Layout(info.paint_layout, info.arrangement)
			{
				contained_layouts = info.layouts;
				rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
			}
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override
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
			static std::shared_ptr<LayoutVariableContainer> make(Info& info)
			{
				return std::make_shared<LayoutVariableContainer>(info);
			}


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

			LayoutLinearWidgetContainer(Info& info) : Layout(info.paintLayout, info.arrangement)
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
			void draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override
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

			static std::shared_ptr<LayoutLinearWidgetContainer> make(Info& info)
			{
				return std::make_shared<LayoutLinearWidgetContainer>(info);
			}

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

			LayoutVariableWidgetContainer(Info& info) : Layout(info.paintLayout, info.arrangement)
			{
				contained_widgets = info.widgets;
				rectangles_of_contained_layouts = info.rectangles_of_contained_layouts;
			}
			void LayoutVariableWidgetContainer::draw(SkCanvas* canvas_to_draw, SkRect& drawing_area, SkRect& window_size) override
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
			static std::shared_ptr<LayoutVariableWidgetContainer> make(Info& info)
			{
				return std::make_shared<LayoutVariableWidgetContainer>(info);
			}
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

			Context() {
			}

			~Context() {
			}

			bool initialize_context()
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
			bool init_instance_extensions_and_layers(std::vector<VkExtensionProperties>& instanceExtensions, std::vector<VkLayerProperties>& instanceLayers)
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

			bool init_device_extensions_and_layers(std::vector<VkLayerProperties>& deviceLayers, std::vector<VkExtensionProperties>& deviceExtensions)
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
				
			void destroy_context()
			{
				if (extensions != nullptr)
					delete extensions;
				if (instance != VK_NULL_HANDLE)
					vkDestroyInstance(instance, nullptr);

				glfwTerminate();
			}
			static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData)
			{
				std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

				return VK_FALSE;
			}
				
				void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo)
				{
					createInfo = {};
					createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
					createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
					createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
					createInfo.pfnUserCallback = debugCallback;
				}
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

			/*
			This should output the layout of the page unto a common 
			file logic. In case we ever automatize the generation of
			the pages, it might be nice to be able to read them from
			a static file.
			*/
			friend std::ostream& operator<<(std::ostream& out, const Page& o);

		public:

			struct Info {
				SkColor paint_page;
				std::shared_ptr<Layout> page_layout;
				init_callback initialize_page_callback;
			};

			Page(Info& info)
			{
				paint_page = info.paint_page;
				page_layout = info.page_layout;
				intialization_callback = info.initialize_page_callback;
			}
			void draw(SkCanvas* canvas_to_draw, SkRect& window_size)
			{
				SkSurface* surf = canvas_to_draw->getSurface();
				SkRect total_size = SkRect::MakeIWH(surf->width(), surf->height());
				SkRect range = SkRect::MakeLTRB(window_size.fLeft / surf->width(), window_size.fTop / surf->height(), window_size.fRight / surf->width(), window_size.fBottom / surf->height());
				page_layout->draw(canvas_to_draw, range, total_size);
			}
			static std::shared_ptr<Page> make(Info& info)
			{
				return std::make_shared<Page>(info);
			}
			void callback(Signal signal, bool* interacted)
			{
				for (auto widg : widgets_to_callback)
					widg->callback(signal, interacted);
			}
			void monitor(std::shared_ptr<Widget> widg)
			{
				widgets_to_callback.push_back(widg);
			}
			void started()
			{
				if (intialization_callback)
					intialization_callback();
			}
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

			Window(DisplayParams& pars)
			{
				params = pars;
			};
			Window()
			{};

			void swapBuffers()
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

				GrBackendSemaphore beSemaphore;
				beSemaphore.initVulkan(semaphore);

				surface->wait(1, &beSemaphore);

				return surface;
			}
			void add_page(std::shared_ptr<Page> page)
			{
				contained_pages.push_back(page);
			}
			void draw(SkCanvas* canvas, int page_index)
			{
				static int previews_page_index = 0;
				SkRect rect = SkRect::MakeLTRB(0, 0, swapChainExtent.width, swapChainExtent.height);
				if (previews_page_index != page_index) {
					contained_pages[page_index]->started();
					previews_page_index = page_index;
				}
				contained_pages[page_index]->draw(canvas, rect);
			}


			void connect_handler() {
				glfwSetCursorPosCallback(this->window, cursor_position_callback);
				glfwSetMouseButtonCallback(this->window, cursor_position_click_callback);
				glfwSetScrollCallback(this->window, scroll_callback);
				glfwSetDropCallback(this->window, item_droped_callback);
			}

			bool initialize(Context* in_context)
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
			void destroy()
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
			void process_pending_signals(int page_index)
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
		private:

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
			BackbufferInfo* getAvailableBackBuffer()
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
			static void framebufferResizeCallback(GLFWwindow* window, int width, int height)
			{
				auto app = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
				app->framebufferResized = true;
			}

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

			RadioButton(Info& info)
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

				switch (info.layout) {
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
			void draw(SkCanvas* canvas, SkRect& widget_rect) override
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
					RadioItem* item = radio_components.data() + index;

					SkRect temp = SkRect::MakeXYWH(sized_rectangle.width() * item->normalized_position.x() + sized_rectangle.x(),
						sized_rectangle.height() * item->normalized_position.y() + sized_rectangle.y(),
						sized_rectangle.width() * item->normalized_position.width(),
						sized_rectangle.height() * item->normalized_position.height());

					item->real_item_position = SkRect::MakeXYWH(temp.fLeft + DK_RADIO_BUTTON_DIMENSION,
						temp.centerY() - (DK_RADIO_BUTTON_DIMENSION) / 2.0,
						DK_RADIO_BUTTON_DIMENSION,
						DK_RADIO_BUTTON_DIMENSION);

					canvas->drawRect(item->real_item_position, paint);

					if (item->is_selected) {
						canvas->drawLine({ item->real_item_position.fLeft,item->real_item_position.fTop }, { item->real_item_position.fRight,item->real_item_position.fBottom }, paint);
						canvas->drawLine({ item->real_item_position.fRight,item->real_item_position.fTop }, { item->real_item_position.fLeft,item->real_item_position.fBottom }, paint);
					}

					canvas->drawTextBlob(item->text, item->real_item_position.fRight + DK_RADIO_BUTTON_DIMENSION, item->real_item_position.centerY() + item->text_size.height() / 2, paint_text);
				}
			}
			static std::shared_ptr<RadioButton> make(Info& info)
			{
				return std::make_shared<RadioButton>(info);
			}
			void callback(Signal signal, bool* interacted) override
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
							if (current_selected_index >= 0 && current_selected_index != index && is_exclusive) {
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
		};

	}
}

int main() {
	std::shared_ptr<curan::display::Page> page;
	SkColor colbuton = { SK_ColorWHITE };
	SkColor coltext = { SK_ColorBLACK };

	SkPaint paint_square;
	paint_square.setStyle(SkPaint::kFill_Style);
	paint_square.setAntiAlias(true);
	paint_square.setStrokeWidth(4);
	paint_square.setColor(colbuton);

	SkPaint paint_text;
	paint_text.setStyle(SkPaint::kFill_Style);
	paint_text.setAntiAlias(true);
	paint_text.setStrokeWidth(4);
	paint_text.setColor(coltext);

	const char* fontFamily = nullptr;
	SkFontStyle fontStyle;
	sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

	SkFont text_font = SkFont(typeface, curan::display::DK_DEFAULT_TEXT_SIZE, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	curan::display::RadioButton::Info info_radio;
	info_radio.background_color = SK_ColorBLACK;
	info_radio.color = SK_ColorWHITE;
	info_radio.is_exclusive = false;
	info_radio.options = { "Option 1","Option 2" };
	info_radio.layout = curan::display::RadioButton::RadioButtonLayout::VERTICAL;
	info_radio.size = SkRect::MakeXYWH(0, 0, 200, 200);
	info_radio.text_font = text_font;

	std::shared_ptr<curan::display::RadioButton> radio = curan::display::RadioButton::make(info_radio);

	curan::display::LayoutLinearWidgetContainer::Info info1;
	info1.arrangement = curan::display::Layout::Arrangement::VERTICAL;
	info1.paintLayout = paint_square2;
	info1.widgets = { radio };
	std::shared_ptr<curan::display::LayoutLinearWidgetContainer> button_container = curan::display::LayoutLinearWidgetContainer::make(info1);

	curan::display::Page::Info info5;
	info5.page_layout = std::static_pointer_cast<curan::display::Layout>(button_container);
	info5.paint_page = SK_ColorWHITE;
	page = curan::display::Page::make(info5);

	page->monitor(std::static_pointer_cast<curan::display::Widget>(radio));
	curan::display::Context context;
	context.initialize_context();

	curan::display::DisplayParams param;
	curan::display::Window* viewer = new curan::display::Window{ param };
	viewer->initialize(&context);
	viewer->add_page(page);
	viewer->connect_handler();
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		viewer->draw(canvas, 0);
		//if (overlay_page_index > 0) {
		//	auto image = canvas->getSurface()->makeImageSnapshot();
		//	canvas->drawImage(image, 0, 0, SkSamplingOptions(), &overlay_paint);
		//}
		viewer->swapBuffers();
		viewer->process_pending_signals(0);
		glfwPollEvents();
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	viewer->destroy();
	context.destroy_context();
	return 0;
}