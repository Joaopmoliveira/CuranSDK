#ifndef CURAN_SLIDER_PANEL_HEADER_FILE_
#define CURAN_SLIDER_PANEL_HEADER_FILE_

#include <functional>
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/ImageWrapper.h"
#include <unordered_map>
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "userinterface/widgets/IconResources.h"
#include <algorithm>
#include <vector>
#include "utils/Overloading.h"
#include "utils/SafeQueue.h"

namespace curan
{
	namespace ui
	{

		using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
		using sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;
		using clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

		enum MaskUsed
		{
			CLEAN = 0,
			DIRTY
		};

		enum Direction
		{
			X = 0,
			Y = 1,
			Z = 2
		};

		class Mask
		{
			MaskUsed _mask_flag;
			std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

		public:
			Mask() : _mask_flag{MaskUsed::CLEAN} {}
			Mask(const Mask &m) = delete;
			Mask &operator=(const Mask &) = delete;

			template <typename... T>
			std::pair<std::unordered_map<size_t, curan::ui::Stroke>::iterator, bool> try_emplace(T &&...u)
			{
				_mask_flag = MaskUsed::DIRTY;
				return recorded_strokes.try_emplace(std::forward<T>(u)...);
			}

			inline void erase(std::unordered_map<size_t, curan::ui::Stroke>::iterator it)
			{
				recorded_strokes.erase(it);
			}

			inline std::optional<curan::ui::Stroke> find(const size_t &key)
			{
				if (auto search = recorded_strokes.find(key); search != recorded_strokes.end())
					return search->second;
				else
					return std::nullopt;
			}

			void container_resized(const SkMatrix &inverse_homogenenous_transformation);

			inline operator bool() const
			{
				return _mask_flag;
			}

			std::optional<curan::ui::Stroke> draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed);
		};

		constexpr unsigned int Dimension = 3;

		struct directed_stroke
		{
			std::optional<std::array<double,3>> point; 
			Stroke stroke;
			Direction direction;
		};

		class VolumetricMask;
		using pressedhighlighted_event = std::function<void(VolumetricMask*, ConfigDraw*, const std::optional<directed_stroke>&)>;

		class VolumetricMask
		{

			static size_t counter;

			using PixelType = unsigned char;
			using ImageType = itk::Image<PixelType, Dimension>;

			std::vector<Mask> masks_x;
			std::vector<Mask> masks_y;
			std::vector<Mask> masks_z;

			ImageType::Pointer image;
		public:
			std::list<pressedhighlighted_event> callbacks_pressedhighlighted;

			VolumetricMask(ImageType::Pointer volume);

			VolumetricMask(const VolumetricMask &m) = delete;
			VolumetricMask &operator=(const VolumetricMask &) = delete;

			inline void add_pressedhighlighted_call(pressedhighlighted_event &&call)
			{
				callbacks_pressedhighlighted.emplace_back(std::move(call));
			}

			inline bool filled(){
				return image.IsNotNull();
			}

			template <typename... T>
			bool try_emplace(const Direction &direction, const float &along_dimension, T &&...u)
			{
				assert(along_dimension >= 0 && along_dimension <= 1 && "the received size is not between 0 and 1");
				switch (direction)
				{
				case Direction::X:
				{
					auto _current_index_x = std::round(along_dimension * (masks_x.size() - 1));
					auto returned_pair = masks_x[_current_index_x].try_emplace(counter, std::forward<T>(u)...);
					bool erase = true;
					if (returned_pair.second)
					{
						std::visit(curan::utilities::overloaded{[&](const Path &path)
																{
																	erase = false;
																},						  //        x                    y                          z
																[&](const Point &point) { // (along_dimension ) point.normalized_point.fX point.normalized_point.fY
																	if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
																	{
																		erase = false;
																		auto _current_index_y = std::round(point.normalized_point.fX * (masks_y.size() - 1));
																		masks_y[_current_index_y].try_emplace(counter, Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
																		auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
																		masks_z[_current_index_z].try_emplace(counter, Point{SkPoint::Make(along_dimension, point.normalized_point.fX)});
																	}
																}},
								   returned_pair.first->second);
						++counter;
					}
					if (erase)
						masks_x[_current_index_x].erase(returned_pair.first);
					return returned_pair.second;
				}
				case Direction::Y:
				{
					auto _current_index_y = std::round(along_dimension * (masks_y.size() - 1));
					auto returned_pair = masks_y[_current_index_y].try_emplace(counter, std::forward<T>(u)...);
					bool erase = true;
					if (returned_pair.second)
					{
						std::visit(curan::utilities::overloaded{[&](const Path &path)
																{
																	erase = false;
																},						  //        x                          y                          z
																[&](const Point &point) { //  point.normalized_point.fX (along_dimension )   point.normalized_point.fY
																	if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
																	{
																		erase = false;
																		auto _current_index_x = std::round(point.normalized_point.fX * (masks_x.size() - 1));
																		masks_x[_current_index_x].try_emplace(counter, Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
																		auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
																		masks_z[_current_index_z].try_emplace(counter, Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
																	}
																}},
								   returned_pair.first->second);
						++counter;
					}
					if (erase)
						masks_y[_current_index_y].erase(returned_pair.first);
					return returned_pair.second;
				}
				case Direction::Z:
				{
					auto _current_index_z = std::round(along_dimension * (masks_z.size() - 1));
					auto returned_pair = masks_z[_current_index_z].try_emplace(counter, std::forward<T>(u)...);
					bool erase = true;
					if (returned_pair.second)
					{
						std::visit(curan::utilities::overloaded{[&](const Path &path)
																{
																	erase = false;
																},						  //        x                            y                          z
																[&](const Point &point) { // point.normalized_point.fX point.normalized_point.fY (along_dimension )
																	if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
																	{
																		erase = false;
																		auto _current_index_x = std::round(point.normalized_point.fX * (masks_x.size() - 1));
																		masks_x[_current_index_x].try_emplace(counter, Point{SkPoint::Make(point.normalized_point.fY, along_dimension)});
																		auto _current_index_y = std::round(point.normalized_point.fY * (masks_y.size() - 1));
																		masks_y[_current_index_y].try_emplace(counter, Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
																	}
																}},
								   returned_pair.first->second);
						++counter;
					}
					if (erase)
						masks_z[_current_index_z].erase(returned_pair.first);
					return returned_pair.second;
				}
				};
			}

			inline void update_volume(ImageType::Pointer in_volume)
			{
				image = in_volume;
				if(!filled())
					return;
				ImageType::RegionType inputRegion = image->GetBufferedRegion();
				ImageType::SizeType size = inputRegion.GetSize();
				masks_x = std::vector<Mask>(size[Direction::X]);
				masks_y = std::vector<Mask>(size[Direction::Y]);
				masks_z = std::vector<Mask>(size[Direction::Z]);
			}

			inline ImageType::Pointer get_volume()
			{
				return image;
			}

			inline size_t dimension(const Direction &direction) const
			{
				switch (direction)
				{
				case Direction::X:
					return masks_x.size();
				case Direction::Y:
					return masks_y.size();
				case Direction::Z:
					return masks_z.size();
				default:
					return 0;
				};
			}

			template <typename... T>
			void for_each(const Direction &direction, T &&...u) const
			{
				switch (direction)
				{
				case Direction::X:
					std::for_each(masks_x.begin(), masks_x.end(), std::forward<T>(u)...);
					break;
				case Direction::Y:
					std::for_each(masks_y.begin(), masks_y.end(), std::forward<T>(u)...);
					break;
				case Direction::Z:
					std::for_each(masks_z.begin(), masks_z.end(), std::forward<T>(u)...);
					break;
				};
			}

			template <typename... T>
			void for_each(const Direction &direction, T &&...u)
			{
				switch (direction)
				{
				case Direction::X:
					std::for_each(masks_x.begin(), masks_x.end(), std::forward<T>(u)...);
					break;
				case Direction::Y:
					std::for_each(masks_y.begin(), masks_y.end(), std::forward<T>(u)...);
					break;
				case Direction::Z:
					std::for_each(masks_z.begin(), masks_z.end(), std::forward<T>(u)...);
					break;
				};
			}

			inline Mask & current_mask(const Direction &direction, const size_t &along_dimension)
			{
				assert(along_dimension >= 0 && along_dimension <= masks_x.size() - 1 && "the received size is not between 0 and 1");
				switch (direction){
				case Direction::X:
				return masks_x[along_dimension];
				case Direction::Y:
				return masks_y[along_dimension];
				case Direction::Z:
				return masks_z[along_dimension];
				};
			}
		};

		constexpr size_t size_of_slider_in_height = 30;
		constexpr size_t buffer_around_panel = 8;

		class SlidingPanel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<SlidingPanel>
		{
		public:
			enum class SliderStates
			{
				WAITING,
				PRESSED,
				HOVER,
				SCROLL,
			};

			struct image_info
			{
				std::optional<curan::ui::ImageWrapper> image;
				double width_spacing = 1;
				double height_spacing = 1;
			};

		private:
			using PixelType = unsigned char;

			using ImageType = itk::Image<PixelType, Dimension>;
			using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;

			ExtractFilterType::Pointer extract_filter;

			SkRect reserved_slider_space;
			SkRect reserved_drawing_space;
			SkRect reserved_total_space;

			SkColor colbuton = {SK_ColorRED};

			SkPaint paint_square;
			SkPaint paint_stroke;
			SkPaint background_paint;
			SkPaint paint_points;

			SkPaint highlighted_panel;

			VolumetricMask *volumetric_mask = nullptr;
			curan::ui::PointCollection current_stroke;

			SkRect background_rect;
			SkMatrix homogenenous_transformation;
			SkMatrix inverse_homogenenous_transformation;

			curan::ui::IconResources &system_icons;
			SkFont text_font;

			image_info background;

			bool is_pressed = false;
			bool is_highlighting = false;
			curan::ui::ZoomIn zoom_in;

			size_t _current_index = 0;
			float current_value = 0.5;
			float value_pressed = 0.5;
			float dragable_percent_size = 0.01f;

			SkSamplingOptions options;
			sk_sp<SkImageFilter> imgfilter;

			SkColor hover_color = SK_ColorLTGRAY;
			SkColor waiting_color = SK_ColorLTGRAY;
			SkColor click_color = SK_ColorGRAY;
			SkColor slider_color = SK_ColorGRAY;
			SkColor highlight_color = SK_ColorGRAY;

			SliderStates current_state = SliderStates::WAITING;
			Direction direction = Direction::X;
			SkPaint slider_paint;

			void query_if_required(bool force_update);

			image_info extract_slice_from_volume(size_t index);

			SlidingPanel(curan::ui::IconResources &other, VolumetricMask *mask, Direction in_direction);

			void insert_in_map(const curan::ui::PointCollection &future_stroke);

		public:
			static std::unique_ptr<SlidingPanel> make(curan::ui::IconResources &other, VolumetricMask *mask, Direction in_direction);

			~SlidingPanel();

			void compile() override;

			void update_volume(VolumetricMask *mask, Direction in_direction);

			void framebuffer_resize(const SkRect &new_page_size) override;

			inline SlidingPanel &trigger(float in_current_value)
			{
				value_pressed = in_current_value;
				return *(this);
			}

			inline float read_trigger()
			{
				return value_pressed;
			}

			inline SlidingPanel &set_current_value(float in_current_value)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				if (in_current_value < 0.0)
					in_current_value = 0.0;
				if (in_current_value > 1.0)
					in_current_value = 1.0;
				current_value = in_current_value;
				query_if_required(false);
				return *(this);
			}

			inline float get_current_value()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return current_value;
			}

			inline SkColor get_hover_color()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return hover_color;
			}

			inline SlidingPanel &set_hover_color(SkColor color)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				hover_color = color;
				return *(this);
			}

			inline SkColor get_waiting_color()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return waiting_color;
			}

			inline SlidingPanel &set_waiting_color(SkColor new_waiting_color)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				waiting_color = new_waiting_color;
				return *(this);
			}

			inline SkColor get_click_color()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return click_color;
			}

			inline SlidingPanel &set_click_color(SkColor new_click_color)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				click_color = new_click_color;
				return *(this);
			}

			inline SkColor get_slider_color()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return slider_color;
			}

			inline SlidingPanel &set_hightlight_color(SkColor new_hightlight_color)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				highlight_color = new_hightlight_color;
				return *(this);
			}

			inline SkColor get_hightlight_color()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return highlight_color;
			}

			inline SlidingPanel &set_slider_color(SkColor new_slider_color)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				slider_color = new_slider_color;
				return *(this);
			}

			inline SliderStates get_current_state()
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				return current_state;
			}

			inline SlidingPanel &set_current_state(SliderStates state)
			{
				std::lock_guard<std::mutex> g{get_mutex()};
				current_state = state;
				return *(this);
			}

			curan::ui::drawablefunction draw() override;

			curan::ui::callablefunction call() override;
		};

	}
}

#endif