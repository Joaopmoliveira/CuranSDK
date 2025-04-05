#include "userinterface/widgets/TaskManager.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/definitions/Interactive.h"

namespace curan {
namespace ui {

void TaskManager::compile() {
  std::lock_guard<std::mutex> g{get_mutex()};
  font = SkFont(typeface, height * 0.6, 1.0f, 0.0f);
  font.setEdging(SkFont::Edging::kAntiAlias);
  font.measureText(predix.data(), predix.size(), SkTextEncoding::kUTF8,
                   &rect_predix);
  if (predix.size() > 0)
    text_predix = SkTextBlob::MakeFromString(predix.c_str(), font);
  if (appendix.size() > 0)
    text_appendix = SkTextBlob::MakeFromString(appendix.c_str(), font);
  if (mainbody.size() > 0)
    text_mainbody = SkTextBlob::MakeFromString(mainbody.c_str(), font);
  compiled = true;
}

TaskManager::~TaskManager() {}

curan::ui::drawablefunction TaskManager::draw() {
  if (!compiled)
    throw std::runtime_error(
        "must compile the button before drawing operations");
  auto lamb = [this](SkCanvas *canvas) {
    switch (current_state) {
    case TaskManagerStates::WAITING:
      paint.setColor(get_waiting_color());
      break;
    case TaskManagerStates::HOVER:
      paint.setColor(get_hover_color());
      break;
    case TaskManagerStates::PRESSED:
      paint.setColor(get_click_color());
      break;
    }
    SkRect drawable = get_position();
    canvas->drawRect(drawable, paint);
    float text_offset_x = drawable.x() + min_padding;
    float text_offset_y = drawable.centerY() + rect_predix.height() / 2.0f;

    float requested_width = rect_predix.width() + rect_mainbody.width() +
                            rect_appendix.width() + 4.0f * min_padding;
    float slack = (drawable.width() - requested_width) / 2.0f;
    if (slack < 0.0) {
      canvas->drawTextBlob(text_predix, text_offset_x, text_offset_y,
                           paint_text);
      text_offset_x += rect_predix.width() + min_padding;
      canvas->drawTextBlob(text_mainbody, text_offset_x, text_offset_y,
                           paint_text);
      text_offset_x += rect_mainbody.width() + min_padding;
      canvas->drawTextBlob(text_appendix, text_offset_x, text_offset_y,
                           paint_text);
    } else {
      canvas->drawTextBlob(text_predix, text_offset_x, text_offset_y,
                           paint_text);
      text_offset_x += rect_predix.width() + slack;
      canvas->drawTextBlob(text_mainbody, text_offset_x, text_offset_y,
                           paint_text);
      text_offset_x += rect_mainbody.width() + slack;
      canvas->drawTextBlob(text_appendix, text_offset_x, text_offset_y,
                           paint_text);
    }
  };
  return lamb;
}

std::unique_ptr<curan::ui::Overlay> TaskManager::create_overlay_with_previous_messages() {
  using namespace curan::ui;
  auto slidercontainer =
      Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                      Container::Arrangement::UNDEFINED);
  auto tracked_messages = curan::ui::ImutableTextPanel::make("information");
  tracked_messages->set_background_color({1.f, 1.0f, 1.0f, 1.0f})
      .set_text_color({.0f, .0f, .0f, 1.0f});
  previous_tracked_messages = tracked_messages.get();
  *slidercontainer << std::move(tracked_messages);
  slidercontainer->set_color(SK_ColorTRANSPARENT);
  slidercontainer->set_variable_layout({SkRect::MakeLTRB(0.2, 0.2, 0.8, 0.8)});
  return Overlay::make(std::move(slidercontainer), SK_ColorTRANSPARENT, false);
}

curan::ui::callablefunction TaskManager::call() {
  if (!compiled)
    throw std::runtime_error(
        "must compile the button before drawing operations");
  auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config) {
    auto check_inside_fixed_area = [this](double x, double y) {
      auto widget_rect = get_position();
      SkRect drawable = SkRect::MakeXYWH(widget_rect.x(), widget_rect.y(),
                                         widget_rect.width(), height);
      return drawable.contains(x, y);
    };

    interpreter.process(check_inside_fixed_area, check_inside_fixed_area, sig);

    if (interpreter.check(curan::ui::INSIDE_FIXED_AREA |
                          curan::ui::MOUSE_CLICKED_LEFT_EVENT)) {
      config->stack_page->stack(overlay_info);
      return true;
    }

    if (interpreter.check(curan::ui::INSIDE_FIXED_AREA |
                          curan::ui::MOUSE_CLICKED_LEFT)) {
      set_current_state(TaskManagerStates::PRESSED);
      return true;
    }

    if (interpreter.check(curan::ui::LEFT_ALLOCATED_AREA_EVENT)) {
      set_current_state(TaskManagerStates::WAITING);
      return false;
    }

    if (interpreter.check(curan::ui::INSIDE_FIXED_AREA)) {
      set_current_state(TaskManagerStates::HOVER);
      return false;
    }

    return false;
  };
  return lamb;
}

void TaskManager::framebuffer_resize(const SkRect &new_page_size) {
  std::lock_guard<std::mutex> g{get_mutex()};
  if (!compiled) {
    throw std::runtime_error(
        "cannot query positions while container not compiled");
  }
  auto my_position = get_position();
  set_size(SkRect::MakeWH(my_position.width(), height));
}

TaskManager::TaskManager(const std::string &in_predix) : predix{in_predix} {
  hover_color = SkColorSetARGB(0xFF, 0, 136, 208);
  waiting_color = SkColorSetARGB(0xFF, 0, 166, 238);
  click_color = SkColorSetARGB(0xFF, 0, 91, 191);
  text_color = SK_ColorWHITE;

  paint.setStyle(SkPaint::kFill_Style);
  paint.setAntiAlias(true);
  paint.setStrokeWidth(4);
  paint.setColor(hover_color);

  paint_text.setStyle(SkPaint::kFill_Style);
  paint_text.setAntiAlias(true);
  paint_text.setStrokeWidth(4);
  paint_text.setColor(text_color);

  typeface = curan::ui::defaultTypeface();
  overlay_info = create_overlay_with_previous_messages()->take_ownership();
}

} // namespace ui
} // namespace curan