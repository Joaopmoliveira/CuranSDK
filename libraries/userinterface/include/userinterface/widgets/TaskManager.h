#ifndef CURAN_TASK_MANAGER_HEADER_FILE_
#define CURAN_TASK_MANAGER_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"
#include "LightWeightPage.h"
#include "ImutableTextPanel.h"
#include "Overlay.h"
#include <memory>

namespace curan{
namespace ui{

class TaskManager final : public curan::ui::Drawable,
                          public curan::utilities::Lockable,
                          public curan::ui::SignalProcessor<TaskManager> {
public:
  enum class TaskManagerStates {
    WAITING,
    PRESSED,
    HOVER,
  };

  inline TaskManager &set_appendix(std::string in_appendix) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled) {
      font.measureText(in_appendix.data(), in_appendix.size(),
                       SkTextEncoding::kUTF8, &rect_appendix);
      text_appendix = SkTextBlob::MakeFromString(in_appendix.c_str(), font);
    } else {
      appendix = in_appendix;
      
    }
    return *(this);
  }

  inline TaskManager &set_mainbody(std::string in_mainbody) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled) {
      font.measureText(in_mainbody.data(), in_mainbody.size(),
                       SkTextEncoding::kUTF8, &rect_mainbody);
      text_mainbody = SkTextBlob::MakeFromString(in_mainbody.c_str(), font);
      previous_tracked_messages->appendtext(in_mainbody+"\n");
      
    } else {
      previous_tracked_messages->appendtext(in_mainbody+"\n");
      mainbody = in_mainbody;
    }
    return *(this);
  }

  inline TaskManager &clear_mainbody() { text_mainbody = nullptr; }

  inline TaskManager &clear_appendix() { text_appendix = nullptr; }

  inline TaskManager &clear() {
    text_appendix = nullptr;
    text_mainbody = nullptr;
  }

  inline TaskManager &set_height(size_t in_height) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    height = in_height;
    return *(this);
  }

  inline TaskManager &set_minimum_padding(size_t in_min_padding) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    min_padding = in_min_padding;
    return *(this);
  }

  inline static std::unique_ptr<TaskManager> make(const std::string &button_text) {
    std::unique_ptr<TaskManager> task =
        std::unique_ptr<TaskManager>(new TaskManager{button_text});
    return task;
  }

  void compile() override;

  ~TaskManager();

  curan::ui::drawablefunction draw() override;

  curan::ui::callablefunction call() override;

  inline TaskManager &set_font_source(sk_sp<SkTypeface> in_typeface) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    typeface = in_typeface;
    return *(this);
  }

  inline SkColor get_hover_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return hover_color;
  }

  inline TaskManager &set_hover_color(SkColor color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    hover_color = color;
    return *(this);
  }

  inline SkColor get_waiting_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return waiting_color;
  }

  inline TaskManager &set_waiting_color(SkColor new_waiting_color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    waiting_color = new_waiting_color;
    return *(this);
  }

  inline TaskManager &set_click_color(SkColor new_click_color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    click_color = new_click_color;
    return *(this);
  }

  inline SkColor get_click_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return click_color;
  }

  inline TaskManagerStates get_current_state() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return current_state;
  }

  inline TaskManager &set_current_state(TaskManagerStates state) {
    std::lock_guard<std::mutex> g{get_mutex()};
    current_state = state;
    return *(this);
  }

  void framebuffer_resize(const SkRect &new_page_size) override;

private:
  TaskManager(const std::string &in_predix);

  TaskManager(const TaskManager &other) = delete;

  std::unique_ptr<curan::ui::Overlay> create_overlay_with_previous_messages();

  SkColor hover_color;
  SkColor waiting_color;
  SkColor click_color;
  SkColor text_color;
  SkPaint paint;
  SkPaint paint_text;
  sk_sp<SkTypeface> typeface;
  size_t height = 30;
  size_t min_padding = 30;
  SkFont font;

  std::string predix;
  SkRect rect_predix;
  sk_sp<SkTextBlob> text_predix;
  std::string mainbody;
  SkRect rect_mainbody;
  sk_sp<SkTextBlob> text_mainbody;
  std::string appendix;
  SkRect rect_appendix;
  sk_sp<SkTextBlob> text_appendix;

  TaskManagerStates current_state = TaskManagerStates::WAITING;
  bool compiled = false;
  curan::ui::SignalInterpreter interpreter;
  curan::ui::ImutableTextPanel* previous_tracked_messages = nullptr;
  std::shared_ptr<curan::ui::LightWeightPage> overlay_info;
};

}
}

#endif