#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImutableTextPanel.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "utils/TheadPool.h"
#include <iostream>
#include <thread>

#include "itkAffineTransform.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkScaleTransform.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"


std::atomic<bool> value_to_trigger = false;

class TaskTracker final : public curan::ui::Drawable,
                          public curan::utilities::Lockable,
                          public curan::ui::SignalProcessor<TaskTracker> {
public:
  enum class TaskTrackerStates {
    WAITING,
    PRESSED,
    HOVER,
  };

  TaskTracker &set_appendix(std::string in_appendix) {
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

  TaskTracker &set_mainbody(std::string in_mainbody) {
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

  TaskTracker &clear_mainbody() { text_mainbody = nullptr; }

  TaskTracker &clear_appendix() { text_appendix = nullptr; }

  TaskTracker &clear() {
    text_appendix = nullptr;
    text_mainbody = nullptr;
  }

  TaskTracker &set_height(size_t in_height) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    height = in_height;
    return *(this);
  }

  TaskTracker &set_minimum_padding(size_t in_min_padding) {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    min_padding = in_min_padding;
    return *(this);
  }

  static std::unique_ptr<TaskTracker> make(const std::string &button_text) {
    std::unique_ptr<TaskTracker> task =
        std::unique_ptr<TaskTracker>(new TaskTracker{button_text});
    return task;
  }

  void compile() override {
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

  ~TaskTracker() {}

  curan::ui::drawablefunction draw() override {
    if (!compiled)
      throw std::runtime_error(
          "must compile the button before drawing operations");
    auto lamb = [this](SkCanvas *canvas) {
      switch (current_state) {
      case TaskTrackerStates::WAITING:
        paint.setColor(get_waiting_color());
        break;
      case TaskTrackerStates::HOVER:
        paint.setColor(get_hover_color());
        break;
      case TaskTrackerStates::PRESSED:
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

  std::unique_ptr<curan::ui::Overlay> create_overlay_with_previous_messages() {
    using namespace curan::ui;
    auto slidercontainer = Container::make(Container::ContainerType::VARIABLE_CONTAINER,Container::Arrangement::UNDEFINED);
    auto tracked_messages = curan::ui::ImutableTextPanel::make("information");
    tracked_messages->set_background_color({1.f,1.0f,1.0f,1.0f}).set_text_color({.0f,.0f,.0f,1.0f});
    previous_tracked_messages = tracked_messages.get();
    *slidercontainer << std::move(tracked_messages);
    slidercontainer->set_color(SK_ColorTRANSPARENT);
    slidercontainer->set_variable_layout({SkRect::MakeLTRB(0.2,0.2,0.8,0.8)});
    return Overlay::make(std::move(slidercontainer), SK_ColorTRANSPARENT, false);
  }

  curan::ui::callablefunction call() override {
    if (!compiled)
      throw std::runtime_error("must compile the button before drawing operations");
    auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config) {
      auto check_inside_fixed_area = [this](double x, double y) {
        auto widget_rect = get_position();
        SkRect drawable = SkRect::MakeXYWH(widget_rect.x(), widget_rect.y(),
                                           widget_rect.width(), height);
        return drawable.contains(x, y);
      };

      interpreter.process(check_inside_fixed_area, check_inside_fixed_area,
                          sig);

      if (interpreter.check(curan::ui::INSIDE_FIXED_AREA |
                            curan::ui::MOUSE_CLICKED_LEFT_EVENT)) {
        config->stack_page->stack(overlay_info);
        return true;
      }

      if (interpreter.check(curan::ui::INSIDE_FIXED_AREA |
                            curan::ui::MOUSE_CLICKED_LEFT)) {
        set_current_state(TaskTrackerStates::PRESSED);
        return true;
      }

      if (interpreter.check(curan::ui::LEFT_ALLOCATED_AREA_EVENT)) {
        set_current_state(TaskTrackerStates::WAITING);
        return false;
      }

      if (interpreter.check(curan::ui::INSIDE_FIXED_AREA)) {
        set_current_state(TaskTrackerStates::HOVER);
        return false;
      }

      return false;
    };
    return lamb;
  }

  inline TaskTracker &set_font_source(sk_sp<SkTypeface> in_typeface) {
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

  inline TaskTracker &set_hover_color(SkColor color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    hover_color = color;
    return *(this);
  }

  inline SkColor get_waiting_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return waiting_color;
  }

  inline TaskTracker &set_waiting_color(SkColor new_waiting_color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    waiting_color = new_waiting_color;
    return *(this);
  }

  inline TaskTracker &set_click_color(SkColor new_click_color) {
    std::lock_guard<std::mutex> g{get_mutex()};
    click_color = new_click_color;
    return *(this);
  }

  inline SkColor get_click_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return click_color;
  }

  inline TaskTrackerStates get_current_state() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return current_state;
  }

  inline TaskTracker &set_current_state(TaskTrackerStates state) {
    std::lock_guard<std::mutex> g{get_mutex()};
    current_state = state;
    return *(this);
  }

  void framebuffer_resize(const SkRect &new_page_size) override {
    std::lock_guard<std::mutex> g{get_mutex()};
    if (!compiled) {
      throw std::runtime_error(
          "cannot query positions while container not compiled");
    }
    auto my_position = get_position();
    set_size(SkRect::MakeWH(my_position.width(), height));
  }

private:
  TaskTracker(const std::string &in_predix) : predix{in_predix} {
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

  TaskTracker(const TaskTracker &other) = delete;

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

  TaskTrackerStates current_state = TaskTrackerStates::WAITING;
  bool compiled = false;
  curan::ui::SignalInterpreter interpreter;
  curan::ui::ImutableTextPanel* previous_tracked_messages = nullptr;
  std::shared_ptr<curan::ui::LightWeightPage> overlay_info;
};

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

int main() {
  try {
    using namespace curan::ui;
    IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
    std::unique_ptr<Context> context = std::make_unique<Context>();
    DisplayParams param{std::move(context), 2200, 1800};
    std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    using ImageReaderType = itk::ImageFileReader<itk::Image<double, 3>>;

    std::printf("\nReading input volume...\n");
    auto fixedImageReader = ImageReaderType::New();
    fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH
                                  "/precious_phantom/precious_phantom.mha");

    // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale =
        itk::RescaleIntensityImageFilter<itk::Image<double, 3>,
                                         itk::Image<double, 3>>::New();
    rescale->SetInput(fixedImageReader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(255.0);

    auto castfilter =
        itk::CastImageFilter<itk::Image<double, 3>, ImageType>::New();
    castfilter->SetInput(rescale->GetOutput());
    castfilter->Update();

    curan::ui::VolumetricMask vol{castfilter->GetOutput()};

    std::unique_ptr<SlidingPanel> image_display_x =
        SlidingPanel::make(resources, &vol, Direction::X);
    std::unique_ptr<SlidingPanel> image_display_y =
        SlidingPanel::make(resources, &vol, Direction::Y);
    std::unique_ptr<SlidingPanel> image_display_z =
        SlidingPanel::make(resources, &vol, Direction::Z);

    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                     Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display_x) << std::move(image_display_y)
               << std::move(image_display_z);

    auto tasktracker = TaskTracker::make("ultrasound:");
    auto ptr_tasktracker = tasktracker.get();
    tasktracker->set_height(40).set_appendix("100%").set_mainbody("processing data...");

    auto pool = curan::utilities::ThreadPool::create(1);
    std::atomic<bool> keep_running = true;
    pool->submit("doing hard work",[&](){
      int work = 0;
      while(keep_running){
        work += 1.0;
        if(work>100.0)
          work = 0.0;
        ptr_tasktracker->set_appendix(std::to_string(work)+" %");
        ptr_tasktracker->set_mainbody("working " + std::to_string(work+10) + " at velocity: ");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
      
    });

    auto container_with_task =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::VERTICAL);
    *container_with_task << std::move(container) << std::move(tasktracker);
    container_with_task->set_divisions({0.0, 0.98, 1.0});

    auto page = Page{std::move(container_with_task), SK_ColorBLACK};
    page.update_page(viewer.get());

    ConfigDraw config_draw{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
      auto start = std::chrono::high_resolution_clock::now();
      SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
      SkCanvas *canvas = pointer_to_surface->getCanvas();
      if (viewer->was_updated()) {
        page.update_page(viewer.get());
        viewer->update_processed();
      }
      page.draw(canvas);
      auto signals = viewer->process_pending_signals();
      for (const auto &sig : signals)
        page.propagate_signal(sig, &config_draw);

      glfwPollEvents();

      bool val = viewer->swapBuffers();
      if (!val)
        std::cout << "failed to swap buffers\n";
      auto end = std::chrono::high_resolution_clock::now();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(16) -
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
    }
    keep_running = false;
    return 0;
  } catch (std::exception &e) {
    std::cout << "Failed" << e.what() << std::endl;
    return 1;
  }
}