#include "userinterface/widgets/Signal.h"

void signal_tutorial() {
  using namespace curan::ui;
  using namespace curan::utilities;
  {
    Signal sig{Empty{}};
    std::visit(overloaded{[&](Empty arg) {
                            std::printf("empty is contained inside signal\n");
                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }

  {
    Signal sig{ItemDropped{2, {"path/to/file", {"path/to/file2"}}}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {
                            std::printf(
                                "item dropped is contained inside signal\n");
                          }},
               sig);
  }

  {
    Signal sig{Key{GLFW_KEY_E, glfwGetKeyScancode(GLFW_KEY_E), GLFW_PRESS, 0}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {
                            std::printf(
                                "key pressed is contained inside signal\n");
                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }

  {
    Signal sig{Move{400, 300}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {
                            std::printf("move is contained inside signal\n");
                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }

  {
    Signal sig{Press{400, 300}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {
                            std::printf("press is contained inside signal\n");
                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }

  {
    Signal sig{Scroll{400, 300, 10, 10}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {
                            std::printf("scroll is contained inside signal\n");
                          },
                          [&](Unpress arg) {

                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }

  {
    Signal sig{Unpress{400, 300}};
    std::visit(overloaded{[&](Empty arg) {

                          },
                          [&](Move arg) {

                          },
                          [&](Press arg) {

                          },
                          [&](Scroll arg) {

                          },
                          [&](Unpress arg) {
                            std::printf("unpress is contained inside signal\n");
                          },
                          [&](Key arg) {

                          },
                          [&](ItemDropped arg) {

                          }},
               sig);
  }
}

#include "userinterface/widgets/SignalProcessor.h"
#include <iostream>

void signal_processor_tutorial() {
  using namespace curan::ui;
  SkRect rectangle_outside = SkRect::MakeXYWH(50, 50, 100, 100);
  SkRect rectangle_inside = SkRect::MakeXYWH(75, 75, 25, 25);
  SignalInterpreter interpreter{};

  interpreter.set_format(true);
  auto check_inside_size = [&](double x, double y) {
    return rectangle_inside.contains(x, y);
  };
  auto check_allocated_area = [&](double x, double y) {
    return rectangle_outside.contains(x, y);
  };
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{1, 1});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{54, 54});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{54, 54});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{77, 77});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{77, 77});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Press{77, 77});
  std::cout << interpreter;
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{77, 77});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Unpress{77, 77});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{54, 54});
  std::cout << interpreter;
  interpreter.process(check_allocated_area, check_inside_size,
                      curan::ui::Move{1, 1});
  std::cout << interpreter;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include <iostream>

void empty_canvas_tutorial() {
  using namespace curan::ui;
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: empty canvas";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  SkPaint paint_square;
  paint_square.setStyle(SkPaint::kFill_Style);
  paint_square.setAntiAlias(true);
  paint_square.setStrokeWidth(4);
  paint_square.setColor(SK_ColorRED);

  while (!glfwWindowShouldClose(viewer->window)) {
    auto start = std::chrono::high_resolution_clock::now();
    SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
    SkCanvas *canvas = pointer_to_surface->getCanvas();
    canvas->drawColor(SK_ColorWHITE);
    SkPoint point{400, 400};
    canvas->drawCircle(point, 20.0, paint_square);
    glfwPollEvents();
    auto signals = viewer->process_pending_signals();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  return;
}

#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"

using button_ptr = std::unique_ptr<curan::ui::Button>;
using container_ptr = std::unique_ptr<curan::ui::Container>;

void create_buttons_for_demo(button_ptr &button, button_ptr &button2,
                             button_ptr &button3,
                             curan::ui::IconResources &resources) {
  using namespace curan::ui;
  button = Button::make("Touch!", resources);
  button->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  button2 = Button::make("Touch2!", resources);
  button2->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  button3 = Button::make("Touch3!", resources);
  button3->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));
}

void create_buttons_for_demo(button_ptr &button, button_ptr &button2,
                             button_ptr &button3, button_ptr &button4,
                             curan::ui::IconResources &resources) {
  using namespace curan::ui;
  button = Button::make("Touch!", resources);
  button->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  button2 = Button::make("Touch2!", resources);
  button2->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  button3 = Button::make("Touch3!", resources);
  button3->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  button4 = Button::make("Touch4!", resources);
  button4->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));
}

void buttons_and_containers_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  {
    button_ptr button, button2, button3;
    create_buttons_for_demo(button, button2, button3, resources);
    container_ptr container =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::HORIZONTAL);
    *container << std::move(button) << std::move(button2) << std::move(button3);
    container->set_divisions({0.0f, 0.33333f, 0.66666f, 1.0f}); // optional line
    container->compile();
    for (const auto &rec : container->get_positioning())
      std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop
                << " right: " << rec.fRight << " bottom: " << rec.fBottom
                << "\n";
  }

  {
    button_ptr button, button2, button3;
    create_buttons_for_demo(button, button2, button3, resources);
    container_ptr container =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::VERTICAL);
    *container << std::move(button) << std::move(button2) << std::move(button3);
    container->set_divisions({0.0f, 0.33333f, 0.66666f, 1.0f}); // optional line
    container->compile();
    for (const auto &rec : container->get_positioning())
      std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop
                << " right: " << rec.fRight << " bottom: " << rec.fBottom
                << "\n";
  }
  {
    button_ptr button, button2, button3;
    create_buttons_for_demo(button, button2, button3, resources);
    container_ptr container =
        Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                        Container::Arrangement::UNDEFINED);
    *container << std::move(button) << std::move(button2) << std::move(button3);
    container->set_variable_layout(
        {SkRect::MakeLTRB(0.4f, 0.0f, 0.3333f, 0.8f),
         SkRect::MakeLTRB(0.2f, 0.3333f, 0.6666f, 0.6f),
         SkRect::MakeLTRB(0.4f, 0.6666f, 1.0f, 0.8f)});
    container->compile();
    for (const auto &rec : container->get_positioning())
      std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop
                << " right: " << rec.fRight << " bottom: " << rec.fBottom
                << "\n";
  }
  {
    button_ptr button, button2, button3;
    create_buttons_for_demo(button, button2, button3, resources);
    Button *ptr_to_button = button.get();
    Button *ptr_to_button2 = button2.get();
    Button *ptr_to_button3 = button3.get();

    container_ptr container =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::HORIZONTAL);
    *container << std::move(button) << std::move(button2) << std::move(button3);
    container->set_divisions({0.0, 0.33333, 0.66666, 1.0});
    container->compile();

    SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
    container->set_position(my_small_window);
    container->framebuffer_resize(my_small_window);
    auto pos1 = ptr_to_button->get_position();
    std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop
              << " right: " << pos1.fRight << " bottom: " << pos1.fBottom
              << "\n";
    auto pos2 = ptr_to_button2->get_position();
    std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop
              << " right: " << pos2.fRight << " bottom: " << pos2.fBottom
              << "\n";
    auto pos3 = ptr_to_button3->get_position();
    std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop
              << " right: " << pos3.fRight << " bottom: " << pos3.fBottom
              << "\n";
  }
  {
    button_ptr button, button2, button3, button4;
    create_buttons_for_demo(button, button2, button3, button4, resources);
    Button *ptr_to_button = button.get();
    Button *ptr_to_button2 = button2.get();
    Button *ptr_to_button3 = button3.get();
    Button *ptr_to_button4 = button4.get();
    container_ptr container =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::VERTICAL);
    *container << std::move(button) << std::move(button2) << std::move(button3);
    container->set_divisions({0.0f, 0.33333f, 0.66666f, 1.0f});

    container_ptr container2 =
        Container::make(Container::ContainerType::LINEAR_CONTAINER,
                        Container::Arrangement::HORIZONTAL);
    *container2 << std::move(container) << std::move(button4);
    container2->set_divisions({0.0f, 0.5f, 1.0f});

    SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
    container2->set_position(my_small_window);
    container2->compile();
    container2->framebuffer_resize(my_small_window);

    auto pos1 = ptr_to_button->get_position();
    std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop
              << " right: " << pos1.fRight << " bottom: " << pos1.fBottom
              << "\n";
    auto pos2 = ptr_to_button2->get_position();
    std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop
              << " right: " << pos2.fRight << " bottom: " << pos2.fBottom
              << "\n";
    auto pos3 = ptr_to_button3->get_position();
    std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop
              << " right: " << pos3.fRight << " bottom: " << pos3.fBottom
              << "\n";
    auto pos4 = ptr_to_button4->get_position();
    std::cout << "Button3 left: " << pos4.fLeft << " top: " << pos4.fTop
              << " right: " << pos4.fRight << " bottom: " << pos4.fBottom
              << "\n";
  }
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include <iostream>

void containers_and_pages(){
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: containers and pages";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
  auto button = Button::make("Touch!", resources);
  button->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  auto button2 = Button::make("Touch2!", resources);
  button2->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  auto button3 = Button::make("Touch3!", resources);
  button3->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
  *container << std::move(button) << std::move(button2) << std::move(button3);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};
  page.update_page(viewer.get());
  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  return;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "utils/TheadPool.h"
#include <iostream>

void update_image_display(curan::ui::ImageDisplay *image_display,
                          size_t image_width, size_t image_height) {
  using namespace curan::ui;
  using namespace curan::utilities;

  auto raw_data =
      std::make_shared<std::vector<uint8_t>>(image_width * image_height, 0);
  for (auto &dat : *raw_data.get())
    dat = rand();
  image_display->update_image(ImageWrapper{
      CaptureBuffer::make_shared(raw_data->data(),
                                 raw_data->size() * sizeof(uint8_t), raw_data),
      image_width, image_height});
}

void image_display_tutorial() {
  using namespace curan::ui;
  using namespace curan::utilities;
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: image display";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
  std::unique_ptr<ImageDisplay> image_display = ImageDisplay::make();
  ImageDisplay *pointer_to = image_display.get();
  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::HORIZONTAL);
  *container << std::move(image_display);
  curan::ui::Page page{std::move(container), SK_ColorBLACK};
  page.update_page(viewer.get());

  std::atomic<bool> running = true;

  auto pool = ThreadPool::create(1);
  pool->submit("image display updater", [&]() {
    size_t image_width = 50;
    size_t image_height = 50;
    double timer = 0.0;
    while (running) {
      update_image_display(pointer_to, image_width + 20 * std::sin(timer),
                           image_height + 20 * std::cos(timer));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      timer += std::chrono::milliseconds(20).count() * 1e-3;
    }
  });

  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  running = false;
  return;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImutableTextPanel.h"
#include "userinterface/widgets/Page.h"
#include "utils/TheadPool.h"
#include <iostream>

void imutable_text_panel_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: imutable text";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  std::unique_ptr<ImutableTextPanel> layer =
      ImutableTextPanel::make("write for life");
  layer->set_background_color({1.f, 1.0f, 1.0f, 1.0f})
      .set_text_color({.0f, .0f, .0f, 1.0f});
  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::VERTICAL);
  layer->setFont(ImutableTextPanel::typeface::sans_serif);
  ImutableTextPanel *layer_ptr = layer.get();
  *container << std::move(layer);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ItemExplorer.h"
#include "userinterface/widgets/Page.h"
#include <iostream>

void item_explorer_tutorial() {
  using namespace curan::ui;
  using namespace curan::utilities;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();
  ;
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: item explorer";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  std::shared_ptr<std::array<unsigned char, 100 * 100>> image_buffer =
      std::make_shared<std::array<unsigned char, 100 * 100>>();
  constexpr float maximum_size = 2 * 50.0 * 50.0;
  for (size_t row = 0; row < 100; ++row)
    for (size_t col = 0; col < 100; ++col)
      (*image_buffer)[row + col * 100] = static_cast<unsigned char>(
          (((row - 50.0) * (row - 50.0) + (col - 50.0) * (col - 50.0)) /
           maximum_size) *
          255.0);

  auto buff = CaptureBuffer::make_shared(
      image_buffer->data(), image_buffer->size() * sizeof(unsigned char),
      image_buffer);

  std::map<int, std::string> items_to_add;
  items_to_add.emplace(0, "zero");
  items_to_add.emplace(1, "one");
  items_to_add.emplace(2, "two");
  items_to_add.emplace(3, "three");
  items_to_add.emplace(4, "four");
  items_to_add.emplace(5, "five");
  items_to_add.emplace(6, "six");
  items_to_add.emplace(7, "seven");
  items_to_add.emplace(8, "eight");
  items_to_add.emplace(9, "nine");
  items_to_add.emplace(10, "ten");
  items_to_add.emplace(11, "eleven");
  items_to_add.emplace(12, "twelve");
  items_to_add.emplace(13, "thirteen");

  auto item_explorer = ItemExplorer::make("file_icon.png", resources);
  auto ptr_item_explorer = item_explorer.get();

  std::atomic<bool> running = true;

  auto pool = ThreadPool::create(1);
  pool->submit("data injector and remover", [&]() {
    for (size_t i = 0; i < 14; ++i) {
      if(!running) return;
      ptr_item_explorer->add(Item{i, items_to_add.at(i), buff, 100, 100});
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if(!running) return;
    }

    for (size_t i = 0; i < 14; ++i) {
      if(!running) return;
      ptr_item_explorer->remove(i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if(!running) return;
    }
  });

  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::VERTICAL);
  *container << std::move(item_explorer);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  running = false;
  return;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Loader.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Page.h"
#include "utils/Logger.h"

#include <iostream>

std::unique_ptr<curan::ui::Overlay>
warning_overlay(const std::string &warning,
                curan::ui::IconResources &resources) {
  using namespace curan::ui;
  auto warn = Button::make(" ", "warning.png", resources);
  warn->set_click_color(SK_AlphaTRANSPARENT)
      .set_hover_color(SK_AlphaTRANSPARENT)
      .set_waiting_color(SK_AlphaTRANSPARENT)
      .set_size(SkRect::MakeWH(400, 200));

  auto button = Button::make(warning, resources);
  button->set_click_color(SK_AlphaTRANSPARENT)
      .set_hover_color(SK_AlphaTRANSPARENT)
      .set_waiting_color(SK_AlphaTRANSPARENT)
      .set_size(SkRect::MakeWH(200, 50));

  auto viwers_container =
      Container::make(Container::ContainerType::LINEAR_CONTAINER,
                      Container::Arrangement::VERTICAL);
  *viwers_container << std::move(warn) << std::move(button);
  viwers_container->set_color(SK_ColorTRANSPARENT)
      .set_divisions({0.0, .8, 1.0});

  return Overlay::make(std::move(viwers_container),
                       SkColorSetARGB(10, 125, 125, 125), true);
}

void loader_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: loader";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto button1 = Button::make("Temporal Calibration", resources);
  button1->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button1->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started Temporal Calibration", resources));
  });

  auto button2 = Button::make("Spatial Calibration", resources);
  button2->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button2->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started Spatial Calibration", resources));
  });

  auto button3 = Button::make("Volume ROI", resources);
  button3->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button3->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started desired Volume Specification", resources));
  });

  auto button4 = Button::make("Reconstruction", resources);
  button4->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button4->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started Volumetric Reconstruction", resources));
  });

  auto button5 = Button::make("Registration", resources);
  button5->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button5->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started Real-Time Registration", resources));
  });

  auto button6 = Button::make("Neuro Navigation", resources);
  button6->set_click_color(SK_ColorDKGRAY)
      .set_hover_color(SK_ColorLTGRAY)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(300, 300));
  button6->add_press_call([&](Button *inbut, Press pres, ConfigDraw *config) {
    if (config)
      config->stack_page->stack(
          warning_overlay("Started Neuro-Navigation", resources));
  });

  auto icon = resources.get_icon("hr_repeating.png");
  auto widgetcontainer =
      (icon) ? Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                               Container::Arrangement::VERTICAL, *icon)
             : Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                               Container::Arrangement::VERTICAL);
  *widgetcontainer << std::move(button1) << std::move(button2)
                   << std::move(button3) << std::move(button4)
                   << std::move(button5) << std::move(button6);
  widgetcontainer->set_variable_layout(
      {SkRect::MakeXYWH(0.0, 0.0, 0.3333, 0.5),
       SkRect::MakeXYWH(0.3332, 0.0, 0.3333, 0.5),
       SkRect::MakeXYWH(0.6665, 0.0, 0.3333, 0.5),
       SkRect::MakeXYWH(0.0, 0.5, 0.3333, 0.5),
       SkRect::MakeXYWH(0.3332, 0.5, 0.3333, 0.5),
       SkRect::MakeXYWH(0.6665, 0.5, 0.3333, 0.5)});

  widgetcontainer->set_color(SK_ColorBLACK);
  auto page = Page{std::move(widgetcontainer), SK_ColorBLACK};
  page.update_page(viewer.get());

  ConfigDraw config_draw{&page};
  config_draw.stack_page->stack(
      Loader::make("human_robotics_logo.jpeg", resources));

  viewer->set_minimum_size(page.minimum_size());

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

    if (!signals.empty())
      page.propagate_signal(signals.back(), &config_draw);
    page.propagate_heartbeat(&config_draw);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Loader.h"
#include "userinterface/widgets/Minipage.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Page.h"
#include "utils/Logger.h"

#include <iostream>

auto container1(curan::ui::IconResources &resources) {
  using namespace curan::ui;
  std::unique_ptr<Button> button = Button::make("Touch!", resources);
  button->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  std::unique_ptr<Button> button1 = Button::make("Leave!", resources);
  button1->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::HORIZONTAL);
  *container << std::move(button) << std::move(button1);
  return container;
}

auto container2(curan::ui::IconResources &resources) {
  using namespace curan::ui;
  std::unique_ptr<Button> button = Button::make("Do not Touch!", resources);
  button->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));

  std::unique_ptr<Button> button1 = Button::make("Do not leave!", resources);
  button1->set_click_color(SK_ColorRED)
      .set_hover_color(SK_ColorCYAN)
      .set_waiting_color(SK_ColorGRAY)
      .set_size(SkRect::MakeWH(100, 200));
  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::HORIZONTAL);
  *container << std::move(button) << std::move(button1);
  return container;
}

void minipage_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: minipage";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto minipage = MiniPage::make(container1(resources), SK_ColorBLACK);
  auto ptr_minipage = minipage.get();
  auto container_with_minipage =
      Container::make(Container::ContainerType::LINEAR_CONTAINER,
                      Container::Arrangement::VERTICAL);
  *container_with_minipage << std::move(minipage);

  curan::ui::Page page{std::move(container_with_minipage), SK_ColorBLACK};
  auto pool = curan::utilities::ThreadPool::create(1);
  std::atomic<bool> keep_running = true;
  pool->submit("update minipage", [&]() {
    size_t i = 0;
    while (keep_running) {
      ++i;
      ptr_minipage->construct(
          (i % 2 == 0 ? container1(resources) : container2(resources)),
          SK_ColorBLACK);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  });
  ConfigDraw config{&page};

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
    for(auto sig : signals)
      page.propagate_signal(sig, &config);
    page.propagate_signal(Empty{}, &config);
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
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/MutatingTextPanel.h"
#include "userinterface/widgets/Page.h"

void mutating_text_panel_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: editor panel";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  std::unique_ptr<MutatingTextPanel> layer =
      MutatingTextPanel::make(false, "write for life");
  layer->set_background_color({.0f, .0f, .0f, 1.0f})
      .set_text_color({1.f, 1.f, 1.f, 1.f})
      .set_highlighted_color({.2f, .2f, .2f, 1.0f})
      .set_cursor_color({1.0, 0.0, 0.0, 1.0});
  auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                                   Container::Arrangement::UNDEFINED);
  container->set_variable_layout({SkRect::MakeLTRB(0.1, 0.1, 0.9, 0.9)});
  container->set_color(SkColorSetARGB(255, 255, 255, 255));

  *container << std::move(layer);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
}

#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "utils/Logger.h"
#include <csignal>
#include <thread>

void GetRandomTestMatrix(igtl::Matrix4x4 &matrix) {
  float position[3];
  float orientation[4];
  static float phi = 0.0;
  position[0] = 50.0 * cos(phi);
  position[1] = 50.0 * sin(phi);
  position[2] = 50.0 * cos(phi);
  phi = phi + 0.2;
  static float theta = 0.0;
  orientation[0] = 0.0;
  orientation[1] = 0.6666666666 * cos(theta);
  orientation[2] = 0.577350269189626;
  orientation[3] = 0.6666666666 * sin(theta);
  theta = theta + 0.1;
  igtl::QuaternionToMatrix(orientation, matrix);
  matrix[0][3] = position[0];
  matrix[1][3] = position[1];
  matrix[2][3] = position[2];
}

void generate_image_message(curan::ui::OpenIGTLinkViewer *button) {
  static auto raw_data = std::vector<uint8_t>(100 * 100, 0);
  for (auto &dat : raw_data)
    dat = rand();

  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();

  int size[] = {100, 100, 1};
  float spacing[] = {1.0, 1.0, 5.0};
  int svsize[] = {100, 100, 1};
  int svoffset[] = {0, 0, 0};
  int scalarType = igtl::ImageMessage::TYPE_UINT8;

  ts->GetTime();

  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetDeviceName("ImagerClient");
  imgMsg->SetSubVolume(svsize, svoffset);
  imgMsg->AllocateScalars();

  std::memcpy(imgMsg->GetScalarPointer(), raw_data.data(), raw_data.size());

  igtl::Matrix4x4 matrix;
  GetRandomTestMatrix(matrix);
  imgMsg->SetMatrix(matrix);
  imgMsg->Pack();

  igtl::MessageHeader::Pointer header_to_receive = igtl::MessageHeader::New();
  header_to_receive->InitPack();
  std::memcpy(header_to_receive->GetPackPointer(), imgMsg->GetPackPointer(),
              header_to_receive->GetPackSize());

  header_to_receive->Unpack();
  igtl::MessageBase::Pointer message_to_receive = igtl::MessageBase::New();
  message_to_receive->SetMessageHeader(header_to_receive);
  message_to_receive->AllocatePack();
  std::memcpy(message_to_receive->GetPackBodyPointer(),
              imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());

  button->process_message(message_to_receive);
}

void generate_transform_message(curan::ui::OpenIGTLinkViewer *button) {
  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();

  auto start = std::chrono::high_resolution_clock::now();
  igtl::Matrix4x4 matrix;
  GetRandomTestMatrix(matrix);
  ts->GetTime();

  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetDeviceName("Tracker");

  transMsg->SetMatrix(matrix);
  transMsg->SetTimeStamp(ts);
  transMsg->Pack();

  igtl::MessageHeader::Pointer header_to_receive = igtl::MessageHeader::New();
  header_to_receive->InitPack();
  transMsg->GetBufferPointer();
  std::memcpy(header_to_receive->GetPackPointer(), transMsg->GetPackPointer(),
              header_to_receive->GetPackSize());

  header_to_receive->Unpack();
  igtl::MessageBase::Pointer message_to_receive = igtl::MessageBase::New();
  message_to_receive->SetMessageHeader(header_to_receive);
  message_to_receive->AllocatePack();
  std::memcpy(message_to_receive->GetPackBodyPointer(),
              transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());

  button->process_message(message_to_receive);
}

void open_igtlink_viewer_tutorial() {
  using namespace curan::ui;
  std::unique_ptr<Context> context = std::make_unique<Context>();
  ;
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: openigtlinkviewer";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto igtlink_viewer = OpenIGTLinkViewer::make();
  auto ptr_igtlink_viewer = igtlink_viewer.get();
  auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                                   Container::Arrangement::UNDEFINED);
  container->set_variable_layout({SkRect::MakeLTRB(0.1, 0.1, 0.9, 0.9)});
  container->set_color(SkColorSetARGB(255, 255, 255, 255));
  auto pool = curan::utilities::ThreadPool::create(1);
  *container << std::move(igtlink_viewer);

  std::atomic<bool> running = true;

  pool->submit("image generator and transform", [&]() {
    while (running) {
      auto start = std::chrono::high_resolution_clock::now();
      generate_image_message(ptr_igtlink_viewer);
      generate_transform_message(ptr_igtlink_viewer);
      auto end = std::chrono::high_resolution_clock::now();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(16) -
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
    }
  });

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      page.propagate_signal(signals.back(), &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  running = false;
}

#define STB_IMAGE_IMPLEMENTATION
#include "itkImage.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/definitions/Interactive.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

ImageType::Pointer get_volume(size_t dimensions) {
  ImageType::Pointer memo = ImageType::New();

  ImageType::SizeType size;
  size[0] = dimensions;
  size[1] = dimensions;
  size[2] = dimensions;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::RegionType region;
  region.SetIndex(start);
  region.SetSize(size);

  const itk::SpacePrecisionType origin[Dimension] = {0.0, 0.0, 0.0};
  memo->SetOrigin(origin);
  const itk::SpacePrecisionType spacing[Dimension] = {1.0, 1.0, 1.0};
  memo->SetSpacing(spacing);
  memo->SetRegions(region);

  memo->Allocate();

  auto raw_buffer = memo->GetBufferPointer();
  for (size_t i = 0; i < dimensions * dimensions * dimensions; ++i)
    raw_buffer[i] = rand();

  return memo;
}

void slider_panel_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1200};
  param.windowName = "tutorial: slider panel";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  VolumetricMask mask{get_volume(100)};
  std::unique_ptr<curan::ui::SlidingPanel> image_display =
      curan::ui::SlidingPanel::make(resources, &mask, curan::ui::Direction::Z);

  auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                                   Container::Arrangement::UNDEFINED);
  container->set_variable_layout({SkRect::MakeLTRB(0.1, 0.1, 0.9, 0.9)});
  container->set_color(SkColorSetARGB(255, 255, 255, 255));
  *container << std::move(image_display);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    for (auto &&sig : signals)
      page.propagate_signal(sig, &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  return;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Plotter.h"

void plotter_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "tutorial: plotter";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto plotter = Plotter::make(500, 2);
  auto ptr_plotter = plotter.get();
  auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                                   Container::Arrangement::UNDEFINED);
  container->set_variable_layout({SkRect::MakeLTRB(0.1, 0.1, 0.9, 0.9)});
  container->set_color(SkColorSetARGB(255, 255, 255, 255));
  auto pool = curan::utilities::ThreadPool::create(1);

  std::atomic<bool> running = true;

  pool->submit("plotter update", [&]() {
    double timing = 0.0;
    while (running) {
      auto start = std::chrono::high_resolution_clock::now();
      ptr_plotter->append(SkPoint::Make(timing, std::sin(timing)), 0);
      ptr_plotter->append(SkPoint::Make(timing, std::cos(timing) + 1), 1);
      auto end = std::chrono::high_resolution_clock::now();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(16) -
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
      timing += 16 * 1e-3;
    }
  });

  *container << std::move(plotter);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      for (auto &&sig : signals)
        page.propagate_signal(sig, &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  running = false;
  return;
}

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Slider.h"


void slider_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();

  DisplayParams param{std::move(context), 2200, 1200};
  param.windowName = "tutorial: slider";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto slider = Slider::make({ 0.0f, 300.0f });
		slider->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorCYAN).set_waiting_color(SK_ColorGRAY)
				.set_slider_color(SK_ColorLTGRAY).set_callback([](Slider* slider, ConfigDraw* config) {
			std::cout << "received signal!\n";
		}).set_size(SkRect::MakeWH(300,40));

  auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,
                                   Container::Arrangement::VERTICAL);
  *container << std::move(slider);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    for(auto sig : signals)
      page.propagate_signal(sig, &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  return;
}


#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/TextBlob.h"
#include <iostream>
#include <thread>

void textblob_tutorial() {
  using namespace curan::ui;
  IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 2200, 1800};
  param.windowName = "textblob tutorial";
  std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

  auto textblob = TextBlob::make("example text");
  textblob->set_text_color(SK_ColorWHITE)
      .set_background_color(SK_ColorBLACK)
      .set_size(SkRect::MakeWH(200, 90));
  auto container = Container::make(Container::ContainerType::VARIABLE_CONTAINER,
                                   Container::Arrangement::UNDEFINED);
  container->set_variable_layout({SkRect::MakeLTRB(0.1, 0.1, 0.9, 0.9)});
  container->set_color(SkColorSetARGB(255, 255, 255, 255));
  *container << std::move(textblob);

  curan::ui::Page page{std::move(container), SK_ColorBLACK};

  ConfigDraw config{&page};

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
    if (!signals.empty())
      for (auto &&sig : signals)
        page.propagate_signal(sig, &config);
    glfwPollEvents();

    bool val = viewer->swapBuffers();
    if (!val)
      std::cout << "failed to swap buffers\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(16) -
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  return;
}

int main() {
  try {
    
    //signal_tutorial();
    signal_processor_tutorial();
    //empty_canvas_tutorial();
    //buttons_and_containers_tutorial();
    //image_display_tutorial();
    //imutable_text_panel_tutorial();
    //item_explorer_tutorial();
    //loader_tutorial();
    //minipage_tutorial();
    //mutating_text_panel_tutorial();
    //open_igtlink_viewer_tutorial();
    //plotter_tutorial();
    slider_panel_tutorial();
    //slider_tutorial();
    //textblob_tutorial();
    return 0;
  } catch (...) {
    return 1;
  }
}