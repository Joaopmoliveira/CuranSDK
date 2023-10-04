---
layout: "page"
permalink : "/user_interface/"
---

### User Interface

The user interface library is build upon [SKIA](https://skia.org/). This is a great library in cpp which renders geometries, images, paths, and many other geometries. Check the SKIA [API](https://skia.org/docs/user/api/) to see what is possible. The main thing you need to understand is that Curan only executes the connection between the GPU and your CPU, all other things are taken care of by SKIA. 

The first step in all our programming is to properly link our executable to the user interface library, we can achieve this through.

```cmake
add_executable(myexecutable main.cpp)


target_link_libraries(myexecutable PUBLIC
userinterface
)
```
Now the compiler can link safely to our library. 

* Empty Canvas : [Empty Canvas](#Empty Canvas)
* Buttons : [Buttons](#Buttons)
* ImageDisplay : [ImageDisplay](#ImageDisplay)

## Empty Canvas 

This is a snipeat of code which shows how you can create an empty canvas

```cpp
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
```

First we need to include the necessary classes to link to our executable. Next we define our the entry point to our executable looks like, i.e. the classic main function 

```cpp
int main() {
	std::unique_ptr<curan::ui::Context> context = std::make_unique<curan::ui::Context>();
	curan::ui::DisplayParams param{ std::move(context),1200,800 };
	std::unique_ptr<curan::ui::Window> viewer = std::make_unique<curan::ui::Window>(std::move(param));
```

the first object of the type curan::ui::Context basically establishes a link to your GPU, so that we can use Vulkan under the hood to render our application blazing fast. The constructor of the class creates the necessary abstractions and when it is deleted the vulkan objects are deleated in the proper order. The DisplayParams object defines the size of the window and receives the unique pointer of the context object. This object is unique, meaning that only a single instance exists, i.e. when you can the std::move() function the context object is no longer usable in your code because it is now a member of the param object.

```cpp
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
```

first you can check if the viewer has received an external request to terminate. If it has then you should get out of the loop so that the window can close properly. If it has not then we can continue with our custom rendering logic. The underlying library we are using to render things on screen is SKIA, which we can do thourhg the canvas pointer. For more information please visit this [website](https://skia.org/docs/user/api/skcanvas_overview/). 

```cpp
        //do your own things
		canvas->drawColor(SK_ColorWHITE);
```

here is where we would draw our 2D figures as needed for our application, SKIA gives you a lot of freedom on how to render things on screen. You could draw boxes with text, which we could call buttons, we could draw images, etc... Later we will show you the abstractions we created to save you some of this bottersome work. 

```cpp
		glfwPollEvents();
```

This last portion of the code queries the underlying window of the operating system for signals which where receives whilst we were drawing our scene, (in detail the glfwPollEvents() call takes the signal received and places it in a queue of the viewer object for later pos-processing).  

```cpp
		auto signals = viewer->process_pending_signals();
		bool val = viewer->swapBuffers();
		if (!val)
			std::printf("failed to swap buffers\n");
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	return 0;
}
```

The last portion of the code takes the signals placed in the interal queue of the viewer for our custom logic. Notice that you could use these signals for some custom behavior of your application, e.g. detect if an item has been drag and dropped. 

To sum up, the previous source code creates a window through the [GLFW library](https://www.glfw.org/) which appends signals to the 'std::unique_ptr<Window> viewer' object. The types of signals Curan propagates are Empty,Move, Press, Scroll, Unpress, ItemDropped, Key. The move is a mouse movement, the Press is a mouse press, the Scroll is the scroll with a mouse, the unpress is when the mouse is released, itemdropped is when you drag a item into the window and key is a keyboard press. To obtain which signals have been propagated to our window we can query the internal buffer of the window.

## Buttons

This while loop runs until the window is closed. Now obviously you don't want to program all types of objects like buttons and so on, everytime you want this type of behavior. Curan has a light Widget implementation which you can use for your goals. Lets see how curan goes about defining this widget behavior

Assume that you want three buttons all with distict behaviors:

1. Button 1 - name "print name" which prints a name when clicked on the output stream

2. Button 2 - name "print age" which prints an age when clicked on the output stream

3. Button 3 - name "print Eureka" which prints a Eureka when clicked on the output stream

Further notice that we would like to control the colors when we are hovering with the mouse, when we click the button and while nothing is interacting with the button. This is controlled through the Button::Info structure feed into the constructor of Button. We would also like Button 1 to be above button 2 and these two are on the left side of button 3. To achieve this we must first create the buttons as follows

```cpp
std::unique_ptr<Button> button1 = Button::make("print name",resources);

std::unique_ptr<Button> button2 = Button::make("print age",resources);

std::unique_ptr<Button> button3 = Button::make("print eureka",resources);

```
Notice that the simplicity of creating these buttons, you dont need a lot of work to get buttons up a running. By default they have a custom coloring scheme when you hover, when you click, etc.. Assume that you want to custumize the look the buttons. Btw because we are fancy, we can use the auto keyword to reduce the amount of lines of code we need to write as such (auto deduces the type of the object at compile time, thus both expressions are equivalent)

```cpp
auto button1 = Button::make("print name",resources);
button1->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

auto button2 = Button::make("print age",resources);
button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));

auto button3 = Button::make("print eureka",resources);
button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
```

Notice that we can compose calls in sequence when configuring buttons, this applies to most functions of the object (after the set_size call you can't compose more calls in sequence because the method does not return a reference to our object). The only thing we are missing is to customize the callback that we desire for our object as such

```cpp
auto button1 = Button::make("print name",resources);
button1->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button1->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "my name is Eminem\n";
});
auto button2 = Button::make("print age",resources);
button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button2->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "I am old\n";
});
auto button3 = Button::make("print eureka",resources);
button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button3->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "Eureka, I wrote a rime\n";
});
```

Now the buttons have some of the desired behavior we desire, but some things are still missing, such as their layout. To control the layout of buttons on screen Curan has the concept of a container. Containers are objects which contain other objects, be them widgets, or other containers. Given the specification of the buttons we defined we need two containers. One which is a vertical container which contains button 1 and 2 and an horizontal container which contains the first container and button 3. This is done as follows

```cpp
auto button1 = Button::make("print name",resources);
button1->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button1->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "my name is Eminem\n";
});
auto button2 = Button::make("print age",resources);
button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button2->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "I am old\n";
});
auto button3 = Button::make("print eureka",resources);
button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button3->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "Eureka, I wrote a rime\n";
});

auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
*buttoncontainer << std::move(button1) << std::move(button2);

auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
*widgetcontainer << std::move(buttoncontainer) << std::move(button3);
 ```

This takes the total available space on screen and by default it lays out our objects on screen. If you want to control how must space each object has, we can specify the related spacing as such 

```cpp
auto button1 = Button::make("print name",resources);
button1->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button1->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "my name is Eminem\n";
});
auto button2 = Button::make("print age",resources);
button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button2->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "I am old\n";
});
auto button3 = Button::make("print eureka",resources);
button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button3->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "Eureka, I wrote a rime\n";
});

auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
*buttoncontainer << std::move(button1) << std::move(button2);
buttoncontainer->set_division({0.0 0.3 1.0});

auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
*widgetcontainer << std::move(buttoncontainer) << std::move(button3);
widgetcontainer->set_division({0.0 0.6 1.0});
 ```

 Now button 1 and 2 will ocupy 60% of the screen horizontally and 30% for button 1 vertically and 70% for button 2 while button 3 ocupies 40% horizontally and 100% vertically.

Now we need to draw the containers somehow. We do this by creating a page. A page contains one container which can itself contain containers recursivelly. This allows us to create a tree structure of things to draw on screen. Because our containers are already completly defined we just need to tell the page that it must draw the widgetcontainer

```cpp
auto rec = viewer->get_size();
Page page = Page{widgetcontainer, SK_ColorBLACK};
page.propagate_size_change(rec);
```

We query the size of the viewer and propagate this size throught the page so that all widgets/ containers compute their relative position on screen.
With this we finaly have a page that we can draw. Lets look at the loop which draws our window

```cpp
int width = rec.width();
int height = rec.height();

ConfigDraw config{&page};

while (!glfwWindowShouldClose(viewer->window)) {
	auto start = std::chrono::high_resolution_clock::now();
	SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
	auto temp_height = pointer_to_surface->height();
	auto temp_width = pointer_to_surface->width();
	SkCanvas* canvas = pointer_to_surface->getCanvas();
	if (temp_height != height || temp_width != width) {
		rec = SkRect::MakeWH(temp_width, temp_height);
		page->propagate_size_change(rec);
    }
	page->draw(canvas);
	auto signals = viewer->process_pending_signals();
	if (!signals.empty())
		page->propagate_signal(signals.back(),&config);
	glfwPollEvents();
	
	bool val = viewer->swapBuffers();
	if (!val)
		std::cout << "failed to swap buffers\n";
	auto end = std::chrono::high_resolution_clock::now();
	std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
}
return 0;
```

This is the result of all of our hard work
![buttons_container]({{ site.baseurl }}/assets/images/buttons_container.png)

Altough its a bit anoying that the lettering type is too small. Well no problem, in our Button we can custumize the size of the type of letter to be larger, (the default is 15) and you can also customize the type of lettering used!, lets change that from 15 to 30 as follows

```cpp
auto button1 = Button::make("print name",resources);
button1->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button1->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "my name is Eminem\n";
}).set_font_size(30);
auto button2 = Button::make("print age",resources);
button2->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button2->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "I am old\n";
}).set_font_size(30);
auto button3 = Button::make("print eureka",resources);
button3->set_click_color(SK_ColorGRAY).set_hover_color(SK_ColorDKGRAY).set_waiting_color(SK_ColorBLACK).set_size(SkRect::MakeWH(100, 80));
button3->set_callback([](Button* button,ConfigDraw* config){
    std::cout << "Eureka, I wrote a rime\n";
}).set_font_size(30);
```

This results in the following 
![buttons_larger_letter_type]({{ site.baseurl }}/assets/images/buttons_larger_letter_type.png)

Now obviously there are more widgets which are usefull in this context. For example in the context of Curan, it is extremelly important to draw an image which we received from a peripheral at a constant framerate. To do this we developed the ImageDisplay class. 

## ImageDisplay

Assume that you are testing an algorithm and just want to see how the image looks. Well for that we can define the ImageDisplay class and a single container which completly fills our Page as follows.

```cpp
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>

int main() {
try {
	using namespace curan::ui;
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),1200,800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    auto image_display = ImageDisplay::make();
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display);
    curan::ui::Page page{std::move(container),SK_ColorBLACK}

    int width = rec.width();
    int height = rec.height();

    ConfigDraw config{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
	    auto start = std::chrono::high_resolution_clock::now();
	    SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
    	auto temp_height = pointer_to_surface->height();
    	auto temp_width = pointer_to_surface->width();
	    SkCanvas* canvas = pointer_to_surface->getCanvas();
    	if (temp_height != height || temp_width != width) {
    		rec = SkRect::MakeWH(temp_width, temp_height);
    		page->propagate_size_change(rec);
        }
    	page->draw(canvas);
    	auto signals = viewer->process_pending_signals();
    	if (!signals.empty())
    		page->propagate_signal(signals.back(),&config);
    	glfwPollEvents();
	
    	bool val = viewer->swapBuffers();
    	if (!val)
    		std::cout << "failed to swap buffers\n";
    	auto end = std::chrono::high_resolution_clock::now();
    	std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
    }
    return 0;   
    }
catch (...) {
	std::cout << "Failed";
	return 1;
}
}
```

Now we need to use this pointer somewhere. Remember that we are using unique pointer, i.e. only a single instance of this pointer exists. To use this object somewhere else we can get the underlying pointer, but the memory is still controlled by the unique pointer. Lets lauch a thread to generate a random image. We can first implement a class that represents a block of memory which we can acess as an image. This is a simple implementation of a class that contains memory. We can make copies of this class as we wish.

```cpp
class ImageTesting {
	int _width;
	int _height;
	std::shared_ptr<unsigned char[]> buffer;
public:

	ImageTesting(int width, int height) : _width{ width }, _height{ height } {
		buffer = std::shared_ptr<unsigned char[]>(new unsigned char[width() * height()]);
	}

	inline int width() {
		return _width;
	}

	inline int height() {
		return _height;
	}

	inline void set(int w, int h, char val) {
		unsigned char* loc = nullptr;
		if (buffer) {
			loc = buffer.get();
			loc[w + h * height()] = val;
		}
	}

	inline int size() {
		return _width * _height;
	}

	unsigned char* get_scalar_pointer() {
		if (buffer)
			return buffer.get();
		return nullptr;
	}
};
```
Lets also write a function that takes this image and generates a spiral shape as such

```cpp

ImageTesting update_texture(ImageTesting image, float value) {

	for (int32_t r = 0; r < image.height(); ++r)
	{
		float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
		for (int c = 0; c < image.width(); ++c)
		{
			float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

			vec2 delta{ (r_ratio - 0.5f), (c_ratio - 0.5f) };

			float angle = std::atan2(delta.x, delta.y);

			float distance_from_center = delta.norm();

			float intensity = (sin(1.0 * angle + 30.0f * distance_from_center + 10.0 * value) + 1.0f) * 0.5f;
			unsigned char val = (int)((intensity + 0.5) * 255);
			image.set(c, r, val);
		}
	}
	return image;
}
```
Now we can finally lauch our thread with the logic to upload our image. Notice that we must capture in the lambda our image. Thats because we need to guarantee that the block of memory of the image (the shared pointer) is alive when the page tries to render it. 

```cpp
#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>

void funtion(ImageDisplay* display){
    double timer = 0.0;
    while(true){
        ImageTesting img{ 100,100 };
        img = update_texture(std::move(img), 1.0 + time);
        auto lam = [img](SkPixmap& requested) {
	        auto inf = SkImageInfo::Make(img.width(), img.height(), SkColorType::kGray_8_SkColorType, SkAlphaType::kOpaque_SkAlphaType);
	        size_t row_size = img.width() * sizeof(char);
	        SkPixmap map{inf,img->GetScalarPointer(),row_size};
	        requested = map;
	        return;
        };
        display->update_image(lam);
        timer += 0.001;
    }
}

int main() {
try {
	using namespace curan::ui;
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),1200,800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

    auto image_display = ImageDisplay::make();
    ImageDisplay* pointer_to_block_of_memory = image_display.get();
    auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
    *container << std::move(image_display);
    curan::ui::Page page{std::move(container),SK_ColorBLACK}

    std::thread image_generator(funtion(pointer_to_block_of_memory));

    int width = rec.width();
    int height = rec.height();

    ConfigDraw config{&page};

    while (!glfwWindowShouldClose(viewer->window)) {
	    auto start = std::chrono::high_resolution_clock::now();
	    SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
    	auto temp_height = pointer_to_surface->height();
    	auto temp_width = pointer_to_surface->width();
	    SkCanvas* canvas = pointer_to_surface->getCanvas();
    	if (temp_height != height || temp_width != width) {
    		rec = SkRect::MakeWH(temp_width, temp_height);
    		page->propagate_size_change(rec);
        }
    	page->draw(canvas);
    	auto signals = viewer->process_pending_signals();
    	if (!signals.empty())
    		page->propagate_signal(signals.back(),&config);
    	glfwPollEvents();
	
    	bool val = viewer->swapBuffers();
    	if (!val)
    		std::cout << "failed to swap buffers\n";
    	auto end = std::chrono::high_resolution_clock::now();
    	std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
    }
    image_generator.join();
    return 0;   
    }
catch (...) {
	std::cout << "Failed";
	return 1;
}
}
```
There are more wigets available in the library. Read some of the examples to find the tools which are usefull to your needs (there are slider), pure text, and more!