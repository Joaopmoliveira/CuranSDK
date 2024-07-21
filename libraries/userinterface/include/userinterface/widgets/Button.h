#ifndef CURAN_BUTTON_HEADER_FILE_
#define CURAN_BUTTON_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"

namespace curan {
namespace ui {

/*
The Button class codifies behavior so that when the user clicks 
on the region where the button is located, any callback provided 
by the user is called. One can codify the string which is shown
to the user, with the button name a customize the visual look of the object.

The first thing to know is that a button has three states that it can
be in:
WAITING - the button is not being hovered and not being clicked
PRESSED - the button is being pressed thus the callback is called
HOVER - the mouse is currently hovering the button

Given these three states, you can customize the behavior of the button
depending on the state which is in.

Its also important to understand the logic of the API behind the buttons. 
To reduce clutter, every setter method returns a reference to (*this) 
object, meaning that you can compose multiple calls in a row

auto button = Button::make("Connect",resources);
button->set_click_color(SK_ColorGRAY)
	   .set_hover_color(SK_ColorDKGRAY)
	   .set_waiting_color(SK_ColorBLACK)
	   .set_size(SkRect::MakeWH(100, 80));

This is readable, and looks cool, as as Kannye West said:
"I do dope things because I am dope and my life is dope"

You can attach these buttons to pages which will be rendered as explained
in the Window class

You can also customize the callback behavior as in 

button->set_callback([](Button* button, ConfigDraw* config) {
		std::cout << "This button was clicked";
	});

You can add multiple callbacks in a row, they will be called in order
because they are stored in a list. Another cool trick is that 
the callback receives a pointer to the button that was clicked, thus 
you can change the state of the button 

button->set_callback([](Button* button, ConfigDraw* config) {
		std::cout << "This button was clicked";
		if(button) button->set_waiting_color(SK_ColorRED);
	});

Now the button waits with the red color in the following drawing operations

*/

class Button : public  Drawable , public utilities::Lockable, public SignalProcessor<Button> {

public:

enum class ButtonStates {
	WAITING,
	PRESSED,
	HOVER,
};

static std::unique_ptr<Button> make(const std::string& button_text,IconResources& system_icons);
static std::unique_ptr<Button> make(const std::string& button_text,const std::string& icon_identifier,IconResources& system_icons);

void compile() override;

~Button();

drawablefunction draw() override;
callablefunction call() override;

inline Button& set_font_size(const size_t & in_size) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	font_size = in_size;
    return *(this);
}

inline Button& set_font_source(sk_sp<SkTypeface> in_typeface) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	typeface = in_typeface;
    return *(this);
}

inline SkColor get_hover_color() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return hover_color;
}

inline Button& set_hover_color(SkColor color) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	hover_color = color;
    return *(this);
}

inline SkColor get_waiting_color() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return waiting_color;
}

inline Button& set_waiting_color(SkColor new_waiting_color) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	waiting_color = new_waiting_color;
    return *(this);
}

inline Button& set_click_color(SkColor new_click_color) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	click_color = new_click_color;
    return *(this);
}

inline SkColor get_click_color() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return click_color;
}

inline ButtonStates get_current_state() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return current_state;
}

inline Button& set_current_state(ButtonStates state) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	current_state = state;
    return *(this);
}

private:
	Button(const std::string& button_text,IconResources& system_icons);
	Button(const std::string& button_text,const std::string& icon_identifier,IconResources& system_icons);
	Button(const Button& other) = delete;

	SkColor hover_color;
	SkColor waiting_color;
	SkColor click_color;
	SkColor text_color;
	SkPaint paint;
	SkPaint paint_text;
	SkRect widget_rect_text;
	sk_sp<SkTypeface> typeface;
	size_t font_size = 15;
	std::string button_text;
	std::string icon_identifier;
	sk_sp<SkTextBlob> text;
	sk_sp<SkImage> icon_data;
	ButtonStates current_state = ButtonStates::WAITING;
	IconResources& system_icons;
	bool compiled = false;
	SignalInterpreter interpreter;
};

}
}

#endif