#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include <iostream>

void create_horizontal_layout(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button3 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);

	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\nRect1 left: 0 top: 0  right: 0.33333 bottom: 1 \n";
	std::cout << "Rect2 left: 0.33333 top: 0  right: 0.66666 bottom: 1 \n";
	std::cout << "Rect3 left: 0.66666 top: 0  right: 1.00000 bottom: 1 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
}

void create_vertical_layout(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button3 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);

	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\nRect1 left: 0.0 top: 0.0  right: 1.0 bottom: 1.0 \n";
			   std::cout << "Rect2 left: 0.0 top: 0.3333  right: 1.0 bottom: 0.6666 \n";
			   std::cout << "Rect3 left: 0.0 top: 0.6666  right: 1.0 bottom: 1.0 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
}

void create_variable_layout(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button3 = Button::make(infor);

	Container::InfoVariableContainer info;
	info.layouts = { button ,button2 , button3 };
	info.rectangles_of_contained_layouts = { SkRect::MakeLTRB(0.0,0.0,0.3333,1.0),SkRect::MakeLTRB(0.3333,0.0,0.6666,1.0),SkRect::MakeLTRB(0.6666,0.0,1.0,1.0) };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);

	auto rect_layout = container->get_positioning();
	std::cout << "Container layout";
	std::cout << "Expected:\nRect1 left: 0 top: 0  right: 0.33333 bottom: 1 \n";
	std::cout << "Rect2 left: 0.33333 top: 0  right: 0.66666 bottom: 1 \n";
	std::cout << "Rect3 left: 0.66666 top: 0  right: 1.00000 bottom: 1 \n";

	std::cout << "Real:\n";
	for (const auto& rec : rect_layout)
		std::cout << "Rect left: " << rec.fLeft << " top: " << rec.fTop << " right: " << rec.fRight << " bottom: " << rec.fBottom << "\n";
}


void create_horizontal_layout_propagate(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button3 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container->set_position(my_small_window);
	container->framebuffer_resize();

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left:  50 top:  50  right: 350 bottom: 950 \n";
	std::cout << "Button2 left: 350 top:  50  right: 650 bottom: 950 \n";
	std::cout << "Button3 left: 650 top:  50  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = button->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
}


void create_vertical_layout_propagate(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button3 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container->set_position(my_small_window);
	container->framebuffer_resize();

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left: 50 top:  50  right: 950 bottom: 350 \n";
	std::cout << "Button2 left: 50 top: 350  right: 950 bottom: 650 \n";
	std::cout << "Button3 left: 50 top: 650  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = button->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
}


void create_nested_layout_propagate(curan::ui::IconResources& resources){
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	std::shared_ptr<Button> button3 = Button::make(infor);

	infor.button_text = "Touch 4!";
	std::shared_ptr<Button> button4 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);


	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.5 , 1.0 };
	info.layouts = { container , button4 };
	std::shared_ptr<Container> container2 = Container::make(info);

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container2->set_position(my_small_window);
	container2->framebuffer_resize();

	std::cout << "Container layout";
	std::cout << "Expected:\n";
	std::cout << "Button1 left:  50 top:  50  right: 450 bottom: 350 \n";
	std::cout << "Button2 left:  50 top: 350  right: 450 bottom: 650 \n";
	std::cout << "Button3 left:  50 top: 650  right: 450 bottom: 950 \n";
	std::cout << "Button4 left: 450 top:  50  right: 950 bottom: 950 \n";

	std::cout << "Real:\n";
	auto pos1 = button->get_position();
	std::cout << "Button1 left: " << pos1.fLeft << " top: " << pos1.fTop << " right: " << pos1.fRight << " bottom: " << pos1.fBottom << "\n";
	auto pos2 = button2->get_position();
	std::cout << "Button2 left: " << pos2.fLeft << " top: " << pos2.fTop << " right: " << pos2.fRight << " bottom: " << pos2.fBottom << "\n";
	auto pos3 = button3->get_position();
	std::cout << "Button3 left: " << pos3.fLeft << " top: " << pos3.fTop << " right: " << pos3.fRight << " bottom: " << pos3.fBottom << "\n";
	auto pos4 = button4->get_position();
	std::cout << "Button4 left: " << pos4.fLeft << " top: " << pos4.fTop << " right: " << pos4.fRight << " bottom: " << pos4.fBottom << "\n";
};

void test_linearization(curan::ui::IconResources& resources) {
	using namespace curan::ui;
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

	SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
	text_font.setEdging(SkFont::Edging::kAntiAlias);

	SkPaint paint_square2;
	paint_square2.setStyle(SkPaint::kFill_Style);
	paint_square2.setAntiAlias(true);
	paint_square2.setStrokeWidth(4);
	paint_square2.setColor(SkColorSetARGB(255, 201, 201, 201));

	Button::Info infor{ resources };
	infor.button_text = "Touch!";
	infor.click_color = SK_ColorRED;
	infor.hover_color = SK_ColorCYAN;
	infor.waiting_color = SK_ColorGRAY;
	infor.icon_identifier = "";
	infor.paintButton = paint_square;
	infor.paintText = paint_text;
	infor.size = SkRect::MakeLTRB(0, 0, 100, 200);
	infor.textFont = text_font;
	std::shared_ptr<Button> button = Button::make(infor);

	infor.button_text = "Touch 2!";
	std::shared_ptr<Button> button2 = Button::make(infor);

	infor.button_text = "Touch 3!";
	std::shared_ptr<Button> button3 = Button::make(infor);

	infor.button_text = "Touch 4!";
	std::shared_ptr<Button> button4 = Button::make(infor);

	Container::InfoLinearContainer info;
	info.arrangement = curan::ui::Arrangement::VERTICAL;
	info.divisions = { 0.0 , 0.33333 , 0.66666 , 1.0 };
	info.layouts = { button ,button2 , button3 };
	info.paint_layout = paint_square2;
	std::shared_ptr<Container> container = Container::make(info);


	info.arrangement = curan::ui::Arrangement::HORIZONTAL;
	info.divisions = { 0.0 , 0.5 , 1.0 };
	info.layouts = { container , button4 };
	std::shared_ptr<Container> container2 = Container::make(info);

	SkRect my_small_window = SkRect::MakeLTRB(50, 50, 950, 950);
	container2->set_position(my_small_window);
	container2->framebuffer_resize();

	std::vector<drawablefunction> temp_draw;
	std::vector<callablefunction> temp_call;
	container2->linearize_container(temp_draw,temp_call);

	std::cout << "expected size drawable: (6) real size: (" << temp_draw.size() << ")\n";
	std::cout << "expected size callable: (6) real size: (" << temp_call.size() << ")\n";
}

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ "C:/dev/Curan/resources" };
		create_horizontal_layout(resources);
		create_vertical_layout(resources);
		create_variable_layout(resources);
		create_horizontal_layout_propagate(resources);
		create_vertical_layout_propagate(resources);
		create_nested_layout_propagate(resources);
		test_linearization(resources);
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}