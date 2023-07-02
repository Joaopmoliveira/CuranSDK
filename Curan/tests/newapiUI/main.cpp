#define STB_IMAGE_IMPLEMENTATION
#include "modifieduserinterface/widgets/ConfigDraw.h"
#include "modifieduserinterface/Window.h"
#include "modifieduserinterface/widgets/Button.h"
#include "modifieduserinterface/widgets/IconResources.h"
#include <iostream>

int main() {
		curan::ui::IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		curan::ui::Button button{resources,""};
        button.set_click_color(SK_ColorBLUE).set_current_state(curan::ui::Button::ButtonStates::PRESSED).set_hover_color(SK_ColorBLUE).set_waiting_color(SK_ColorBLUE);
		return 0;
}