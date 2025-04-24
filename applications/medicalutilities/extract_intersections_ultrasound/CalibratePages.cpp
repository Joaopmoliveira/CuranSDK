#include "CalibratePages.h"

curan::ui::Page create_main_page(std::shared_ptr<ProcessingMessage>& processing ,curan::ui::IconResources& resources) {
	using namespace curan::ui;

	auto igtlink_viewer = OpenIGTLinkViewer::make();
	igtlink_viewer->set_size(SkRect::MakeWH(0,0));
	auto igtlink_viewer_pointer = igtlink_viewer.get();
	uint_least8_t dicom_compliant_conversion[256] = {0 ,30 ,33 ,35 ,37 ,39 ,40 ,41 ,43 ,44 ,45 ,46 ,47 ,48 ,48 ,49 ,50 ,51 ,51 ,52 ,53 ,54 ,55 ,55 ,56 ,57 ,57 ,58 ,59 ,60 ,60 ,61 ,62 ,62 ,63 ,64 ,64 ,65 ,65 ,66 ,67 ,67 ,68 ,69 ,69 ,70 ,71 ,71 ,72 ,73 ,73 ,74 ,75 ,76 ,76 ,77 ,77 ,78 ,79 ,80 ,80 ,80 ,81 ,82 ,83 ,83 ,84 ,84 ,85 ,86 ,86 ,87 ,88 ,88 ,89 ,90 ,90 ,91 ,92 ,93 ,93 ,94 ,95 ,95 ,96 ,96 ,97 ,98 ,98 ,99 ,100 ,101 ,101 ,102 ,103 ,103 ,104 ,105 ,106 ,106 ,107 ,108 ,109 ,109 ,110 ,111 ,111 ,112 ,113 ,113 ,114 ,115 ,116 ,116 ,117 ,118 ,119 ,119 ,120 ,121 ,122 ,123 ,123 ,124 ,125 ,126 ,126 ,127 ,128 ,128 ,129 ,130 ,131 ,132 ,132 ,133 ,134 ,135 ,136 ,136 ,137 ,138 ,139 ,140 ,141 ,141 ,142 ,143 ,144 ,145 ,145 ,146 ,147 ,148 ,149 ,150 ,151 ,151 ,152 ,153 ,154 ,155 ,156 ,157 ,158 ,158 ,159 ,160 ,161 ,162 ,163 ,164 ,164 ,166 ,167 ,167 ,169 ,170 ,170 ,171 ,172 ,173 ,174 ,175 ,176 ,176 ,178 ,178 ,180 ,181 ,182 ,183 ,184 ,184 ,186 ,186 ,188 ,188 ,189 ,191 ,191 ,192 ,194 ,194 ,196 ,197 ,197 ,199 ,200 ,201 ,202 ,203 ,204 ,205 ,207 ,207 ,208 ,209 ,210 ,212 ,213 ,214 ,215 ,216 ,217 ,218 ,220 ,221 ,222 ,223 ,224 ,225 ,226 ,228 ,229 ,230 ,231 ,233 ,234 ,235 ,236 ,238 ,239 ,240 ,241 ,242 ,244 ,245 ,246 ,248 ,249 ,250 ,251 ,253 ,254 ,255};

	auto image_display = ImageDisplay::make();
	auto image_display_pointer = image_display.get();
	image_display->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));
	igtlink_viewer->set_color_filter(SkColorFilters::Table(dicom_compliant_conversion));

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(igtlink_viewer) << std::move(image_display);
	displaycontainer->set_divisions({ 0.0 , 0.5 , 1.0 });

	processing = std::make_shared<ProcessingMessage>(image_display_pointer,igtlink_viewer_pointer);
	processing->port = 18944;

	auto start_connection_callback = [processing](Button* button, Press press ,ConfigDraw* config) {
		if (!processing->connection_status.value()) {
			processing->pool->submit("connection thread",[processing]() { processing->communicate();});
		}
		else {
			processing->attempt_stop();
			processing->connection_status.set(false);
		}
	};

	auto start_connection = Button::make("Connect",resources);
	start_connection->set_click_color(SK_ColorGRAY)
						.set_hover_color(SK_ColorDKGRAY)
						.set_waiting_color(SK_ColorBLACK)
						.set_size(SkRect::MakeWH(100, 80));
	start_connection->add_press_call(start_connection_callback);
	auto start_connection_pointer = start_connection.get();

	auto record_images = Button::make("Record",resources);
	record_images->set_click_color(SK_ColorGRAY)
					.set_hover_color(SK_ColorDKGRAY)
					.set_waiting_color(SkColorSetARGB(0xFF, 0x00, 0xFF, 0xFF))
					.set_size(SkRect::MakeWH(100, 80));
	record_images->add_press_call([processing](Button* button, Press press ,ConfigDraw* config){
		processing->record_images = !processing->record_images;
		if(processing->record_images)
			button->set_waiting_color(SkColorSetARGB(0xF0, 0x00, 0xFF, 0xFF));
		else
			button->set_waiting_color(SkColorSetARGB(0xFF, 0x00, 0xFF, 0xFF));
	});
	

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(start_connection) << std::move(record_images);
	buttoncontainer->set_shader_colors({SkColorSetRGB(225, 225, 225), SkColorSetRGB(246, 246, 246)});
	processing->button = start_connection_pointer;

	start_connection_pointer->set_waiting_color(SK_ColorRED);

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	return Page{std::move(widgetcontainer),SK_ColorBLACK};
}