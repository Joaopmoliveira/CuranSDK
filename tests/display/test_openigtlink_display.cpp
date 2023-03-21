#include "userinterface/Window.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include <csignal>
#include "utils/Logger.h"

void GetRandomTestMatrix(igtl::Matrix4x4& matrix)
{
	float position[3];
	float orientation[4];

	// random position
	static float phi = 0.0;
	position[0] = 50.0 * cos(phi);
	position[1] = 50.0 * sin(phi);
	position[2] = 50.0 * cos(phi);
	phi = phi + 0.2;

	// random orientation
	static float theta = 0.0;
	orientation[0] = 0.0;
	orientation[1] = 0.6666666666 * cos(theta);
	orientation[2] = 0.577350269189626;
	orientation[3] = 0.6666666666 * sin(theta);
	theta = theta + 0.1;

	//igtl::Matrix4x4 matrix;
	igtl::QuaternionToMatrix(orientation, matrix);

	matrix[0][3] = position[0];
	matrix[1][3] = position[1];
	matrix[2][3] = position[2];
}


void generate_transform_message(std::shared_ptr<curan::ui::OpenIGTLinkViewer> button) {
	igtl::TimeStamp::Pointer ts;
	ts = igtl::TimeStamp::New();
	size_t counter = 0;
	while(counter<10){
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
		std::memcpy(header_to_receive->GetPackPointer(), transMsg->GetPackPointer(), header_to_receive->GetPackSize());

		header_to_receive->Unpack();
		igtl::MessageBase::Pointer message_to_receive = igtl::MessageBase::New();
		message_to_receive->SetMessageHeader(header_to_receive);
		message_to_receive->AllocatePack();
		std::memcpy(message_to_receive->GetPackBodyPointer(), transMsg->GetPackBodyPointer(),transMsg->GetPackBodySize());

		button->process_message(message_to_receive);
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(500) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		++counter;
	}
	curan::utils::cout << "stopped to send data";
}

void GLFW_error(int error, const char* description)
{
	fputs(description, stdout);
}


int main() {
	try {
		{
			glfwSetErrorCallback(GLFW_error);
			using namespace curan::ui;
			std::unique_ptr<Context> context = std::make_unique<Context>();;
			DisplayParams param{ std::move(context),1200,800 };
			std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

			const char* fontFamily = nullptr;
			SkFontStyle fontStyle;
			sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
			sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

			SkFont text_font = SkFont(typeface, 10, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);

			OpenIGTLinkViewer::Info infor;
			infor.text_font = text_font;
			std::shared_ptr<OpenIGTLinkViewer> open_viwer = OpenIGTLinkViewer::make(infor);
			auto caldraw = open_viwer->draw();
			auto calsignal = open_viwer->call();
			SkRect rect = SkRect::MakeLTRB(40, 40, 1000, 700);
			open_viwer->set_position(rect);

			auto lamd = [open_viwer]() {
				generate_transform_message(open_viwer);
			};
			std::thread message_generator{ lamd };

			//generate_transform_message(open_viwer);

			while (!glfwWindowShouldClose(viewer->window)) {
				auto start = std::chrono::high_resolution_clock::now();
				SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
				SkCanvas* canvas = pointer_to_surface->getCanvas();
				canvas->drawColor(SK_ColorWHITE);
				caldraw(canvas);
				glfwPollEvents();
				auto signals = viewer->process_pending_signals();
				if (!signals.empty())
					calsignal(signals.back());
				bool val = viewer->swapBuffers();
				if (!val)
					curan::utils::cout << "failed to swap buffers\n";
				auto end = std::chrono::high_resolution_clock::now();
				std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
			}
			curan::utils::cout << "stopped window";
			message_generator.join();
		}
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}