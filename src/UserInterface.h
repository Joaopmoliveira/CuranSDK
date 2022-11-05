#ifndef UserInterface_h_DEFINED
#define UserInterface_h_DEFINED

#include <functional>
#include <map>
#include <chrono>
#include <memory>
#include "DkUtilities.h"
#include <fstream>
#include <GLFW/glfw3.h>
#include "DkLibrary.h"
#include "PkLibrary.h"
#include "CkLibrary.h"

#ifdef WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

namespace curan {
	namespace ui {
		//************************* UserInterface.h ****************************************
	/*
	File with the source code related with the viewers used by the application.
	All code related with window managment should be present in this file. The classes
	in this file are wrappers around the DkLibrary source files, that add specific
	behaviour depending on the requirments.
	*/
	//*********************************************************************************

	/*
	The WindowBody class contains the base methods that all
	windows must posses to be compatible with the logic
	implemented in the window manager of the code.
	Namely an init method which initializes the any variable as necessary
	the run method which is called by a dedicated thread to draw 
	the events on screen.
	*/
		class WindowBody
		{
		protected:
			uint32_t page_index = 0;
			uint32_t overlay_page_index = 0;
			curan::display::Context* context = nullptr;
			curan::display::DisplayParams param;
			int window_identifier = -1;
			mutable std::mutex mut;
			SkPaint overlay_paint;
		public:

			curan::display::Window* viewer = nullptr;

			WindowBody(curan::display::Context* context, curan::display::DisplayParams param, int window_identifier);
			/*
			Method used to initialize any pages of the
			application and connected the correct callbacks
			to receive input from the user.
			*/
			virtual void init();
			/*
			Method called to start the event loop of
			the application.
			*/
			virtual void run();

			/*
			Method used to change the current page on display
			*/
			void change_page_index(uint32_t page_index);

			/*
			Method used to superimpose an overlay over the main
			page
			*/
			void add_overlay(uint32_t overlay_page_index);

			/*
			*/
			uint32_t get_page_index();

			/*
			Method used to obtain the unique identifier of
			the instance of the window
			*/
			int identifier();
		};

		/*
		The main window is suposed to be used by the physician
		to select the study of the specific patient that one
		wishes to use for his specific task. The choice of this
		task should also be done in this window.

		Thus the pages in this window are two:

		1 - The first page is related with image input from local
		files and selection of the correct study. [index 0]

		2 - Selection of the desired action to be performed on said
		images. This is basically an enumeration of the possible
		task that the software can perform. [index 1]

		3 - Options of loading local files
		*/
		class MainWindow : public WindowBody {

			std::shared_ptr<curan::display::ItemPreview> managed_images;
			std::vector<std::string> array_of_files_to_load;
		public:

			MainWindow(curan::display::Context* in_context, int window_identifier, curan::display::DisplayParams param);
			void init() override;
			void run() override;
			static std::shared_ptr<MainWindow> make(curan::display::Context* in_context, int window_identifier, curan::display::DisplayParams param);

		private:
			//  ------- Methods used to create the pages of the window ----------//
			/*
			Creates the main page where the user can see
			the studies currently uploaded into memory and
			they can preview the studies to select the ones
			needed for the surgical procedures.
			*/
			void create_load_file_page(std::shared_ptr<curan::display::Page>& first_page);
			/*
			Creates the page which lists the currently available
			tasks which can be performed by the software, i.e.
			view a medical file, perform a biopsy with the surgical
			aparatus, etc...
			*/
			void create_tasks_page(std::shared_ptr<curan::display::Page>& second_page);
			/*
			Creates an overlay which will be superimposed over
			the first page. This overlay indicates to the user
			how he should upload images into the local repository
			*/
			void create_load_file_overlay(std::shared_ptr<curan::display::Page>& third_page);

			/*
			Creates an overlay with the possible errors that can appear
			throught the execution of the UIMainWindow page.
			*/
			void create_error_feedback_page(std::shared_ptr<curan::display::Page>& fourth_page);
		};

		/*
		The biopsy viewer is a window which contains all
		functionality required by the surgical intervention.
		The viewer is divided in stages were the stages are:

		1 - The planning stage - The physician must have the
		tools to define the trajectory required in the
		surgical procedure. This stage in layout should be simillar
		to the medical viewer, without the filters and extra
		material which should not be required

		2 - Verification of the robot motion -  along the planned
		trajectory. If the motion is invalid, then the robot
		must be repositioned so that everything works.

		3 - System connection verification - This stage provides feedback
		on the current connections of the system.

		4 - Ultrasound calibration - The ultrasound calibration
		page contains a viewer of the ultrasounds currently received
		in the system

		5 - Registration procedure - The registration procedure
		stage allows the surgeon to find the current position of the
		cranium in relation to the base of the cranium

		6 - Validation of the registration procedure - This
		stage involves the surgeon moving the probe installed on
		the robot to a given set of points in the patients cranium to
		validate the registration

		7 - The execution stage involves showing the surgeons the fused medical images
		with feedback in real time from the robotic system
		*/
		class BiopsyViewer : public WindowBody {
			curan::image::Study current_volume;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> dinamic_image_display;
			std::shared_ptr<curan::display::ThreeWayButton> stage_button;
			std::shared_ptr<curan::display::StaticImageDisplay> calibration_image_display;

		public:
			BiopsyViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
			void init() override;
			void run() override;
			static std::shared_ptr<BiopsyViewer> make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
		private:
			void create_image_display_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_validation_robot_motion(std::shared_ptr<curan::display::Page>& image_page);
			void create_communication_validation_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_calibration_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_registration_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_registration_validation_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_intraoperative_page(std::shared_ptr<curan::display::Page>& image_page);
		};

		/*
		The medical viewer is a window which allows the
		medical team to manipulate the images, so that
		they can manipulate the images through filters
		and inspected details in a non planning stage of the
		surgical procedure.
		*/
		class MedicalViewer : public WindowBody
		{
			curan::image::Study current_volume;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> dinamic_image_display;
		public:
			MedicalViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
			void init() override;
			void run() override;
			static std::shared_ptr<MedicalViewer> make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
			void change_layout(curan::display::ImageDisplayDinamicLayout::LayoutState new_layout);
		private:
			void create_image_display_page(std::shared_ptr<curan::display::Page>& image_page);
			void create_overlay_image_display(std::shared_ptr<curan::display::Page>& image_page);
		};

		/*
		*/
		class FAIViewer : public WindowBody
		{
			curan::image::Study current_volume;
			std::shared_ptr<curan::display::ThreeWayButton> stage_button;
			std::shared_ptr<curan::display::ImageDisplayDinamicLayout> dinamic_image_display;
		public:
			FAIViewer(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
			void init() override;
			void run() override;
			static std::shared_ptr<FAIViewer> make(curan::display::Context* in_context, int window_identifier, uint64_t volume_index, curan::display::DisplayParams param);
		private:
			void create_segmentation_page(std::shared_ptr<curan::display::Page>& page);
			void statistical_femur_comparison(std::shared_ptr<curan::display::Page>& page);
			void create_overlay_image_display(std::shared_ptr<curan::display::Page>& image_page);
			void create_segmentation_option_box(std::shared_ptr<curan::display::Page>& image_page,std::shared_ptr<curan::display::ImageDisplayDinamicLayout> associated_display);
		};

		/*
		The WindowManager takes feedback from all windows and
		when all windows are closed the instances should inform 
		the window manager through the unregister window  method which will
		decrement a counter,when said counter reaches zero the window manager 
		will return from the process_os_events() method.
		
		The logic behind this class is the following. The window manager is created,

		The GLFW library requires that the glfwPollEvent() function
		to be called from the main thread. Therefore windows shoud
		register themself's with the WindowManager and be started
		with their run method while the process_os_events() method is
		called in the main thread.
		*/
		class WindowManager {
		private:
			int window_identifier = 1;
			std::atomic<bool> window_manager_active = true;
			uint32_t number_of_active_windows = 0;
			mutable std::mutex mut;
			std::map<int, std::shared_ptr<WindowBody>> contained_windows;
			curan::display::Context* context = nullptr;
			WindowManager();
		public:
			void register_window(std::shared_ptr<WindowBody> new_window);
			void unregister_window(int identifier_old_window);
			void process_os_events();
			void lauch_main_window();
			void lauch_medical_visualizer(int volume_identifier);
			void lauch_biopsy_visualizer(int volume_identifier);
			void lauch_faiviewer_visualizer(int volume_identifier);
			int next_window_identifier();
			void set_context(curan::display::Context* in_context);
			static WindowManager* Get();
		};
	}
}
#endif
