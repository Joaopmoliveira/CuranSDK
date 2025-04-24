#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"
#include "utils/FileStructures.h"

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"

#include <random>
#include <optimization/WireCalibration.h>


/*
This executable requires:

1. Temporal calibration has been performed on the robot
which translates into a file called temporal calibration 
with the date at which the calibration was performed

And it generates a file called optimization_result 
which contains the calibrated ultrasound pose with respect to the flange of the robot
*/

int main(int argc, char* argv[]) {
	using namespace curan::ui;

	

	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context)};
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(processing,resources);

    curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};
	auto cali = calibration.homogeneous_transformation();
    std::cout << "using calibration matrix: (" << cali.rows() << "," << cali.cols() << ")\n"  << cali << std::endl;
    for (Eigen::Index row = 0; row < 4; ++row)
        for (Eigen::Index col = 0; col < 4; ++col)
           	processing->calibration_transformation(row, col) = cali(row, col);

	auto line_params = calibration.line_parameterization();
	for(size_t i = 0; i < line_params.size(); ++i){
		Eigen::Matrix<double,3,3> rot;
		curan::optim::custom_mat<double,3,3> rot_mat;
		curan::optim::vec2rot<double>(line_params[i].data(), rot_mat);
		std::memcpy(rot.data(),rot_mat.values,3*3*sizeof(double));
		processing->line_parameterization.emplace_back(rot,*(line_params[i].data() + 3));
	}

	page.update_page(viewer.get());

	ConfigDraw config{&page};

	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
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
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	processing->attempt_stop();
	std::cout << "trying to stop communication\n" << std::endl;

	return 0;
}