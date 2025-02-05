#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/Window.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "utils/TheadPool.h"
#include <iostream>
#include <functional>

#include <array>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <iostream>

constexpr double noise_level = 0.0;
constexpr double spacing = 0.05/20;

template <size_t num_lines>
struct phantom
{
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> p;
    double dx = 0.05;
    double dy = 0.2;
    double dz = 0.05;
    Eigen::Matrix<double,num_lines,6> unprocessed_data = Eigen::Matrix<double, num_lines,6>::Zero();
    Eigen::Matrix<double,num_lines,6> transformed_line_data = Eigen::Matrix<double, num_lines,6>::Zero();

    consteval inline size_t get_num_lines()
    {
        return num_lines;
    }

    void transform_line_data()
    {
        for (size_t i = 0; i < num_lines; ++i){
            if (unprocessed_data(i, 1)< -1e-10 || unprocessed_data(i, 4)> 1e-10 +dy )
                throw std::runtime_error("The two points that define the line in the phantom do not belong to the plane Oxz or the Oxz+length_y");
            if ((!(unprocessed_data(i, 0) > -1e-10 && unprocessed_data(i, 0) <= dx+1e-10)) || (!(unprocessed_data(i, 2) >= -1e-10 && unprocessed_data(i, 2) <= dx+1e-10)))
                throw std::runtime_error("A point from one of the lines is not contained in the area of the phantom in the Oxz plane");
            if ((!(unprocessed_data(i, 3) >= -1e-10 && unprocessed_data(i, 3) <= dz+1e-10)) || (!(unprocessed_data(i, 5) >= -1e-10 && unprocessed_data(i, 5) <= dz+1e-10)))
                throw std::runtime_error("A point from one of the lines is not contained in the area of the phantom in the Oxz plane+ length_y");
            Eigen::Matrix<double,1,3> vec_line = unprocessed_data.block<1,3>(i,3)-unprocessed_data.block<1,3>(i,0);
            transformed_line_data.block<1,3>(i,0) = ((R*vec_line.transpose())*(1.0/vec_line.norm())).transpose();
            transformed_line_data.block<1,3>(i,3) = (p+R*(unprocessed_data.block<1,3>(i,0).transpose())).transpose();
        }   
    }
};

Eigen::RowVectorXd f_spline(Eigen::RowVectorXd nodes, size_t n_points)
{
    Eigen::RowVectorXd time;
    time.resize(nodes.size());
    double time_scalar = 0.0;
    size_t i = 0;
    for (i = 0; i < nodes.size(); ++i, time_scalar += 1.0 / (nodes.size()))
        time(i) = time_scalar;
    Eigen::Spline<double, 1, 5> fit;
    switch(nodes.size()){
        case 1:
            throw std::runtime_error("cannot interpolate with one point, infinite solutions");
        case 2: 
        case 3:
        case 4:
        {
            fit = Eigen::SplineFitting<Eigen::Spline<double, 1, 3>>::Interpolate(nodes, 3, time);
            Eigen::Spline<double, 1, 3> spline(fit);
            Eigen::RowVectorXd fitted_out;
            fitted_out.resize(n_points);
            time_scalar = 0.0;
            for (i = 0; i < n_points; ++i, time_scalar += 1.0 / (n_points))
                fitted_out(i) = spline(time_scalar).coeff(0);
            return fitted_out;
        }
            break;
        case 5:
        case 6:
        {
            fit = Eigen::SplineFitting<Eigen::Spline<double, 1, 3>>::Interpolate(nodes, 3, time);
            Eigen::Spline<double, 1, 3> spline(fit);
            Eigen::RowVectorXd fitted_out;
            fitted_out.resize(n_points);
            time_scalar = 0.0;
            for (i = 0; i < n_points; ++i, time_scalar += 1.0 / (n_points))
                fitted_out(i) = spline(time_scalar).coeff(0);
            return fitted_out;
        }
            break;
        default: 
        {
            fit = Eigen::SplineFitting<Eigen::Spline<double, 1, 3>>::Interpolate(nodes, 3, time);
            Eigen::Spline<double, 1, 3> spline(fit);
            Eigen::RowVectorXd fitted_out;
            fitted_out.resize(n_points);
            time_scalar = 0.0;
            for (i = 0; i < n_points; ++i, time_scalar += 1.0 / (n_points))
                fitted_out(i) = spline(time_scalar).coeff(0);
            return fitted_out;
        }
            break;
    }

}

Eigen::Matrix<double, 3, 3> eul2rot(double a, double b, double c)
{
    Eigen::Matrix<double, 3, 3> R;
     
    double t1 = cos(c);
    double t2 = sin(c);
    double t3 = cos(b);
    double t4 = sin(b);
    double t5 = cos(a);
    double t6 = sin(a);

    R(0,0) = t1 * t3;
    R(1,0) = t2 * t3;
    R(2,0) = -t4;

    R(0,1) = t1 * t4 * t6 - t2 * t5;
    R(1,1) = t1 * t5 + t2 * t4 * t6;
    R(2,1) = t3 * t6;

    R(0,2) = t2 * t6 + t1 * t4 * t5;
    R(1,2) = t2 * t4 * t5 - t1 * t6;
    R(2,2) = t3 * t5;

    return R;
}

Eigen::Matrix<double, 3, 1> compute_intersection(Eigen::Matrix<double, 4, 4> T01, Eigen::Matrix<double, 4, 4> T12, Eigen::Matrix<double, 1, 6> transformed_line_data)
{
    Eigen::Matrix<double, 4, 4> T02 = T01 * T12;
    Eigen::Matrix<double, 3, 1> perpVec_to_imag_plane = T02.block<3, 1>(0, 3);
    Eigen::Matrix<double, 1, 3> parallel_line_vec = transformed_line_data.block<1, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> point_of_line = transformed_line_data.block<1, 3>(0, 3).transpose();
    double plane_d = T02.block<3, 1>(0, 3).transpose() * perpVec_to_imag_plane;
    double K = std::abs(parallel_line_vec * perpVec_to_imag_plane)> 1e-5 ? (1.0 / (parallel_line_vec * perpVec_to_imag_plane)) * (plane_d - point_of_line.transpose() * perpVec_to_imag_plane) : (1.0 / 1e-5) * (plane_d - point_of_line.transpose() * perpVec_to_imag_plane);
    Eigen::Matrix<double, 3, 1> p03 = point_of_line + K * parallel_line_vec.transpose();
    Eigen::Matrix<double, 3, 1> p23 = T12.block<3, 3>(0, 0).transpose() * (T01.block<3, 3>(0, 0).transpose() * (p03 - T01.block<3, 1>(0, 3)) - T12.block<3, 1>(0, 3));
    return p23;
}

template <size_t n_points, size_t num_lines>
struct trajectory
{
    Eigen::RowVectorXd angle_x;
    Eigen::RowVectorXd angle_y;
    Eigen::RowVectorXd angle_z;
    Eigen::RowVectorXd motion_x;
    Eigen::RowVectorXd motion_y;
    Eigen::RowVectorXd motion_z;
    std::array<Eigen::Matrix<double, 4, 4>, n_points> transforms;
    std::array<Eigen::Matrix<double, 3, num_lines>, n_points> wire_poses;

    void interpolate_flange_trajectory(Eigen::Matrix<double, 3, 1> p12, Eigen::Matrix<double, 3, 3> R12, phantom<num_lines> phant)
    {
        transforms.fill(Eigen::Matrix<double, 4, 4>::Identity());
        wire_poses.fill(Eigen::Matrix<double, 3, num_lines>::Zero());

        Eigen::Matrix<double, 4, 4> T_flange_to_image = Eigen::Matrix<double, 4, 4>::Identity();
        T_flange_to_image.block<3, 3>(0, 0) = R12.transpose();
        T_flange_to_image.block<3, 1>(0, 3) = -R12.transpose() * p12;

        Eigen::Matrix<double, 4, 4> T_phantom_to_base = Eigen::Matrix<double, 4, 4>::Identity();
        T_phantom_to_base.block<3, 3>(0, 0) = phant.R;
        T_phantom_to_base.block<3, 1>(0, 3) = phant.p;

        auto local_distance_x = f_spline(motion_x, n_points) * phant.dx;
        auto local_distance_y = f_spline(motion_y, n_points) * phant.dy;
        auto local_distance_z = f_spline(motion_z, n_points) * phant.dz;

        auto local_angle_x = f_spline(angle_x, n_points);
        auto local_angle_y = f_spline(angle_y, n_points);
        auto local_angle_z = f_spline(angle_z, n_points);

        std::cout << "distance_x: " << local_distance_x << std::endl;
        std::cout << "distance_y: " << local_distance_y << std::endl;
        std::cout << "distance_z: " << local_distance_z << std::endl;
        std::cout << "angle_x: " << local_angle_x << std::endl;
        std::cout << "angle_y: " << local_angle_y << std::endl;
        std::cout << "angle_z: " << local_angle_z << std::endl;

        for (size_t i = 0; i < n_points; ++i)
        {
            auto R_image_to_phantom = eul2rot(local_angle_x(i), local_angle_y(i), local_angle_z(i));
            Eigen::Matrix<double, 4, 4> T_image_to_phantom = Eigen::Matrix<double, 4, 4>::Identity();
            T_image_to_phantom(0, 3) = local_distance_x(i);
            T_image_to_phantom(1, 3) = local_distance_y(i);
            T_image_to_phantom(2, 3) = local_distance_z(i);
            T_image_to_phantom.block<3, 3>(0, 0) = R_image_to_phantom;
            transforms[i] = T_phantom_to_base * T_image_to_phantom * T_flange_to_image;
        }
    }

    void compute_multiple_intersections(Eigen::Matrix<double, 4, 4> T12, phantom<num_lines> phant, std::function<void(Eigen::Matrix<double, 3, 3>)> arg)
    {
        Eigen::Matrix<double, 4, 1>  wire_pose_j = Eigen::Matrix<double, 4, 1>::Ones(); 
        for(size_t i = 0; i < n_points; ++i){
            for(size_t j = 0; j < num_lines; ++j){
                //std::cout << "here 1\n";
                wire_poses[i] = Eigen::Matrix<double, 3, num_lines>::Zero();
                wire_pose_j.block<3,1>(0,0) = compute_intersection(transforms[i], T12, phant.transformed_line_data.block<1,6>(j,0));
                //std::cout << "here 2\n";
                Eigen::Matrix<double, 4, 1> p03 = transforms[i]*T12*wire_pose_j;
                Eigen::Matrix<double, 3, 1> pdiff = phant.R.transpose()*(p03.block<3,1>(0,0)-phant.p);
                bool is_inf = false;
                if(pdiff[0]-1e-10>phant.dx || pdiff[0]+1e-10<0)       
                    is_inf = true;
                else if(pdiff(1)-1e-10>phant.dy || pdiff[1]+1e-10<0)
                    is_inf = true;
                else if(pdiff(2)-1e-10>phant.dz || pdiff(2)+1e-10<0)
                    is_inf = true;
                if(is_inf)
                    wire_poses[i].block<3, 1>(0, j) = Eigen::Matrix<double, 3, 1>{1e10,1e10,1e10};
                else
                    wire_poses[i].block<3, 1>(0, j) = wire_pose_j.block<3,1>(0,0);
                //std::cout << "here: "  << wire_pose_j.block<3,1>(0,0).transpose() << std::endl;
                
            }
            arg(wire_poses[i]);    
            std::this_thread::sleep_for(std::chrono::milliseconds(32));
        }
    }
};

void simulate(curan::ui::ImageDisplay *image_display,size_t image_width, size_t image_height)
{
    const std::array<double, 3> theta_2{0.2, 0.2, 0.2};

    Eigen::Matrix<double, 3, 1> p12 = Eigen::Matrix<double, 3, 1>{0.1, 0.2, 0.15};
    Eigen::Matrix<double, 3, 3> R12 = eul2rot(theta_2[0],theta_2[1],theta_2[2]);

    phantom<3> data;
    data.R = eul2rot(0,0,10);
    data.p= Eigen::Matrix<double, 3, 1>{0.1 , 0.1 , 0.1};

    std::cout << "phatom p:" << data.p.transpose() << std::endl;
    std::cout << "phatom R:\n" << data.R << std::endl;

    std::cout << "p12:" << p12.transpose() << std::endl;
    std::cout << "R12:\n" << R12 << std::endl;

    trajectory<50, data.get_num_lines()> traject;
    traject.angle_x = Eigen::RowVectorXd(5);
    traject.angle_x << -1.7453 ,  -1.5708 ,  -1.2217  , -1.5708  , -1.7453;
    traject.angle_y = Eigen::RowVectorXd(5);
    traject.angle_y  << 0  ,  0.5236     ,    0  , -0.5236     ,    0;
    traject.angle_z = Eigen::RowVectorXd(5);
    traject.angle_z  << 0  ,  0.1745     ,    0  , -0.1745     ,    0;
    traject.motion_x = Eigen::RowVectorXd(5);
    traject.motion_x  << 0 ,0 , 0.1 , 0, 0; 
    traject.motion_y = Eigen::RowVectorXd(5);
    traject.motion_y  << 0 , 1 , 0 , 1 ,0; 
    traject.motion_z = Eigen::RowVectorXd(5);
    traject.motion_z  << 0 , 0 ,  0 ,0, 0; 

    switch (data.get_num_lines())
    {
    case 1:
    {
        data.unprocessed_data.block<1, 6>(0, 0) = Eigen::Matrix<double, 1, 6>{0.025, 0.0, 0.025, 0.025, data.dy, 0.025};
    }
    break;
    case 2:
    {
        data.unprocessed_data.block<1, 6>(0, 0) = Eigen::Matrix<double, 1, 6>{0.0, 0.0, 0.0, 0.0, data.dy, 0.0};
        data.unprocessed_data.block<1, 6>(1, 0) = Eigen::Matrix<double, 1, 6>{0.05, 0.0, 0.05, 0.05, data.dy, 0.05};
    }
    break;
    case 3:
    {
        data.unprocessed_data.block<1, 6>(0, 0) = Eigen::Matrix<double, 1, 6>{0.0, 0.0, 0.0, 0.0, data.dy, 0.0};
        data.unprocessed_data.block<1, 6>(1, 0) = Eigen::Matrix<double, 1, 6>{0.05, 0.0, 0.05, 0.05, data.dy, 0.05};
        data.unprocessed_data.block<1, 6>(2, 0) = Eigen::Matrix<double, 1, 6>{0.0, 0.0, 0.05, 0.0, data.dy, 0.05};
    }
    break;
    case 4:
    {
        data.unprocessed_data.block<1, 6>(0, 0) = Eigen::Matrix<double, 1, 6>{0.0, 0.0, 0.0, 0.0, data.dy, 0.0};
        data.unprocessed_data.block<1, 6>(1, 0) = Eigen::Matrix<double, 1, 6>{0.05, 0.0, 0.05, 0.05, data.dy, 0.05};
        data.unprocessed_data.block<1, 6>(2, 0) = Eigen::Matrix<double, 1, 6>{0.0, 0.0, 0.05, 0.0, data.dy, 0.05};
        data.unprocessed_data.block<1, 6>(3, 0) = Eigen::Matrix<double, 1, 6>{0.05, 0.0, 0.0, 0.05, data.dy, 0.0};
    }
    break;
    default:
        throw std::runtime_error("cannot simulate more wires for now");
    }
    data.transform_line_data();
    traject.interpolate_flange_trajectory(p12, R12, data);
    std::cout << "line_data:\n" <<  data.transformed_line_data << std::endl;
    Eigen::Matrix<double, 4, 4> T12 = Eigen::Matrix<double, 4, 4>::Identity();
    T12.block<3,3>(0,0) = R12;
    T12.block<3,1>(0,3) = p12; 
    
    traject.compute_multiple_intersections(T12,data,[=](Eigen::Matrix<double, 3, 3> point){
        using namespace curan::ui;
        using namespace curan::utilities;

        auto raw_data = std::make_shared<std::vector<uint8_t>>(image_width * image_height, 0);
        for (auto &dat : *raw_data.get())
            dat = 0;

        for(size_t i = 0; i < 3 ; ++i){
            double pixel_x = point(0,i)/spacing;
            double pixel_y = point(1,i)/spacing;

            if(pixel_x>0 && pixel_x < image_width && pixel_y > 0 && pixel_y < image_height)
                if(pixel_x+image_width*pixel_y < raw_data->size()){
                    std::cout << "print!\n";
                    (*raw_data)[pixel_x+image_width*pixel_y] = 255;
                }
                    
                   
        }

        image_display->update_image(ImageWrapper{CaptureBuffer::make_shared(raw_data->data(),raw_data->size() * sizeof(uint8_t), raw_data),image_width, image_height});
    });
    
}

void image_display_tutorial() {
  using namespace curan::ui;
  using namespace curan::utilities;
  std::unique_ptr<Context> context = std::make_unique<Context>();
  DisplayParams param{std::move(context), 1200, 1200};
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
    size_t image_width = 100;
    size_t image_height = 100;
    while (running) {
      simulate(pointer_to,image_width,image_height);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(16) -std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
  }
  running = false;
  return;
}

int main()
{
    try
    {
        image_display_tutorial();
        return 0;
    }
    catch (...)
    {
        return 1;
    }
}
