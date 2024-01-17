#include <Eigen/Dense>
#include <iostream>


void create_vol_vertices(Eigen::Matrix<double,8,3>& vol_vertices_coords, Eigen::Matrix<double,3,1> volume_size) {

    vol_vertices_coords(0,0) = 0.0;
    vol_vertices_coords(0,1) = 0.0;
    vol_vertices_coords(0,2) = 0.0;
    vol_vertices_coords(1,0) = volume_size[0];
    vol_vertices_coords(1,1) = 0.0;
    vol_vertices_coords(1,2) = 0.0;
    vol_vertices_coords(2,0) = volume_size[0];
    vol_vertices_coords(2,1) = volume_size[1];
    vol_vertices_coords(2,2) = 0.0;
    vol_vertices_coords(3,0) = 0.0;
    vol_vertices_coords(3,1) = volume_size[1];
    vol_vertices_coords(3,2) = 0.0;
    vol_vertices_coords(4,0) = 0.0;
    vol_vertices_coords(4,1) = 0.0;
    vol_vertices_coords(4,2) = volume_size[2];
    vol_vertices_coords(5,0) = volume_size[0];
    vol_vertices_coords(5,1) = 0.0;
    vol_vertices_coords(5,2) = volume_size[2];
    vol_vertices_coords(6,0) = volume_size[0];
    vol_vertices_coords(6,1) = volume_size[1];
    vol_vertices_coords(6,2) = volume_size[2];
    vol_vertices_coords(7,0) = 0.0;
    vol_vertices_coords(7,1) = volume_size[1];
    vol_vertices_coords(7,2) = volume_size[2];

}


void exclude_row_matrix(Eigen::MatrixXd& matrix, size_t row_to_remove){
    int numRows = static_cast<int>(matrix.rows()) - 1;
    int numCols = static_cast<int>(matrix.cols());

    if(row_to_remove < numRows){
        matrix.block(row_to_remove,0,numRows-row_to_remove,numCols) = matrix.bottomRows(numRows-row_to_remove);
    }

    matrix.conservativeResize(numRows,numCols);
}


void obtain_rot_matrix_by_angles(Eigen::Matrix<double,3,3>& R_matrix, Eigen::Matrix<double,3,1>& rotaton_angles){

    typedef double T;

    auto a = rotaton_angles[0];
    auto b = rotaton_angles[1];
    auto c = rotaton_angles[2];

    R_matrix << cos(b)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c),
            cos(b)*sin(c), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), cos(a)*sin(b)*sin(c)-sin(a)*cos(c),
            -sin(b), sin(a)*cos(b), cos(a)*cos(b);
}



int main(){
    Eigen::Matrix<double,3,1> volume_size;
    Eigen::Matrix<double,8,3> vol_vertices_coords;

    volume_size[0] = 1.0;
    volume_size[1] = 2.0;
    volume_size[2] = 3.0;


    create_vol_vertices(vol_vertices_coords, volume_size);


    Eigen::Matrix<double,3,3> R_matrix;
    Eigen::Matrix<double,3,1> rotaton_angles;

    rotaton_angles[0] = 1.3;
    rotaton_angles[1] = -2.6;
    rotaton_angles[2] = 0.08;


    obtain_rot_matrix_by_angles(R_matrix,rotaton_angles);


    Eigen::Matrix<double,3,1> needle_tip;

    needle_tip[0] = 0.5;
    needle_tip[1] = 1.0;
    needle_tip[2] = 1.5;

    double d = -(R_matrix.col(2).transpose()*needle_tip)(0,0);

    Eigen::Matrix<double,4,1> plane_equation;
    plane_equation[0] = R_matrix(0,2);
    plane_equation[1] = R_matrix(1,2);
    plane_equation[2] = R_matrix(2,2);
    plane_equation[3] = d;

    Eigen::MatrixXd intersections;
    intersections.resize(12,3);

    double EPS = 1e-15;

    size_t i = 0;
    for(size_t j = 0; j < static_cast<int>(vol_vertices_coords.rows()); ++j){
        if(vol_vertices_coords(j,0) == 0){
            if(std::sqrt(std::pow(plane_equation[0],2)) > EPS){
                intersections(i,0) = -(plane_equation[1]*vol_vertices_coords(j,1) + plane_equation[2]*vol_vertices_coords(j,2) + plane_equation[3]) / plane_equation[0];
            }else{
                intersections(i,0) = 1/EPS;
            }
            intersections(i,1) = vol_vertices_coords(j,1);
            intersections(i,2) = vol_vertices_coords(j,2);
            i = i+1;

        }
        if(vol_vertices_coords(j,1) == 0){
            if(std::sqrt(std::pow(plane_equation[1],2)) > EPS){
                intersections(i,1) = -(plane_equation[0]*vol_vertices_coords(j,0) + plane_equation[2]*vol_vertices_coords(j,2) + plane_equation[3]) / plane_equation[1];
            }else{
                intersections(i,1) = 1/EPS;
            }
            intersections(i,0) = vol_vertices_coords(j,0);
            intersections(i,2) = vol_vertices_coords(j,2);
            i = i+1;

        }

        if(vol_vertices_coords(j,2) == 0){
            if(std::sqrt(std::pow(plane_equation[2],2)) > EPS){
                intersections(i,2) = -(plane_equation[0]*vol_vertices_coords(j,0) + plane_equation[1]*vol_vertices_coords(j,1) + plane_equation[3]) / plane_equation[2];
            }else{
                intersections(i,2) = 1/EPS;
            }
            intersections(i,0) = vol_vertices_coords(j,0);
            intersections(i,1) = vol_vertices_coords(j,1);
            i = i+1;

        }
    }

    std::cout << "\n The intersections are: \n" << intersections << std::endl;
     

    size_t one = 1;
    size_t initial_number_of_points = static_cast<int>(intersections.rows());
    for(size_t i = 0 ; i< initial_number_of_points; ++i){

        if(intersections(initial_number_of_points-one-i,0) > volume_size[0] + EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);

        } else if(intersections(initial_number_of_points-one-i,0) < 0.0 - EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);

        } else if(intersections(initial_number_of_points-one-i,1) > volume_size[1] + EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);

        } else if(intersections(initial_number_of_points-one-i,1) < 0.0 - EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);

        } else if(intersections(initial_number_of_points-one-i,2) > volume_size[2] + EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);

        } else if(intersections(initial_number_of_points-one-i,2) < 0.0 - EPS){
            exclude_row_matrix(intersections, initial_number_of_points-one-i);
        }
    }

    std::cout << "\n The intersections are: \n" << intersections << std::endl;

    Eigen::Matrix<double,3,1> centroid;

    centroid[0] = intersections.col(0).sum() / static_cast<int>(intersections.rows());
    centroid[1] = intersections.col(1).sum() / static_cast<int>(intersections.rows());
    centroid[2] = intersections.col(2).sum() / static_cast<int>(intersections.rows());

    std::cout << "\n The centroid is: \n" << centroid << std::endl;

    double image_size = std::sqrt(std::pow(volume_size[0],2)+std::pow(volume_size[1],2)+std::pow(volume_size[2],2));

    std::cout << "\n The image_size is: \n" << image_size << std::endl;


    Eigen::Matrix<double,3,1> image_spacing;
    image_spacing[0] = 1.0;
    image_spacing[1] = 1.0;
    image_spacing[2] = 1.0;

    Eigen::Matrix<double,3,1> corner_to_centroid_image_vector;
    corner_to_centroid_image_vector[0] = image_size * image_spacing[0] * 0.5;
    corner_to_centroid_image_vector[1] = image_size * image_spacing[1] * 0.5;
    corner_to_centroid_image_vector[2] = 0.0;


    Eigen::Matrix<double,3,1> image_origin;

    image_origin = centroid - R_matrix * corner_to_centroid_image_vector;

    std::cout << "The image origin is: " << image_origin << std::endl;


    return 0;
}