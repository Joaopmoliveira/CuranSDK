#include <Eigen/Dense>

int main(){
    Eigen::Matrix<double,3,1> volume_size;
    Eigen::Matrix<double,8,3> coord;

    double a = 1.0;
    double b = 1.0;
    double c = 1.0;

    Eigen::Matrix<double,3,3> R_matrix = Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,3,1> needle_tip;

    double d = -(R_matrix.col(2).transpose()*needle_tip)(0,0);
    Eigen::Matrix<double,4,1> plane_equation;
    Eigen::Matrix<double,12,3> intersections;
    Eigen::Matrix<double,12,3> exclusion_matrix;

    size_t i = 0;
    for(size_t j = 0; j< coord.rows(); ++j){
        if(coord(j,0) == 0){

        }
        if(coord(j,1) == 0){

        }

        if(coord(j,2) == 0){

        }
    }

    size_t initial_number_of_points = exclusion_matrix.rows();
    for(size_t i = 0 ; i< initial_number_of_points; ++i){
        if(intersections(initial_number_of_points-1-i,0) > volume_size[0]){

        } else if(intersections(initial_number_of_points-1-i,0) < 0.0){

        } else if(intersections(initial_number_of_points-1-i,1) > volume_size[1]){

        } else if(intersections(initial_number_of_points-1-i,1) < 0.0){

        } else if(intersections(initial_number_of_points-1-i,2) > volume_size[2]){

        } else if(intersections(initial_number_of_points-1-i,2) < 0.0){

        }
    }

    Eigen::Matrix<double,3,1> centroid;
    double image_size = std::sqrt(std::pow(volume_size[0],2)+std::pow(volume_size[1],2)+std::pow(volume_size[2],2));


    Eigen::Matrix<double,3,1> image_spacing;
    Eigen::Matrix<double,3,1> image_centroid;

    Eigen::Matrix<double,3,1> image_origin;

    return 0;
}