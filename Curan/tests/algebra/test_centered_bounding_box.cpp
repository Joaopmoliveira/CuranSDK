#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

enum Strategy{
    CONSERVATIVE,   
};

struct BoundingBox{
    Eigen::Matrix<double,3,1> origin;
    Eigen::Matrix<double,3,3> orientation;
    Eigen::Matrix<double,3,1> size;
    Eigen::Matrix<double,3,1> spacing;

    BoundingBox(const Eigen::Matrix<double,3,1>& in_origin,const Eigen::Matrix<double,3,1>& along_x,Eigen::Matrix<double,3,1> along_y,Eigen::Matrix<double,3,1> along_z, Eigen::Matrix<double,3,1> in_spacing){
        origin = in_origin;
        Eigen::Matrix<double,3,1> direct_x = along_x-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_x = direct_x;
        Eigen::Matrix<double,3,1> direct_y = along_y-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_y = direct_y;
        Eigen::Matrix<double,3,1> direct_z = along_z-in_origin;
        Eigen::Matrix<double,3,1> vector_along_direction_z = direct_z;
        direct_x.normalize();
        direct_y.normalize();
        direct_z.normalize();
        orientation.col(0) = direct_x;
        orientation.col(1) = direct_y;
        orientation.col(2) = direct_z;

        assert(orientation.determinant()>0.9999 && orientation.determinant()<1.0001 && "failure to generate an ortogonal rotation matrix");
        spacing = in_spacing;
        size[0] = vector_along_direction_x.norm()/spacing[0];
        size[1] = vector_along_direction_y.norm()/spacing[1];
        size[2] = vector_along_direction_z.norm()/spacing[2];
    }

    friend std::ostream & operator << (std::ostream &, const BoundingBox &);

    template<Strategy prefered_strategy,bool debug>
    BoundingBox centered_bounding_box(const Eigen::Matrix<double,3,3>& relative_transform){

        if(debug) std::cout << "\ndebug info: (relative_transform)\n" <<  relative_transform;

        Eigen::Matrix<double,3,8> corners_in_rotated_space;
        corners_in_rotated_space.col(0)[0] = 0;
        corners_in_rotated_space.col(0)[1] = 0;
        corners_in_rotated_space.col(0)[2] = 0;

        corners_in_rotated_space.col(1)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(1)[1] = 0;
        corners_in_rotated_space.col(1)[2] = 0;

        corners_in_rotated_space.col(2)[0] = 0;
        corners_in_rotated_space.col(2)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(2)[2] = 0;

        corners_in_rotated_space.col(3)[0] = 0;
        corners_in_rotated_space.col(3)[1] = 0;
        corners_in_rotated_space.col(3)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(4)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(4)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(4)[2] = 0;

        corners_in_rotated_space.col(5)[0] = 0;
        corners_in_rotated_space.col(5)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(5)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(6)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(6)[1] = 0;
        corners_in_rotated_space.col(6)[2] = spacing[2]*size[2];

        corners_in_rotated_space.col(7)[0] = spacing[0]*size[0];
        corners_in_rotated_space.col(7)[1] = spacing[1]*size[1];
        corners_in_rotated_space.col(7)[2] = spacing[2]*size[2];

        if(debug) std::cout << "\ndebug info: (corners_in_rotated_space)\n" <<  corners_in_rotated_space;

        Eigen::Matrix<double,3,8> transformed_corners_in_rotated_space;
        for(size_t col = 0; col < static_cast<size_t>(transformed_corners_in_rotated_space.cols()); ++col)
            transformed_corners_in_rotated_space.col(col) = relative_transform.transpose()*corners_in_rotated_space.col(col);

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> minimum = transformed_corners_in_rotated_space.rowwise().minCoeff();

        if(debug) std::cout << "\ndebug info: (minimum)\n" <<  transformed_corners_in_rotated_space.rowwise().minCoeff();

        transformed_corners_in_rotated_space.colwise() -=minimum;

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_rotated_space)\n" <<  transformed_corners_in_rotated_space;

        Eigen::Matrix<double,3,1> unrounded_transformed_size = transformed_corners_in_rotated_space.rowwise().maxCoeff();

        if(debug) std::cout << "\ndebug info: (unrounded_transformed_size)\n" <<  unrounded_transformed_size;

        Eigen::Matrix<double,3,1> transformed_spacing = spacing;
        transformed_spacing.fill(spacing.minCoeff());

        if(debug) std::cout << "\ndebug info: (transformed_spacing)\n" <<  transformed_spacing;

        Eigen::Matrix<double,3,1> transformed_size;
        transformed_size[0] = std::ceil(unrounded_transformed_size[0]);
        transformed_size[1] = std::ceil(unrounded_transformed_size[1]);
        transformed_size[2] = std::ceil(unrounded_transformed_size[2]);

        if(debug) std::cout << "\ndebug info: (transformed_size)\n" <<  transformed_size;

        Eigen::Matrix<double,3,4> transformed_corners_in_pixel_space;
        transformed_corners_in_pixel_space.col(0)[0] = 0;
        transformed_corners_in_pixel_space.col(0)[1] = 0;
        transformed_corners_in_pixel_space.col(0)[2] = 0;

        transformed_corners_in_pixel_space.col(1)[0] = transformed_spacing[0]*transformed_size[0];
        transformed_corners_in_pixel_space.col(1)[1] = 0;
        transformed_corners_in_pixel_space.col(1)[2] = 0;

        transformed_corners_in_pixel_space.col(2)[0] = 0;
        transformed_corners_in_pixel_space.col(2)[1] = transformed_spacing[1]*transformed_size[1];
        transformed_corners_in_pixel_space.col(2)[2] = 0;

        transformed_corners_in_pixel_space.col(3)[0] = 0;
        transformed_corners_in_pixel_space.col(3)[1] = 0;
        transformed_corners_in_pixel_space.col(3)[2] = transformed_spacing[2]*transformed_size[2];

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_pixel_space)\n" <<  transformed_corners_in_pixel_space;

        auto quatered_bounding_box = orientation*(1.0/2.0)*corners_in_rotated_space;
        if(debug) std::cout << "\ndebug info: (quatered_bounding_box)\n" <<  quatered_bounding_box;
        Eigen::Matrix<double,3,1> center_bounding_box = quatered_bounding_box.col(1)+quatered_bounding_box.col(2)+quatered_bounding_box.col(3);
        if(debug) std::cout << "\ndebug info: (center_bounding_box)\n" <<  center_bounding_box;

        Eigen::Matrix<double,3,3> transformed_rotation;
        transformed_rotation = orientation*relative_transform;

        auto transformed_quatered_bounding_box = transformed_rotation*(1.0/2.0)*transformed_corners_in_pixel_space;
        if(debug) std::cout << "\ndebug info: (transformed_quatered_bounding_box)\n" <<  transformed_quatered_bounding_box;
        Eigen::Matrix<double,3,1> transformed_center_bounding_box = transformed_quatered_bounding_box.col(1)+transformed_quatered_bounding_box.col(2)+transformed_quatered_bounding_box.col(3);
        if(debug) std::cout << "\ndebug info: (transformed_center_bounding_box)\n" <<  transformed_center_bounding_box;
        Eigen::Matrix<double,3,4> transformed_corners_in_world_space;
        transformed_corners_in_world_space.col(0) = origin+center_bounding_box-transformed_center_bounding_box;
        transformed_corners_in_world_space.col(1) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(1);
        transformed_corners_in_world_space.col(2) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(2);
        transformed_corners_in_world_space.col(3) = transformed_corners_in_world_space.col(0)+transformed_rotation*transformed_corners_in_pixel_space.col(3);

        if(debug) std::cout << "\ndebug info: (transformed_corners_in_world_space)\n" <<  transformed_corners_in_world_space;

        return BoundingBox{transformed_corners_in_world_space.col(0),transformed_corners_in_world_space.col(1),transformed_corners_in_world_space.col(2),transformed_corners_in_world_space.col(3),transformed_spacing};
    }
};

std::ostream & operator << (std::ostream& o, const BoundingBox& b){
    o << "origin: \n";
    o << b.origin;
    o << "\norientation: \n";
    o << b.orientation;
    o << "\nsize: \n";
    o << b.size;
    o << "\nspacing: \n";
    o << b.spacing;
    return o;
}

constexpr double my_pi = 3.1415;

inline static double degree_to_rad(double degree){
    return degree*my_pi/180;
}

int main(){
    {
        // the bounding box will first be perfect square
        // thus we have

        Eigen::Matrix<double,3,1> origin{{10.0,12.0,30.0}};
        Eigen::Matrix<double,3,1> extrema_along_x{{3.0+origin[0],origin[1],origin[2]}};
        Eigen::Matrix<double,3,1> extrema_along_y{{origin[0],3.0+origin[1],origin[2]}};
        Eigen::Matrix<double,3,1> extrema_along_z{{origin[0],origin[1],3.0+origin[2]}};
        Eigen::Matrix<double,3,1> spacing{{1.0,1.0,1.0}};
        BoundingBox box{origin,extrema_along_x,extrema_along_y,extrema_along_z,spacing};

        Eigen::AngleAxis<double> rotation(degree_to_rad(45.0), Eigen::Vector3d(0.0,0.0,1.0));
        Eigen::Matrix<double,3,3> relative_rotation = rotation.matrix();

        //std::cout << relative_rotation;

        std::cout << "the bounding box is:\n" << box << "\n";
        auto recented_box = box.centered_bounding_box<Strategy::CONSERVATIVE,false>(relative_rotation);
        std::cout << "\n=================\nthe new bounding box is:\n" << recented_box;
    }
    
    return 0;
}