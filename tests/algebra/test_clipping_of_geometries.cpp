#include <iostream>
#include "geometry/Polyheadra.h"
#include "geometry/Intersection.h"


void test_cube_intersection(){
    std::cout << "test_cube_intersection:\n";
    curan::geometry::Cube geom{1,1,1};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_open_cylinder_intersection(){
    std::cout << "test_open_cylinder_intersection:\n";
    curan::geometry::OpenCylinder geom{10,10,1.0,1.0};
    std::cout << "geom:\n" << geom << std::endl;
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}


void test_closed_cylinder_intersection(){
    std::cout << "test_closed_cylinder_intersection:\n";
    curan::geometry::ClosedCylinder geom{10,10,1.0,1.0};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_shpere_intersection(){
    std::cout << "test_shpere_intersection:\n";
    curan::geometry::Sphere geom{10,10,1.0};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_torus_intersection(){
    std::cout << "test_torus_intersection:\n";
    curan::geometry::Torus geom{10,10,2.0,1.0};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_tethrahedron_intersection(){
    std::cout << "test_tethrahedron_intersection:\n";
    curan::geometry::Tetrahedron geom{};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_hexahedron_intersection(){
    std::cout << "test_hexahedron_intersection:\n";
    curan::geometry::Hexahedron geom{};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_octaheadron_intersection(){
    std::cout << "test_octaheadron_intersection:\n";
    curan::geometry::Octahedron geom{};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_docaheadron_intersection(){
    std::cout << "test_docaheadron_intersection:\n";
    curan::geometry::Dodecahedron geom{};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

void test_icosaheadron_intersection(){
    std::cout << "test_icosaheadron_intersection:\n";
    curan::geometry::Icosahedron geom{};
    auto value = curan::geometry::clip_with_plane(geom,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
}

int main(){
    test_cube_intersection();
    test_open_cylinder_intersection();
    //test_closed_cylinder_intersection();
    //test_shpere_intersection();
    //test_torus_intersection();
    test_tethrahedron_intersection();
    test_hexahedron_intersection();
    test_octaheadron_intersection();
    test_docaheadron_intersection();
    test_icosaheadron_intersection();
    return 0;
}