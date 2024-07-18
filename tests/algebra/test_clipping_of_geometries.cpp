#include <iostream>
#include "geometry/Polyheadra.h"
#include "geometry/Intersection.h"

int main(){
    curan::geometry::Cube cube{1,1,1};
    auto value = curan::geometry::clip_with_plane(cube,{0.0,0.0,1.0},{0.0,0.0,0.0});
    if(value){
        std::cout << "polygon is: \n" << *value << std::endl;
    } else {
        std::cout << "no polygon found\n";
    }
    return 0;
}