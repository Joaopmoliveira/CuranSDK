#include "rendering/PhaseWiredBox.h"

namespace curan{
namespace renderable{

PhaseWiredBox::PhaseWiredBox(){
    auto node = vsg::Group::create();

    transform = vsg::MatrixTransform::create();
    obj_contained = vsg::Group::create();

    vsg::vec3 v000(vsg::vec3(0.0,0.0,0.0));
    vsg::vec3 v100(vsg::vec3(epsilon,0.0,0.0));
    vsg::vec3 v010(vsg::vec3(0.0,epsilon,0.0));
    vsg::vec3 v001(vsg::vec3(0.0,0.0,epsilon));

    vsg::vec3 v110 = v100 + v010;
    vsg::vec3 v101 = v100 + v001;
    vsg::vec3 v111 = v100 + v010 + v001;
    vsg::vec3 v011 = v010 + v001;

    vsg::vec3 n0 = normalize(v000 - v111);
    vsg::vec3 n1 = normalize(v100 - v011);
    vsg::vec3 n2 = normalize(v110 - v001);
    vsg::vec3 n3 = normalize(v010 - v101);
    vsg::vec3 n4 = -n2;
    vsg::vec3 n5 = -n3;
    vsg::vec3 n6 = -n0;
    vsg::vec3 n7 = -n1;

    vsg::vec3 texture = {0.0f, 1.0f, 1.0f};
    auto [t_origin, t_scale, t_top] = texture.value;

    vsg::vec2 t00(0.0f, t_origin);
    vsg::vec2 t01(0.0f, t_top);
    vsg::vec2 t10(1.0f, t_origin);
    vsg::vec2 t11(1.0f, t_top);

    // set up vertex and index arrays
    vertices = vsg::vec3Array::create(
        {v000, v100, v110, v010,
        v001, v101, v111, v011});

    normals = vsg::vec3Array::create(
        {n0, n1, n2, n3,
        n4, n5, n6, n7});

    texcoords = vsg::vec2Array::create(
        {t00, t10, t11, t01,
        t00, t10, t11, t01});

    indices = vsg::ushortArray::create(
        {0, 1, 1, 2, 2, 3, 3, 0,
        0, 4, 1, 5, 2, 6, 3, 7,
        4, 5, 5, 6, 6, 7, 7, 4});

    vertices->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
    normals->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;

    // setup geometry
    auto vid = vsg::VertexIndexDraw::create();

    color = vsg::vec4Array::create(1, vsg::vec4(1.0,0.0,0.0,1.0));

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(normals);
    arrays.push_back(texcoords);
    arrays.push_back(color);
    vid->assignArrays(arrays);

    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    vid->instanceCount = 1;

    node->addChild(vid);
    obj_contained->addChild(node);
}

vsg::ref_ptr<Renderable> PhaseWiredBox::make() {
    vsg::ref_ptr<PhaseWiredBox> sphere_to_add = PhaseWiredBox::create();
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

void PhaseWiredBox::update_frame(vsg::vec3 origin){
    update_frame_config(origin);
}

void PhaseWiredBox::update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset){
    vsg::vec3 xdir = xdiroffset-origin;
    vsg::vec3 ydir_first_comp = vsg::vec3(xdir.y,-xdir.y,0.0);
    vsg::vec3 ydir_second_comp = vsg::vec3(xdir.z,0.0,xdir.x);
    vsg::vec3 ydir = ydir_first_comp+ydir_second_comp;
    ydir = vsg::normalize(ydir)*epsilon;
    vsg::vec3 zdir = vsg::normalize(vsg::cross(xdir,ydir))*epsilon;
    update_frame_config(origin,xdir,ydir,zdir);
}

void PhaseWiredBox::update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset){
    vsg::vec3 xdir = xdiroffset-origin;
    vsg::vec3 ydir_diff = ydiroffset-xdiroffset;
    auto yprojected = vsg::normalize(vsg::cross(ydir_diff,xdir));
    vsg::vec3 ydir = vsg::normalize(vsg::cross(xdir,yprojected));
    ydir = ydir*vsg::dot(ydir_diff,ydir);
    vsg::vec3 zdir = vsg::normalize(vsg::cross(xdir,ydir))*epsilon;
    update_frame_config(origin,xdir,ydir,zdir);
}

void PhaseWiredBox::update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset, vsg::vec3 zdiroffset){
    vsg::vec3 xdir = xdiroffset-origin;
    vsg::vec3 ydir_diff = ydiroffset-xdiroffset;
    vsg::vec3 zdir_diff = zdiroffset-ydiroffset;
    auto yprojected = vsg::normalize(vsg::cross(ydir_diff,xdir));
    vsg::vec3 ydir = vsg::normalize(vsg::cross(xdir,yprojected));
    ydir = ydir*vsg::dot(ydir_diff,ydir);
    vsg::vec3 zdir = vsg::normalize(vsg::cross(xdir,ydir));
    zdir = zdir*vsg::dot(zdir_diff,zdir);
    update_frame_config(origin,xdir,ydir,zdir);
}

void PhaseWiredBox::update_frame_config(vsg::vec3 origin,vsg::vec3 xdir,vsg::vec3 ydir ,vsg::vec3 zdir ){
    vsg::vec3 v000 = origin;
    vsg::vec3 v100 = origin + xdir;
    vsg::vec3 v010 = origin + ydir;
    vsg::vec3 v001 = origin + zdir;

    vsg::vec3 v110 = origin + xdir + ydir;
    vsg::vec3 v101 = origin + xdir + zdir;
    vsg::vec3 v111 = origin + xdir + ydir + zdir;
    vsg::vec3 v011 = origin + zdir + ydir;

    vsg::vec3 n0 = normalize(v000 - v111);
    vsg::vec3 n1 = normalize(v100 - v011);
    vsg::vec3 n2 = normalize(v110 - v001);
    vsg::vec3 n3 = normalize(v010 - v101);
    vsg::vec3 n4 = -n2;
    vsg::vec3 n5 = -n3;
    vsg::vec3 n6 = -n0;
    vsg::vec3 n7 = -n1;

    // set up vertex and index arrays
    auto local_vertices = vsg::vec3Array::create(
        {v000, v100, v110, v010,
        v001, v101, v111, v011});

    auto local_normals = vsg::vec3Array::create(
        {n0, n1, n2, n3,
        n4, n5, n6, n7});

    for(auto iterator_dst = vertices->begin(), iterator_src = local_vertices->begin(); iterator_dst != vertices->end()&& iterator_src !=local_vertices->end() ; ++iterator_dst,++iterator_src)
        (*iterator_dst) = (*iterator_src);
    vertices->dirty();
};

}
}
