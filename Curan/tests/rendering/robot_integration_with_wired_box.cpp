#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "utils\Flag.h"
#include "utils\TheadPool.h"

struct PhaseCreatedBox : public vsg::Inherit<curan::renderable::Renderable, PhaseCreatedBox>{
    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;
    vsg::ref_ptr<vsg::vec4Array> color;
    static constexpr float epsilon = 0.0001f;

    PhaseCreatedBox(){
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

    static vsg::ref_ptr<Renderable> make() {
        vsg::ref_ptr<PhaseCreatedBox> sphere_to_add = PhaseCreatedBox::create();
        vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
        return val;
    }

    void update_frame(vsg::vec3 origin){
        update_frame_config(origin);
    }

    void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset){
        vsg::vec3 xdir = xdiroffset-origin;
        vsg::vec3 ydir_first_comp = vsg::vec3(xdir.y,-xdir.y,0.0);
        vsg::vec3 ydir_second_comp = vsg::vec3(xdir.z,0.0,xdir.x);
        vsg::vec3 ydir = ydir_first_comp+ydir_second_comp;
        ydir = vsg::normalize(ydir)*epsilon;
        vsg::vec3 zdir = vsg::normalize(vsg::cross(xdir,ydir))*epsilon;
        update_frame_config(origin,xdir,ydir,zdir);
    }

    void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset){
        vsg::vec3 xdir = xdiroffset-origin;
        vsg::vec3 ydir_diff = ydiroffset-xdiroffset;
        auto yprojected = vsg::normalize(vsg::cross(ydir_diff,xdir));
        vsg::vec3 ydir = vsg::normalize(vsg::cross(xdir,yprojected));
        ydir = ydir*vsg::dot(ydir_diff,ydir);
        vsg::vec3 zdir = vsg::normalize(vsg::cross(xdir,ydir))*epsilon;
        update_frame_config(origin,xdir,ydir,zdir);
    }

    void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset, vsg::vec3 zdiroffset){
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

    void update_frame_config(vsg::vec3 origin,vsg::vec3 xdir = {0.001f,0.0f,0.0f},vsg::vec3 ydir = {0.0f,0.001f,0.0f},vsg::vec3 zdir = {0.0f,0.0f,0.001f}){
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
};

int main(){
    //initualize the thread pool;
	curan::utilities::initialize_thread_pool(4);

    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    vsg::ref_ptr<curan::renderable::Renderable> robotRenderable = curan::renderable::SequencialLinks::make(create_info);
    window << robotRenderable;

    curan::renderable::Sphere::Info infosphere;
    infosphere.builder = vsg::Builder::create();
    infosphere.geomInfo.color = vsg::vec4(1.0,0.0,0.0,1.0);
    infosphere.geomInfo.dx = vsg::vec3(0.02f,0.0,0.0);
    infosphere.geomInfo.dy = vsg::vec3(0.0,0.02f,0.0);
    infosphere.geomInfo.dz = vsg::vec3(0.0,0.0,0.02f);
    auto sphere = curan::renderable::Sphere::make(infosphere);
    auto mat = vsg::translate(0.0,0.0,0.0);
    sphere->update_transform(mat);
    window << sphere;

    std::array<float,3> temp_pos = {0.0,0.0,0.0};
    std::atomic<std::array<float,3>> current_position;
    current_position.store(temp_pos);
    
    curan::utilities::Job append_box;
	append_box.description = "function that adds the wired box on screen";
	append_box.function_to_execute = [&window,&current_position]() {
        auto box = PhaseCreatedBox::make();
        window << box;
        auto casted_box = box->cast<PhaseCreatedBox>();

        auto flag1 = curan::utilities::Flag::make_shared_flag();
        flag1->clear();
        curan::utilities::Job key_reader;
        key_reader.description = "read the keys";
        key_reader.function_to_execute = [&flag1](){
            char c;
            std::cout << "reading key\n";
            std::cin >> c;
            std::cout << "key read\n";
            flag1->set();
        };
        curan::utilities::pool->submit(key_reader); //we must click on a key before advancing with or test
        flag1->clear();
        curan::utilities::pool->submit(key_reader);
        std::cout << "fixing first vertex\n";
        std::array<float,3> temp_pos = current_position.load();
        std::printf("x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        casted_box->update_frame(vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]));
        vsg::vec3 origin_fixed = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);

        std::cout << "fixing second vertex\n";
        while(!flag1->value()){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            casted_box->update_frame(origin_fixed,vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]));
        }
        flag1->clear();
        auto xdir = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        curan::utilities::pool->submit(key_reader);
        std::cout << "fixing third vertex\n";
        while(!flag1->value()){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            casted_box->update_frame(origin_fixed,xdir,vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]));
        }
        flag1->clear();
        auto ydir = vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
        std::printf("x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
        curan::utilities::pool->submit(key_reader);
        std::cout << "fixing fourth vertex\n";
        while(!flag1->value()){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            temp_pos = current_position.load();
            casted_box->update_frame(origin_fixed,xdir,ydir,vsg::vec3(temp_pos[0],temp_pos[1],temp_pos[2]));
        }
        std::printf("x : %f y : %f z : %f\n",temp_pos[0],temp_pos[1],temp_pos[2]);
	};
    curan::utilities::pool->submit(append_box);

    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here
	
	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	// Attach tool
	double toolMass = 0.0;                                                                     // No tool for now
	RigidBodyDynamics::Math::Vector3d toolCOM = RigidBodyDynamics::Math::Vector3d::Zero(3, 1);
	RigidBodyDynamics::Math::Matrix3d toolInertia = RigidBodyDynamics::Math::Matrix3d::Zero(3, 3);
	auto myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);

	robot->attachToolToRobotModel(myTool.get());

    RigidBodyDynamics::Math::VectorNd measured_torque = RigidBodyDynamics::Math::VectorNd::Zero(7,1);
    Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
    Vector3d p_0_cur = Vector3d(0, 0, 0.045);
    RigidBodyDynamics::Math::MatrixNd Jacobian = RigidBodyDynamics::Math::MatrixNd::Zero(6, NUMBER_OF_JOINTS);

    auto robotRenderableCasted = robotRenderable->cast<curan::renderable::SequencialLinks>();

    double q_current [NUMBER_OF_JOINTS] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double sampletime = 0.001;
    double time = 0.0;

    while(window.run_once()) {
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
            q_current[i] = std::sin(10*time)*1.57;
		    iiwa->q[i] = q_current[i];
            robotRenderableCasted->set(i,q_current[i]);
	    }
        static RigidBodyDynamics::Math::VectorNd q_old = iiwa->q;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
		    iiwa->qDot[i] = (iiwa->q[i] - q_old[i]) / sampletime;
	    }   

        robot->getMassMatrix(iiwa->M,iiwa->q);
	    iiwa->M(6,6) = 45 * iiwa->M(6,6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c,iiwa->g,iiwa->q,iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur,iiwa->q,pointPosition,NUMBER_OF_JOINTS);              // 3x1 position of flange (body = 7), expressed in base coordinates
        
        mat = vsg::translate(p_0_cur(0,0),p_0_cur(1,0),p_0_cur(2,0));
        sphere->update_transform(mat);

        temp_pos = {(float)p_0_cur(0,0),(float)p_0_cur(1,0),(float)p_0_cur(2,0)};
        current_position.store(temp_pos);

        robot->getJacobian(Jacobian,iiwa->q,pointPosition,NUMBER_OF_JOINTS);

        q_old = iiwa->q;
        time += sampletime;
    }

    //terminate the thread pool;
	curan::utilities::terminate_thread_pool();
    return 0;
}