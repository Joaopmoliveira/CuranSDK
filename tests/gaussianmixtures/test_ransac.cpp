#include <Eigen/Dense>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>
#include <vsg/all.h>
#include <vsgXchange/all.h>

using Normal = Eigen::Vector4f;
using PointCloud = Eigen::Matrix<float, Eigen::Dynamic, 3>;
using HomogeneousPC = Eigen::Matrix<float, Eigen::Dynamic, 4>;

inline double k(int n, float p, float w) {
    return std::log(1 - p) / std::log(1 - std::pow(w, n));
}

inline double randomFloat() { return std::rand() / float(RAND_MAX); }

Normal getNormal(PointCloud const &pc) {
    const size_t N = pc.rows();
    HomogeneousPC hpc(N, 4);
    hpc(Eigen::all, Eigen::seq(0, 2)) << pc;
    hpc(Eigen::all, Eigen::last) << Eigen::MatrixXf::Ones(N, 1);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(hpc, Eigen::ComputeFullU |
                                                   Eigen::ComputeFullV);

    return svd.matrixV()(Eigen::all, Eigen::last);
}

Normal getNormal(HomogeneousPC const &hpc) {
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(hpc, Eigen::ComputeFullU |
                                                   Eigen::ComputeFullV);

    return svd.matrixV()(Eigen::all, Eigen::last);
}

void find(Eigen::VectorXi const &booleanVector,
          std::vector<int> &inliersIndexes) {
    const size_t N = booleanVector.count();
    inliersIndexes.resize(N, 0);
    size_t index = 0, element = 0;
    for (auto const &m : booleanVector) {
        if (m == 1) {
            inliersIndexes[index] = element;
            index++;
            if (index == N)
                return;
        }
        element++;
    }
}

template <size_t size, typename T>
inline void slice(std::vector<T> const &vectorToSlice,
                  std::vector<T> &receivingVector) {
    if (size <= vectorToSlice.size())
        receivingVector =
            std::vector<T>(vectorToSlice.begin(), vectorToSlice.begin() + size);
    else
        std::runtime_error("Size should be less or equal to the vectorToSlice");
}

template <typename T>
inline void shuffle(std::vector<T> &vectorToShuffle, std::mt19937 &mt) {
    std::shuffle(vectorToShuffle.begin(), vectorToShuffle.end(), mt);
}

template <typename T>
inline void linspace(std::vector<T> &vectorToPopulate, T initialValue) {
    std::iota(vectorToPopulate.begin(), vectorToPopulate.end(), initialValue);
}

std::tuple<Normal, PointCloud>
ransacPlane(PointCloud const &pc, float threshold, int safeguard = {}) {
    Normal normal;
    normal.setZero();
    const size_t N = pc.rows();
    HomogeneousPC hpc(N, 4);
    hpc(Eigen::all, Eigen::seq(0, 2)) << pc;
    hpc(Eigen::all, Eigen::last) << Eigen::MatrixXf::Ones(N, 1);

    const size_t maxIter = std::round(2 * k(3, .9, .01));
    int sg = !safeguard ? std::round(.1 * maxIter) : safeguard;

    int iteration = 0, histIter = 0;
    std::vector<int> indexes(N), shuffledIndexes(3), inliers, fittedPoints;
    linspace(indexes, 0);
    Eigen::VectorXf sampleError;
    // Generator of uniformly distributed random integers
    std::random_device rd;
    // Implementation of Mersenne-Twister Engine algorithm, that provides good
    // quality integer randomness
    std::mt19937 mt(rd());
    while (iteration < maxIter && (histIter < sg || sg == -1)) {
        // lógica de permutação entre pc
        shuffle(indexes, mt);
        slice<3>(indexes, shuffledIndexes);

        // calcular normal vindo de sample
        normal = getNormal((HomogeneousPC)hpc(shuffledIndexes, Eigen::all));

        // error = ||pc * normal - 0|| <== pc * normal = 0
        sampleError = (hpc * normal).cwiseAbs();

        // get inliers from error < threshold
        find((sampleError.array() < threshold).cast<int>(), inliers);

        // compare the sizes of previous inliers with the new ones
        if (fittedPoints.size() < inliers.size()) {
            fittedPoints = inliers;
            histIter = 0;
        } else {
            histIter++;
        }
        iteration++;
    }
    normal = getNormal((HomogeneousPC)hpc(fittedPoints, Eigen::all));
    return {normal, pc(fittedPoints, Eigen::all)};
}

void getMiddlePoint(Eigen::Matrix<float, Eigen::Dynamic, 3> const &pc,
                    Eigen::Matrix<float, 1, 3> &middlePoint) {
    const size_t N = pc.rows();
    middlePoint(0, 0) = pc(Eigen::all, 0).sum() / N;
    middlePoint(0, 1) = pc(Eigen::all, 1).sum() / N;
    middlePoint(0, 2) = pc(Eigen::all, 2).sum() / N;
}

void getPlaneTransformFromNormal(
    Normal const &normal, Eigen::Matrix<float, 1, 3> const &origin,
    vsg::ref_ptr<vsg::MatrixTransform> planeTransform) {
    Eigen::Vector3f n;
    n << normal[0], normal[1], normal[2];
    n.normalize();
    Eigen::Vector3f up;
    up << 0, 0, 1;
    float theta = std::acos(n.dot(up));
    Eigen::Vector3f axis = n.cross(up);
    planeTransform->matrix = vsg::rotate(theta, axis[0], axis[1], axis[2]);
    planeTransform->matrix = planeTransform->transform(vsg::translate(
        double(origin[0]), double(origin[1]), double(origin[2])));
}

inline void eigenTest() {
    const int M = 10;
    std::vector<int> indexes(M);
    std::cout << "Indexes[:3]: ";
    for (size_t i = 0; i < 3; i++) {
        std::cout << indexes[i] << "\t";
    }
    std::cout << "\n";
    std::iota(indexes.begin(), indexes.end(), 0);
    std::cout << "Numbered indexes[:3]: ";
    for (size_t i = 0; i < 3; i++) {
        std::cout << indexes[i] << "\t";
    }
    std::cout << "\n";

    std::vector<int> a(indexes.begin(), indexes.begin() + 3);
    std::cout << "a: ";
    for (auto const &b : a) {
        std::cout << b << "\t";
    }
    std::cout << "\n";

    std::random_device rd;
    std::mt19937 mt(rd());
    std::shuffle(indexes.begin(), indexes.end(), mt);
    std::cout << "Shuffled indexes[:3]: ";
    for (size_t i = 0; i < 3; i++) {
        std::cout << indexes[i] << "\t";
    }
    std::cout << "\n";

    std::cout << "a after shuffled: ";
    for (auto const &b : a) {
        std::cout << b << "\t";
    }
    std::cout << "\n";

    const int N = 3;

    std::cout << "c: ";
    std::vector<int> c(indexes.begin(), indexes.begin() + N);
    for (auto const &b : c) {
        std::cout << b << "\t";
    }
    std::cout << "\n";

    Eigen::Matrix<float, Eigen::Dynamic, 4> A(M, 4);
    A(Eigen::all, Eigen::seq(0, 2)) = Eigen::MatrixXf::Random(M, 3);
    A(Eigen::all, Eigen::last) = Eigen::MatrixXf::Ones(M, 1);
    Eigen::MatrixXf V(N, 4);
    V << A(c, Eigen::all);
    // V << 0.8147, 0.9134, 0.2785, 1, 0.9058, 0.6324, 0.5469, 1, 0.1270,
    // 0.0975,
    //     0.9575, 1;
    // V << 1, 0, 0, 1, 0, 1, 0, 1, 3, 1, 0, 1;
    std::cout << "Vector V:\n" << V << "\n";

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(V, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
    std::cout << "SVD's matrix V: \n" << svd.matrixV() << "\n";
    std::cout << "SVD's matrix V last column: \n"
              << svd.matrixV()(Eigen::all, Eigen::last) << "\n";
    std::cout << "Vector V * SVD's matrix V last column: \n"
              << V * svd.matrixV()(Eigen::all, Eigen::last) << "\n";
    Normal normal = svd.matrixV()(Eigen::all, Eigen::last);
    Eigen::VectorXi B = ((A * normal).cwiseAbs().array() < .01f).cast<int>();
    std::vector<int> indexesFound;
    find(B, indexesFound);
    std::cout << "Indexes found in A for this normal: ";
    for (auto const &i : indexesFound)
        std::cout << " " << i;
    std::cout << "\n";

    std::shuffle(indexes.begin(), indexes.end(), mt);
    c = std::vector<int>(indexes.begin(), indexes.begin() + 3);
    V = A(c, Eigen::all);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd2(V, Eigen::ComputeFullU |
                                                  Eigen::ComputeFullV);
    std::cout << "SVD2's matrix V: \n" << svd2.matrixV() << "\n";
    std::cout << "SVD2's matrix V last column: \n"
              << svd2.matrixV()(Eigen::all, Eigen::last) << "\n";
    std::cout << "Vector V * SVD's matrix V last column: \n"
              << V * svd2.matrixV()(Eigen::all, Eigen::last) << "\n";
    normal = svd2.matrixV()(Eigen::all, Eigen::last);
    B = ((A * normal).cwiseAbs().array() < .01f).cast<int>();
    find(B, indexesFound);
    std::cout << "Indexes found in A for this normal: ";
    for (auto const &i : indexesFound)
        std::cout << " " << i;
    std::cout << "\n";
}

inline void testRansacPlane() {
    const int N = 100;
    const float threshold = 0.01f;
    auto [normal, pc] = ransacPlane(PointCloud::Random(N, 3), threshold);

    std::cout << "Normal:\n" << normal << "\n";
    std::cout << "Fitted points (" << pc.rows() << "/" << N << "):\n" << pc << "\n";
}

inline void vsgRansacPlane(int argc, char *argv[]) {
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Particle System";

    vsg::CommandLine arguments(&argc, argv);
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    if (arguments.read({"--fullscreen", "--fs"}))
        windowTraits->fullscreen = true;
    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);

    auto root = vsg::Group::create();
    auto builder = vsg::Builder::create();
    builder->options = options;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    size_t numOfObjects = 1000;
    geomInfo.positions = vsg::vec4Array::create(numOfObjects);
    auto colors = vsg::vec4Array::create(numOfObjects);
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);

    vsg::StateInfo stateInfo;
    stateInfo.blending = true;

    // Generates the point cloud
    const size_t n = arguments.value<size_t>(10, "--rows");
    const size_t N = 2 * n;
    PointCloud pc = PointCloud::Random(N, 3);

    // To enforce a presence of a plane for the first n points, [0,0,1] as
    // normal and random offset
    Eigen::Matrix<float, 3, 3> normalizer;
    normalizer << 1, 0, 0, 0, 1, 0, 0, 0, 0.05f * randomFloat();
    pc(Eigen::seq(0, n - 1), Eigen::all) *= normalizer;
    HomogeneousPC hpc(N, 4);
    hpc(Eigen::all, Eigen::seq(0, 2)) << pc;
    hpc(Eigen::all, Eigen::last) << Eigen::MatrixXf::Ones(N, 1);

    // Ransac parameters
    const int maxIter = std::round(2 * k(3, .9, .01));
    int iteration = 0, histIter = 0;
    std::vector<int> shuffledIndexes(3), inliers, fittedPoints;
    Eigen::VectorXf sampleError;
    sampleError.setZero();
    float threshold = 0.01f;

    // Populate scene with the point cloud
    vsg::ref_ptr<vsg::MatrixTransform> positionPoint, scalePoint;
    auto sphere = builder->createSphere(geomInfo, stateInfo);
    for (size_t row = 0; row < pc.rows(); row++) {
        positionPoint = vsg::MatrixTransform::create();
        scalePoint = vsg::MatrixTransform::create();

        positionPoint->matrix =
            vsg::translate(pc(row, 0), pc(row, 1), pc(row, 2));
        scalePoint->matrix = vsg::scale(0.05);

        scalePoint->addChild(sphere);
        positionPoint->addChild(scalePoint);
        root->addChild(positionPoint);
    }

    // Generates the indexes for the point cloud
    std::vector<int> indexes(N);
    linspace(indexes, 0);
    std::random_device rd;
    std::mt19937 mt(rd());
    shuffle(indexes, mt);
    slice<3>(indexes, shuffledIndexes);

    // First normal, that is dummy
    Normal normal = getNormal((HomogeneousPC)hpc(shuffledIndexes, Eigen::all));

    // Build normal's plane
    Eigen::Matrix<float, 1, 3> middlePoint;
    getMiddlePoint(pc, middlePoint);
    vsg::ref_ptr<vsg::MatrixTransform> fittedPlaneTransform =
        vsg::MatrixTransform::create();
    vsg::ref_ptr<vsg::MatrixTransform> unfittedPlaneTransform =
        vsg::MatrixTransform::create();
    getPlaneTransformFromNormal(normal, middlePoint, fittedPlaneTransform);
    getPlaneTransformFromNormal(normal, middlePoint, unfittedPlaneTransform);
    vsg::ref_ptr<vsg::MatrixTransform> flatCube =
        vsg::MatrixTransform::create();
    flatCube->matrix = vsg::scale(2.0, 2.0, 0.01);
    flatCube->addChild(builder->createBox(geomInfo, stateInfo));
    fittedPlaneTransform->addChild(flatCube);
    unfittedPlaneTransform->addChild(flatCube);
    root->addChild(fittedPlaneTransform);
    root->addChild(unfittedPlaneTransform);

    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();

    vsg::ref_ptr<vsg::Window> window(vsg::Window::create(windowTraits));
    if (!window) {
        std::cout << "Could not create windows." << std::endl;
        return;
    }

    viewer->addWindow(window);

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    root->accept(computeBounds);
    vsg::dvec3 center =
        (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius =
        vsg::length(computeBounds.bounds.max - computeBounds.bounds.min);
    double nearFarRatio = 0.0001;

    // set up the camera
    vsg::dvec3 offset{radius * 1.2, radius * 1.2, radius * .5 * 1.2};
    vsg::dvec3 up{0, 0, 1};
    auto lookAt = vsg::LookAt::create(center + offset, center, up);

    vsg::ref_ptr<vsg::Perspective> perspective = vsg::Perspective::create(
        30.0,
        static_cast<double>(window->extent2D().width) /
            static_cast<double>(window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    auto camera = vsg::Camera::create(
        perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));

    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));

    // compile all the the Vulkan objects and transfer data required to render
    // the scene
    viewer->compile();

    while (viewer->advanceToNextFrame()) {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        if (iteration < maxIter) {
            shuffle(indexes, mt);
            slice<3>(indexes, shuffledIndexes);
            normal = getNormal((HomogeneousPC)hpc(shuffledIndexes, Eigen::all));
            sampleError = (hpc * normal).cwiseAbs();
            find((sampleError.array() < threshold).cast<int>(), inliers);
            if (fittedPoints.size() < inliers.size()) {
                fittedPoints = inliers;
                getMiddlePoint(pc(fittedPoints, Eigen::all), middlePoint);
                getPlaneTransformFromNormal(normal, middlePoint,
                                            fittedPlaneTransform);
            }
            getMiddlePoint(pc(inliers, Eigen::all), middlePoint);
            getPlaneTransformFromNormal(normal, middlePoint,
                                        unfittedPlaneTransform);
            iteration++;
        }

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }
    normal = getNormal((HomogeneousPC)hpc(fittedPoints, Eigen::all));
    std::cout << "final normal computed:\n"
              << normal << "\nwith the following " << fittedPoints.size()
              << " points [x,y,z]:\n"
              << pc(fittedPoints, Eigen::all) << "\ncomputed in " << iteration
              << "/" << maxIter << " iterations\n";
}

int main(int argc, char *argv[]) {
    const char solution = 2;

    switch (solution) {
    default:
    case 0:
        testRansacPlane();
        break;
    case 1:
        eigenTest();
        break;
    case 2:
        vsgRansacPlane(argc, argv);
        break;
    }

    return 0;
}