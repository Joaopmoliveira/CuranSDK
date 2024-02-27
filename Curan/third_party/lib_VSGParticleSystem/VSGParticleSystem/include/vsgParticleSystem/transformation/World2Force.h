#pragma once

#include "Transformation.h"

namespace vsgps {

// todo: Make these transformations not directly on the state variable, but on
// the coordinates, making thereafter a state vector through json and the
// current coordinates
//! Only focused on 3D motion/state = [x y z]^T
class World2Disp : public Transformation {
  public:
    inline static std::string label = "world2disp";
    
    World2Disp(size_t transformationIdx = 1);

    void setNormal(Normal const &n);
    Normal const &getNormal() const;
    void setOrigin(Origin const &o);
    Origin const &getOrigin() const;
    void setVelIdxs(std::array<int, 3> const &v);
    std::array<int, 3> const &getVelIdxs() const;
    void setIsPosition(bool isPos);
    bool const &getIsPosition() const;
    void computeT();
    friend std::ostream &operator<<(std::ostream &out, World2Disp const &w2d);

    Eigen::Matrix<double, 7, 7> const &getT() const;

    virtual void operator()(State &xi);
    virtual void operator()(State const &xi, State &xitilde);
    virtual void inv(State &xitilde);
    virtual void inv(State const &xitilde, State &xi);
    virtual void invVel(State &dxitilde);
    virtual void invVel(State const &dxitilde, State &dxi);
    static World2Disp fromJson(std::string path);

  protected:
    Normal normal;
    Origin origin;
    Eigen::Matrix<double, 7, 7> T, invT;
    Eigen::Matrix<double, 7, 1> h;
    Eigen::Matrix<double, 3, 3> R, K;
    std::array<int, 3> velIdxs;
    std::vector<int> indices;
    bool isPosition = true;

    void computeR();
    void computeK(Eigen::Vector3d const &k);
};

class Disp2Force : public Transformation {
  public:
    inline static std::string label = "disp2world";

    Disp2Force(size_t idxTransformation = 1);

    void setLength(double newL);
    double const &getLength() const;
    void setArea(double newA);
    double const &getArea() const;
    void setYoungModulus(double newE);
    double const &getYoungModulus() const;
    void setStiffness(double newK);
    double const &getStiffness() const;
    void setLimit(double newLimit);
    double const &getLimit() const;

    friend std::ostream &operator<<(std::ostream &out, Disp2Force const &d2f);

    virtual void operator()(State &xi);
    virtual void operator()(State const &xi, State &xitilde);
    virtual void inv(State &xitilde);
    virtual void inv(State const &xitilde, State &xi);
    static Disp2Force fromJson(std::string path);

  protected:
    double L, E, A, K;
    double limit = 1e-6;

    void computeK();
};
} // namespace vsgps