#include <vector>

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <optional>
#include <Eigen/Eigen>

#include "PhysicsObject.hpp"

class PhysicsEngine {
    public:
    const int step = 16;
    const float dt = (float) step / 1000;
    const float g = -9.8f;
    float restitution = 0.7f;
    float friction = 0.8f;
    float dragCoeff = 0.02f;
    float tolerance = 0.0001;
    float epsilon = 0.0001;
    int maxIterations = 16;

    PhysicsEngine() {
        // add config later
    };

    void advanceState();
    int setState(std::vector<PhysicsObject>);
    std::vector<PhysicsObject> getState();

    private:
    std::vector<PhysicsObject> objects;
    std::optional<Eigen::Vector3f> newtonsMethod(PhysicsObject&, 
        Eigen::Vector3f (*)(Eigen::Vector3f, PhysicsObject, PhysicsEngine), 
        Eigen::Matrix3f (*)(Eigen::Vector3f, PhysicsEngine));
    void updatePhysics(PhysicsObject&);
    std::vector<std::pair<int, int>> Colliding_objects(std::vector<PhysicsObject>& objs);
};