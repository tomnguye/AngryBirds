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
    void updatePhysics(PhysicsObject& obj);
    int setState(std::vector<std::unique_ptr<PhysicsObject>>);
    const std::vector<std::unique_ptr<PhysicsObject>>& getState();

    private:
    std::vector<std::unique_ptr<PhysicsObject>> objects;
    std::optional<Eigen::Vector2f> newtonsMethod(PhysicsObject&, 
        Eigen::Vector2f (*)(Eigen::Vector2f, const PhysicsObject&, const PhysicsEngine&), 
        Eigen::Matrix2f (*)(Eigen::Vector2f, const PhysicsEngine&));
    std::vector<std::pair<int, int>> colliding_objects(std::vector<std::unique_ptr<PhysicsObject>>& objs);
};