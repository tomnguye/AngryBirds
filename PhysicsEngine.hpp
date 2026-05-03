#pragma once
#include <vector>

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <optional>
#include <Eigen/Eigen>

#include "PhysicsObject.hpp"
#include "CollisionEvent.hpp"
#include "CollisionDetection.hpp"
#include <unordered_set>


class PhysicsEngine {
    public:
    const int step = 16;
    const float dt = (float) step / 1000;
    // const float dt = 0.001f;   // slowed down for testing
    const float g = -9.8f;
    float restitution = 0.5f;
    float friction = 0.8f;
    float dragCoeff = 0.02f;
    float tolerance = 0.0001;
    float epsilon = 0.0001;
    int maxIterations = 100;

    PhysicsEngine() {
        // add config later
    };
    // to make slingshot work
    void addObject(std::unique_ptr<PhysicsObject> obj) {
    objects.push_back(std::move(obj));
    }

    std::vector<CollisionEvent> getCollisions() {
        return colliding_objects(objects);
    }

    float massAbove(PhysicsObject* obj, 
                                const std::vector<CollisionEvent>& contacts, 
                                std::unordered_set<PhysicsObject*>& visited);
    void advanceState();
    void updatePhysics(PhysicsObject& obj);
    int setState(std::vector<std::unique_ptr<PhysicsObject>>);
    const std::vector<std::unique_ptr<PhysicsObject>>& getState();

    private:
    std::vector<std::unique_ptr<PhysicsObject>> objects;
    std::optional<Eigen::Vector2f> newtonsMethod(PhysicsObject&, 
        Eigen::Vector2f (*)(Eigen::Vector2f, const PhysicsObject&, const PhysicsEngine&), 
        Eigen::Matrix2f (*)(Eigen::Vector2f, const PhysicsEngine&));
    
};