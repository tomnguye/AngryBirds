#pragma once
#include "PhysicsObject.hpp"
#include <Eigen/Eigen>
#include <optional>

class CollisionEvent {
public:
    PhysicsObject *objA;
    PhysicsObject *objB;
    std::vector<Eigen::Vector2f> contactPoints;
    std::vector<float> penetrationDepths;
    Eigen::Vector2f contactNormal;
    CollisionEvent(PhysicsObject *a, PhysicsObject *b,
                   std::vector<float> penetrations,
                   std::vector<Eigen::Vector2f> points,
                   Eigen::Vector2f normal) {
        objA = a;
        objB = b;
        contactNormal = normal;
        contactPoints = points;
        penetrationDepths = penetrations;
    }
};
