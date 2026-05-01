#pragma once
#include <Eigen/Eigen>
#include <optional>
#include "PhysicsObject.hpp"

class CollisionEvent{
    public:
        PhysicsObject* objA;
        PhysicsObject* objB;
        Eigen::Vector2f contactPoint;
        Eigen::Vector2f contactNormal;
        CollisionEvent(PhysicsObject* a, PhysicsObject* b, Eigen::Vector2f point, Eigen::Vector2f normal) {
            objA = a;
            objB = b;
            contactPoint = point;
            contactNormal = normal;
        }
};
