#pragma once
#include <Eigen/Eigen>
#include <optional>

enum obj_type {
    RECTANGLE,
    CIRCLE,
};
class PhysicsObject {
public:
    obj_type type;
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
    bool stationary;
    float rotation;
    float angularVelocity;
    float boundingRadius;
    float mass, inv_mass;
    float inertia, inv_inertia;

    PhysicsObject(obj_type t, float i,
        Eigen::Vector2f p = Eigen::Vector2f::Zero(),
        Eigen::Vector2f v = Eigen::Vector2f::Zero(),
        float r = 0, float w = 0, bool s = false, float m = 1)
    {
        type = t;
        position = p; velocity = v;
        rotation = r; angularVelocity = w;
        stationary = s;
        mass = m; 
        inv_mass = s ? 0.0f : 1.0f / m;  // stationary objects have infinite mass
        inertia = i;
        inv_inertia = s ? 0.0f : 1.0f / i;
        
    }
};

class Circle : public PhysicsObject {
public:
    float radius;
    Circle(float r,
        Eigen::Vector2f p = Eigen::Vector2f::Zero(),
        Eigen::Vector2f v = Eigen::Vector2f::Zero(),
        float rot = 0, float w = 0, bool s = false, float m = 1)
        : PhysicsObject(CIRCLE, 0.5f * m * r * r, p, v, rot, w, s, m)
    {
        radius = r;
        boundingRadius = r;
    }
};

class Rectangle : public PhysicsObject {
public:
    Eigen::Vector2f dimentions;
    float baseAngle;
    Rectangle(Eigen::Vector2f d,
        Eigen::Vector2f p = Eigen::Vector2f::Zero(),
        Eigen::Vector2f v = Eigen::Vector2f::Zero(),
        float r = 0, float w = 0, bool s = false, float m = 1)
        : PhysicsObject(RECTANGLE, (1.0f/12.0f) * m * (d[0]*d[0] + d[1]*d[1]), p, v, r, w, s, m)
    {
        dimentions = d;
        baseAngle = atan2f(d[1], d[0]);
        boundingRadius = d.norm() / 2.0f;
    }
};