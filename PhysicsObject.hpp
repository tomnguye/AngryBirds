#pragma once
#include <Eigen/Eigen>
#include <optional>

enum obj_type {
    RECTANGLE,
    CIRCLE,
};

class PhysicsObject{
    public:
        obj_type type;
        Eigen::Vector2f position;
        Eigen::Vector2f velocity;
        float rotation; // In radians
        float angularVelocity;
        float boundingRadius;
        bool stationary;
        float mass;
        float inv_mass;
        float inertia;
        float inv_inertia;
        PhysicsObject(
            obj_type t,
            const Eigen::Vector2f p = Eigen::Vector2f::Zero(), 
            const Eigen::Vector2f v = Eigen::Vector2f::Zero(), 
            float r = 0,
            float w = 0,
            bool s = false,
            float m = 1,
            float I = 1
        ) 
        {
            type = t;
            position = p;
            velocity = v;
            rotation = r;
            angularVelocity = w;
            stationary = s;
            mass = m;
            inv_mass = 1/m;
            inertia = I;
            inv_inertia = 1/I;
        }
};

class Circle : public PhysicsObject {
    public:
    float radius;
    Circle(float r, Eigen::Vector2f p = Eigen::Vector2f::Zero(), Eigen::Vector2f v = Eigen::Vector2f::Zero()) : PhysicsObject(CIRCLE, p, v) {
        radius = r;
        boundingRadius = r;
        inertia = 0.5 * this->mass * r * r; // Inertia formula for circle: 1/2 * m * r^2
        inv_inertia = 1/inertia;
    };
};

class Rectangle : public PhysicsObject {
    public:
    Eigen::Vector2f dimentions;
    float baseAngle;
    Rectangle(Eigen::Vector2f d, Eigen::Vector2f p = Eigen::Vector2f::Zero(), Eigen::Vector2f v = Eigen::Vector2f::Zero(), float r = 0) : PhysicsObject(RECTANGLE, p, v, r) {
        dimentions = d;
        baseAngle = atan2f(d[1], d[0]);
        boundingRadius = dimentions.norm() / 2.0f;
        inertia = (1/12) * this->mass * (d[0]*d[0], d[1]*d[1]); // Inertia formula for rectangle: 1/12 * m * w^2 * h^2
        inv_inertia = 1/inertia;
    };
};