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
        float boundingRadius;
        bool stationary;
        PhysicsObject(
            obj_type t,
            const Eigen::Vector2f p = Eigen::Vector2f::Zero(), 
            const Eigen::Vector2f v = Eigen::Vector2f::Zero(), 
            float r = 0,
            bool s = false) 
        {
            type = t;
            position = p;
            velocity = v;
            rotation = r;
            stationary = s;
        }
};

class Circle : public PhysicsObject {
    public:
    float radius;
    Circle(float r, Eigen::Vector2f p = Eigen::Vector2f::Zero(), Eigen::Vector2f v = Eigen::Vector2f::Zero()) : PhysicsObject(CIRCLE, p, v) {
        radius = r;
        boundingRadius = r;
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
    };
};