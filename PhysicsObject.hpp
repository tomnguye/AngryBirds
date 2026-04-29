#include <Eigen/Eigen>

class PhysicsObject{
    public:
        Eigen::Vector3f position;
        Eigen::Vector3f velocity; 
        Eigen::Vector3f rotation;
        float boundingSphere;
        bool stationary;
        PhysicsObject(
            const Eigen::Vector3f p = {0,0,0}, 
            const Eigen::Vector3f v = {0,0,0}, 
            const Eigen::Vector3f r = {0,0,0},
            float bs = 1,
            bool s = false) 
        {
            position = p;
            velocity = v;
            rotation = r;
            boundingSphere = bs;
            stationary = s;
        }
};

class Sphere : public PhysicsObject {
    public:
        float radius;
        Sphere(float r, Eigen::Vector3f p = {0,0,0}, Eigen::Vector3f v = {0,0,0}) : PhysicsObject(p, v, r) {
            radius = r;
        };
};

class Rectangle : public PhysicsObject {
    public:

}