#include "PhysicsEngine.hpp"
#include <stdio.h>

std::optional<Eigen::Vector3f> PhysicsEngine::newtonsMethod(PhysicsObject& obj, Eigen::Vector3f (*f)(Eigen::Vector3f, PhysicsObject, PhysicsEngine), Eigen::Matrix3f (*fPrime)(Eigen::Vector3f, PhysicsEngine)) {
    Eigen::Vector3f y, x1;
    Eigen::Matrix3f yPrime;
    Eigen::Vector3f x0 = obj.velocity;
    for (int _ = 0; _ < maxIterations; _++) {
        y = f(x0, obj, *this);
        yPrime = fPrime(x0, *this);
        if (yPrime.norm() < epsilon) {break;}
        x1 = x0 - yPrime.inverse() * y;
        if ((x1 - x0).norm() <= tolerance) { return x1;}
        x0 = x1;
    }
    return std::nullopt;
}

int PhysicsEngine::setState(std::vector<PhysicsObject> objs) {
    objects = objs;
    return 0;
}

std::vector<PhysicsObject> PhysicsEngine::getState() {
    return objects;
}

void PhysicsEngine::advanceState() {
    for (auto& obj: objects) {
        updatePhysics(obj);
    }
    auto c = Colliding_objects(objects);
    for (auto& [a,b]: c) {
        std::cerr << "collision" << a << b << c.size() << std::endl;
    }
}

void PhysicsEngine::updatePhysics(PhysicsObject& obj)
{
    // obj.velocity[1] += g * dt;

    auto f = [](Eigen::Vector3f vNew, PhysicsObject obj, PhysicsEngine eng) -> Eigen::Vector3f {
        return vNew - obj.velocity + eng.dragCoeff * eng.dt * vNew.norm() * vNew; 
    };
    auto fPrime = [](Eigen::Vector3f vNew, PhysicsEngine eng) -> Eigen::Matrix3f {
        return Eigen::Matrix3f::Identity() + eng.dt * eng.dragCoeff * (vNew.norm() * Eigen::Matrix3f::Identity() + (vNew * vNew.transpose())/vNew.norm());
    };

    auto result = newtonsMethod(obj, f, fPrime);

    if (result.has_value()) {
        obj.velocity = result.value();
    }
    else {
        std::cout << "implicite fail";
        float speed = obj.velocity.norm();

        if (speed > 1e-6f) {
            obj.velocity -= dragCoeff * speed * obj.velocity;
        }
    }

    obj.position += obj.velocity * dt;
}

std::vector<std::pair<int, int>> PhysicsEngine::Colliding_objects(std::vector<PhysicsObject>& objs) {
    std::vector<std::pair<int, int>> collisions = {}; 
    for (uint i = 0; i < objs.size(); i ++) {
        for (uint j = i + 1; j < objs.size(); j++) {
            if ((objs[i].position - objs[j].position).norm() <= objs[i].boundingSphere + objs[j].boundingSphere) {
                if (true) {// NOTE replace with actual check
                    collisions.push_back(std::pair(i,j));
                }
            }
        }
    }
    return collisions;
}