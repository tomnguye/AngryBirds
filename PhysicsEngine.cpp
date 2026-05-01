#include "PhysicsEngine.hpp"
#include "CollisionEvent.hpp"
#include "CollisionDetection.hpp"
#include <stdio.h>

std::optional<Eigen::Vector2f> PhysicsEngine::newtonsMethod(PhysicsObject& obj, Eigen::Vector2f (*f)(Eigen::Vector2f, const PhysicsObject&, const PhysicsEngine&), Eigen::Matrix2f (*fPrime)(Eigen::Vector2f, const PhysicsEngine&)) {
    Eigen::Vector2f y, x1;
    Eigen::Matrix2f yPrime;
    Eigen::Vector2f x0 = obj.velocity;
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

int PhysicsEngine::setState(std::vector<std::unique_ptr<PhysicsObject>> objs) {
    objects = std::move(objs);
    return 0;
}

const std::vector<std::unique_ptr<PhysicsObject>>& PhysicsEngine::getState() {
    return objects;
}

void PhysicsEngine::advanceState() {
    for (auto& obj: objects) {
        updatePhysics(*obj);
    }
    auto c = colliding_objects(objects);
    for (auto& event: c) {
        PhysicsObject* a = event.objA;
        PhysicsObject* b = event.objB;
        float penetration = event.penetrationDepth;
        Eigen::Vector2f point = event.contactPoint;
        Eigen::Vector2f normal = event.contactNormal;
        
        // 4. Collision normal and penetration
        Eigen::Vector2f correction = normal * penetration / (a->inv_mass + b->inv_mass);

        // 5. Collision response impulse
        Eigen::Vector2f relativeVelocity = b->velocity - a->velocity;
        float velAlongNormal = relativeVelocity.dot(normal);

        if (velAlongNormal <= 0) { // Check if theyre already moving apart, this is to stop them from stacking velocity by being collided.
            float j = -(1 + restitution) * velAlongNormal;
            j /= a->inv_mass + b->inv_mass;

            Eigen::Vector2f impulse = j * normal;
            a->velocity -= impulse * a->inv_mass;
            b->velocity += impulse * b->inv_mass;

            // 6. Rotation from off-center hits
            Eigen::Vector2f ra = point - a->position;
            Eigen::Vector2f rb = point - b->position;
            
            a->angularVelocity += (ra.x() * impulse.y() - ra.y() * impulse.x()) * a->inv_inertia;
            b->angularVelocity += (rb.x() * impulse.y() - rb.y() * impulse.x()) * b->inv_mass;
        }

        
    }
}

void PhysicsEngine::updatePhysics(PhysicsObject& obj)
{
    // obj.velocity[1] += g * dt;

    auto f = [](Eigen::Vector2f vNew, const PhysicsObject& obj, const PhysicsEngine& eng) -> Eigen::Vector2f {
        return vNew - obj.velocity + eng.dragCoeff * eng.dt * vNew.norm() * vNew; 
    };
    auto fPrime = [](Eigen::Vector2f vNew, const PhysicsEngine& eng) -> Eigen::Matrix2f {
        return Eigen::Matrix2f::Identity() + eng.dt * eng.dragCoeff * (vNew.norm() * Eigen::Matrix2f::Identity() + (vNew * vNew.transpose())/vNew.norm());
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
    obj.rotation += obj.angularVelocity * dt;

    // 7. Brick motion after impact, TODO: I DONT LIKE THIS.
    obj.angularVelocity *= 0.995;
}
