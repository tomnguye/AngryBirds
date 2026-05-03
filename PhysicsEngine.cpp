#include "PhysicsEngine.hpp"
#include "CollisionEvent.hpp"
#include "CollisionDetection.hpp"
#include <stdio.h>

// std::optional<Eigen::Vector2f> PhysicsEngine::newtonsMethod(PhysicsObject& obj, Eigen::Vector2f (*f)(Eigen::Vector2f, const PhysicsObject&, const PhysicsEngine&), Eigen::Matrix2f (*fPrime)(Eigen::Vector2f, const PhysicsEngine&)) {
//     Eigen::Vector2f y, x1;
//     Eigen::Matrix2f yPrime;
//     Eigen::Vector2f x0 = obj.velocity;
//     for (int _ = 0; _ < maxIterations; _++) {
//         y = f(x0, obj, *this);
//         yPrime = fPrime(x0, *this);
//         if (yPrime.norm() < epsilon) {break;}
//         x1 = x0 - yPrime.inverse() * y;
//         if ((x1 - x0).norm() <= tolerance) { return x1;}
//         x0 = x1;
//     }
//     return std::nullopt;
// }

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
    for (auto& event : c) {
            PhysicsObject* a = event.objA;
            PhysicsObject* b = event.objB;
            a->sleeping = false;
            b->sleeping = false;
    }

    struct ContactImpulse { float normal = 0.f; float friction = 0.f; };
    int totalContacts = 0;
    for (auto& e : c) totalContacts += e.contactPoints.size();
    std::vector<ContactImpulse> impulses(totalContacts);

    for (int j = 0; j < maxIterations; j++) {
        int ci = 0;
        for (auto& event : c) {
            PhysicsObject* a = event.objA;
            PhysicsObject* b = event.objB;
            Eigen::Vector2f normal = event.contactNormal;

            for (int i = 0; i < (int)event.contactPoints.size(); i++, ci++) {
                Eigen::Vector2f point = event.contactPoints[i];
                Eigen::Vector2f ra = point - a->position;
                Eigen::Vector2f rb = point - b->position;

                Eigen::Vector2f raPerp = {-a->angularVelocity * ra.y(), a->angularVelocity * ra.x()};
                Eigen::Vector2f rbPerp = {-b->angularVelocity * rb.y(), b->angularVelocity * rb.x()};
                Eigen::Vector2f relativeVelocity = (b->velocity + rbPerp) - (a->velocity + raPerp);

                float velAlongNormal = relativeVelocity.dot(normal);

                float raCrossN = ra.x() * normal.y() - ra.y() * normal.x();
                float rbCrossN = rb.x() * normal.y() - rb.y() * normal.x();
                float effMass = a->inv_mass + b->inv_mass
                    + raCrossN * raCrossN * a->inv_inertia
                    + rbCrossN * rbCrossN * b->inv_inertia;

                float e = (relativeVelocity.norm() < 0.5f) ? 0.0f : restitution;
                float jRaw = -(1.0f + e) * velAlongNormal / effMass;

                float oldNormal = impulses[ci].normal;
                impulses[ci].normal = std::max(oldNormal + jRaw, 0.0f);
                float jDelta = impulses[ci].normal - oldNormal;

                Eigen::Vector2f impulse = jDelta * normal;
                a->velocity -= impulse * a->inv_mass;
                b->velocity += impulse * b->inv_mass;
                a->angularVelocity -= (ra.x() * impulse.y() - ra.y() * impulse.x()) * a->inv_inertia;
                b->angularVelocity += (rb.x() * impulse.y() - rb.y() * impulse.x()) * b->inv_inertia;

                Eigen::Vector2f raPerp2 = {-a->angularVelocity * ra.y(), a->angularVelocity * ra.x()};
                Eigen::Vector2f rbPerp2 = {-b->angularVelocity * rb.y(), b->angularVelocity * rb.x()};
                Eigen::Vector2f relVelAfter = (b->velocity + rbPerp2) - (a->velocity + raPerp2);

                Eigen::Vector2f tangent = relVelAfter - relVelAfter.dot(normal) * normal;
                float tangentLen = tangent.norm();
                if (tangentLen < 1e-6f) continue;
                tangent /= tangentLen;

                float raCrossT = ra.x() * tangent.y() - ra.y() * tangent.x();
                float rbCrossT = rb.x() * tangent.y() - rb.y() * tangent.x();
                float kTangent = a->inv_mass + b->inv_mass
                    + raCrossT * raCrossT * a->inv_inertia
                    + rbCrossT * rbCrossT * b->inv_inertia;

                float vt = relVelAfter.dot(tangent);
                float jtRaw = -vt / kTangent;

                float maxFriction = 0.55f * impulses[ci].normal;
                float oldFriction = impulses[ci].friction;
                impulses[ci].friction = std::clamp(oldFriction + jtRaw, -maxFriction, maxFriction);
                float jtDelta = impulses[ci].friction - oldFriction;

                Eigen::Vector2f frictionImpulse = jtDelta * tangent;
                a->velocity -= frictionImpulse * a->inv_mass;
                b->velocity += frictionImpulse * b->inv_mass;
                a->angularVelocity -= (ra.x() * frictionImpulse.y() - ra.y() * frictionImpulse.x()) * a->inv_inertia;
                b->angularVelocity += (rb.x() * frictionImpulse.y() - rb.y() * frictionImpulse.x()) * b->inv_inertia;
            }
        }
    }
    for (auto& event: c) {
        PhysicsObject* a = event.objA;
        PhysicsObject* b = event.objB;
        Eigen::Vector2f normal = event.contactNormal;

        float maxDepth = 0.0f;
        for (float d : event.penetrationDepths)
            maxDepth = std::max(maxDepth, d);

        float percent = 0.4f;
        float slop = 0.001f;
        float depth = std::max(maxDepth - slop, 0.0f);

        Eigen::Vector2f correction = percent * depth * normal / (a->inv_mass + b->inv_mass);
        a->position -= correction * a->inv_mass;
        b->position += correction * b->inv_mass;
    }

    for (auto& obj : objects) {
        obj->velocity *= 0.999;
        obj->angularVelocity *= 0.999;

        if (obj->sleeping) continue;
        if (obj->velocity.norm() < 0.01f && fabsf(obj->angularVelocity) < 0.01f) {
            obj->velocity = Eigen::Vector2f::Zero();
            obj->angularVelocity = 0.0f;
            obj->sleepTimer++;
            obj->sleeping = true;
        } else {
            obj->sleepTimer = 0;
        }
        if (obj->sleepTimer > 30) {
            obj->velocity = Eigen::Vector2f::Zero();
            obj->angularVelocity = 0.0f;
            obj->sleeping = true;
            obj->sleepTimer = 0;
        }
    }

}

void PhysicsEngine::updatePhysics(PhysicsObject& obj)
{
    if (obj.stationary || obj.sleeping) {
        return;
    }

    obj.velocity += Eigen::Vector2f(0.0f, g * dt);

    float speed = obj.velocity.norm();
    if (speed > 1e-6f) {
        Eigen::Vector2f drag = dragCoeff * speed * obj.velocity;
        obj.velocity -= drag * dt;
    }

    float angSpeed = obj.angularVelocity;
    if (angSpeed > 1e-6f){
    float angDrag = dragCoeff * angSpeed * fabs(angSpeed);
    obj.angularVelocity -= angDrag * dt;}

    obj.position += obj.velocity * dt;
    obj.rotation += obj.angularVelocity * dt;


    
    

}