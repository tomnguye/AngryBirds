#include "PhysicsEngine.hpp"
#include "CollisionEvent.hpp"
#include "CollisionDetection.hpp"
#include <stdio.h>

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

    struct ContactImpulseAccumulator { float normal = 0.f; float friction = 0.f; };
    int totalContacts = 0;
    for (auto& event : c) totalContacts += event.contactPoints.size();
    std::vector<ContactImpulseAccumulator> accumulatedImpulses(totalContacts);

    struct preImpulseData { float targetVelocity = 0.f; };
    std::vector<preImpulseData> preSolveData(totalContacts);

    {
        int ci = 0;
        for (auto& event : c) {
            PhysicsObject* a = event.objA;
            PhysicsObject* b = event.objB;
            Eigen::Vector2f n = event.contactNormal;

            for (int i = 0; i < (int)event.contactPoints.size(); i++, ci++) {
                Eigen::Vector2f contactPoint = event.contactPoints[i];
                Eigen::Vector2f rA = contactPoint - a->position;
                Eigen::Vector2f rB = contactPoint - b->position;

                Eigen::Vector2f vA = a->velocity + Eigen::Vector2f{-a->angularVelocity * rA.y(), a->angularVelocity * rA.x()};
                Eigen::Vector2f vB = b->velocity + Eigen::Vector2f{-b->angularVelocity * rB.y(), b->angularVelocity * rB.x()};
                Eigen::Vector2f vRel = vB - vA;

                float vn = vRel.dot(n);
                float e = (fabsf(vn) < 0.2f) ? 0.0f : restitution;
                preSolveData[ci].targetVelocity = -e * vn;
            }
        }
    }

    for (int iteration = 0; iteration < maxIterations; iteration++) {
        int ci = 0;
        for (auto& event : c) {
            PhysicsObject* a = event.objA;
            PhysicsObject* b = event.objB;
            Eigen::Vector2f n = event.contactNormal;

            for (int i = 0; i < (int)event.contactPoints.size(); i++, ci++) {
                Eigen::Vector2f contactPoint = event.contactPoints[i];
                Eigen::Vector2f rA = contactPoint - a->position;
                Eigen::Vector2f rB = contactPoint - b->position;

                Eigen::Vector2f vA = a->velocity + Eigen::Vector2f{-a->angularVelocity * rA.y(), a->angularVelocity * rA.x()};
                Eigen::Vector2f vB = b->velocity + Eigen::Vector2f{-b->angularVelocity * rB.y(), b->angularVelocity * rB.x()};
                Eigen::Vector2f vRel = vB - vA;

                float vn = vRel.dot(n);

                float rACrossN = rA.x() * n.y() - rA.y() * n.x();
                float rBCrossN = rB.x() * n.y() - rB.y() * n.x();
                float kN = a->inv_mass + b->inv_mass
                    + rACrossN * rACrossN * a->inv_inertia
                    + rBCrossN * rBCrossN * b->inv_inertia;

                float jn = (-vn + preSolveData[ci].targetVelocity) / kN;

                float jnOld = accumulatedImpulses[ci].normal;
                accumulatedImpulses[ci].normal = std::max(jnOld + jn, 0.0f);
                float jnDelta = accumulatedImpulses[ci].normal - jnOld;

                Eigen::Vector2f Jn = jnDelta * n;
                a->velocity -= Jn * a->inv_mass;
                b->velocity += Jn * b->inv_mass;
                a->angularVelocity -= (rA.x() * Jn.y() - rA.y() * Jn.x()) * a->inv_inertia;
                b->angularVelocity += (rB.x() * Jn.y() - rB.y() * Jn.x()) * b->inv_inertia;

                // Recompute relative velocity after normal impulse for friction
                Eigen::Vector2f vA2 = a->velocity + Eigen::Vector2f{-a->angularVelocity * rA.y(), a->angularVelocity * rA.x()};
                Eigen::Vector2f vB2 = b->velocity + Eigen::Vector2f{-b->angularVelocity * rB.y(), b->angularVelocity * rB.x()};
                Eigen::Vector2f vRel2 = vB2 - vA2;

                Eigen::Vector2f vt = vRel2 - vRel2.dot(n) * n;
                float vtLen = vt.norm();
                if (vtLen < 1e-6f) continue;
                Eigen::Vector2f t = vt / vtLen;

                float rACrossT = rA.x() * t.y() - rA.y() * t.x();
                float rBCrossT = rB.x() * t.y() - rB.y() * t.x();
                float kT = a->inv_mass + b->inv_mass
                    + rACrossT * rACrossT * a->inv_inertia
                    + rBCrossT * rBCrossT * b->inv_inertia;

                float jt = -vRel2.dot(t) / kT;

                float coulombLimit = 0.3f * accumulatedImpulses[ci].normal;
                float jtOld = accumulatedImpulses[ci].friction;
                accumulatedImpulses[ci].friction = std::clamp(jtOld + jt, -coulombLimit, coulombLimit);
                float jtDelta = accumulatedImpulses[ci].friction - jtOld;

                Eigen::Vector2f Jf = jtDelta * t;
                a->velocity -= Jf * a->inv_mass;
                b->velocity += Jf * b->inv_mass;
                a->angularVelocity -= (rA.x() * Jf.y() - rA.y() * Jf.x()) * a->inv_inertia;
                b->angularVelocity += (rB.x() * Jf.y() - rB.y() * Jf.x()) * b->inv_inertia;
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
        float slop = 0.01f;
        float depth = std::max(maxDepth - slop, 0.0f);

        Eigen::Vector2f correction = percent * depth * normal / (a->inv_mass + b->inv_mass);
        a->position -= correction * a->inv_mass;
        b->position += correction * b->inv_mass;
    }

    for (auto& obj : objects) {
        obj->velocity *= 0.999;
        obj->angularVelocity *= 0.99;

        if (obj->sleeping) continue;
        if (obj->velocity.norm() < 0.015f && fabsf(obj->angularVelocity) < 0.015f) {
            obj->sleepTimer++;
            obj->velocity = Eigen::Vector2f::Zero();
            obj->angularVelocity = 0.0f;
            obj->sleeping = true;
            obj->sleepTimer = 0;
        } else {
            obj->sleepTimer = 0;
        }
        // if (obj->sleeping) {
        //     float halfPi = M_PI / 2.0f;
        //     float rounded = std::round(obj->rotation / halfPi) * halfPi;
        //     if (fabsf(obj->rotation - rounded) < 0.01f) obj->rotation = rounded;
        // }
    }
}

void PhysicsEngine::updatePhysics(PhysicsObject& obj)
{
    if (obj.stationary || obj.sleeping) {
        return;
    }

    float speed = obj.velocity.norm();
    if (speed > this->maxVelocity) {
        obj.velocity = obj.velocity.normalized() * this->maxVelocity;
    }

    obj.velocity += Eigen::Vector2f(0.0f, g * dt);

    speed = obj.velocity.norm();
    if (speed > 1e-6f) {
        Eigen::Vector2f drag = dragCoeff * speed * obj.velocity;
        obj.velocity -= drag * dt;
    }

    // float angSpeed = obj.angularVelocity;
    // if (angSpeed > 1e-6f){
    // float angDrag = dragCoeff * angSpeed * fabs(angSpeed);
    // obj.angularVelocity -= angDrag * dt;}

    obj.position += obj.velocity * dt;
    obj.rotation += obj.angularVelocity * dt;
}