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
        Eigen::Vector2f normal = event.contactNormal;

        float percent = 0.9f;
        float slop = 0.005f;

        float maxDepth = 0.0f;
        for (float d : event.penetrationDepths)
            maxDepth = std::max(maxDepth, d);

        float depth = std::max(maxDepth - slop, 0.0f);

        Eigen::Vector2f correction =
            percent * depth * normal / (a->inv_mass + b->inv_mass);
        a->position -= correction * a->inv_mass;
        b->position += correction * b->inv_mass;
    }

    for (int j = 0; j < maxIterations; j++) {


        std::unordered_map<PhysicsObject*, Eigen::Vector2f> v0;
        std::unordered_map<PhysicsObject*, float> w0;

        for (auto& obj : objects) {
            v0[obj.get()] = obj->velocity;
            w0[obj.get()] = obj->angularVelocity;
        }

        for (auto& event : c) {
            PhysicsObject* a = event.objA;
            PhysicsObject* b = event.objB;
            Eigen::Vector2f normal = event.contactNormal;

            for (int i = 0; i < (int)event.contactPoints.size(); i++) {

                Eigen::Vector2f point = event.contactPoints[i];

                Eigen::Vector2f ra = point - a->position;
                Eigen::Vector2f rb = point - b->position;

                Eigen::Vector2f raPerp = {-w0[a] * ra.y(), w0[a] * ra.x()};
                Eigen::Vector2f rbPerp = {-w0[b] * rb.y(), w0[b] * rb.x()};

                Eigen::Vector2f relativeVelocity =
                    (v0[b] + rbPerp) - (v0[a] + raPerp);

                float velAlongNormal = relativeVelocity.dot(normal);

                if (velAlongNormal <= 0) {

                    float raCrossN = ra.x() * normal.y() - ra.y() * normal.x();
                    float rbCrossN = rb.x() * normal.y() - rb.y() * normal.x();

                    float j = -(1.0f + restitution) * velAlongNormal;

                    j /= a->inv_mass + b->inv_mass
                        + raCrossN * raCrossN * a->inv_inertia
                        + rbCrossN * rbCrossN * b->inv_inertia;

                    Eigen::Vector2f impulse = j * normal;

                    v0[a] -= impulse * a->inv_mass;
                    v0[b] += impulse * b->inv_mass;

                    w0[a] -= (ra.x() * impulse.y() - ra.y() * impulse.x()) * a->inv_inertia;
                    w0[b] += (rb.x() * impulse.y() - rb.y() * impulse.x()) * b->inv_inertia;

                    Eigen::Vector2f tangent =
                        relativeVelocity - relativeVelocity.dot(normal) * normal;

                    float tlen = tangent.norm();
                    if (tlen < 1e-6f) continue;
                    tangent /= tlen;

                    float raCrossT = ra.x() * tangent.y() - ra.y() * tangent.x();
                    float rbCrossT = rb.x() * tangent.y() - rb.y() * tangent.x();

                    float kT =
                        a->inv_mass + b->inv_mass
                        + raCrossT * raCrossT * a->inv_inertia
                        + rbCrossT * rbCrossT * b->inv_inertia;

                    float vt = relativeVelocity.dot(tangent);

                    float jt = -vt / kT;

                    float mu = 0.3f;
                    float maxFriction = std::abs(j) * mu;

                    jt = std::clamp(jt, -maxFriction, maxFriction);

                    Eigen::Vector2f frictionImpulse = jt * tangent;

                    v0[a] -= frictionImpulse * a->inv_mass;
                    v0[b] += frictionImpulse * b->inv_mass;

                    w0[a] -= (ra.x() * frictionImpulse.y() - ra.y() * frictionImpulse.x()) * a->inv_inertia;
                    w0[b] += (rb.x() * frictionImpulse.y() - rb.y() * frictionImpulse.x()) * b->inv_inertia;
                }
            }
        }

        for (auto& obj : objects) {
            obj->velocity = v0[obj.get()];
            obj->angularVelocity = w0[obj.get()];
        }
    }
    
    for (auto& obj : objects) {
        obj->velocity *= 0.999;
        obj->angularVelocity *= 0.995;
        if (fabsf(obj->angularVelocity) < 0.02f) obj->angularVelocity = 0.0f;
        if (obj->velocity.norm() < 0.02f) obj->velocity = Eigen::Vector2f::Zero();
    }
    
}

void PhysicsEngine::updatePhysics(PhysicsObject& obj)
{
    if (obj.stationary) {
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