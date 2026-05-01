#include "PhysicsEngine.hpp"
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
    for (auto& [a,b]: c) {
        std::cerr << "collision" << a << b << c.size() << std::endl;
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
}

float wrapAngle(float a) {
    a = fmodf(a, 2*M_PI);
    if (a > M_PI)  a -= 2*M_PI;
    if (a < -M_PI) a += 2*M_PI;
    return a;
}

// Tests if there is a separating axis along axisRect's local axes
bool hasGap(Rectangle& axisRect, Rectangle& otherRect, Eigen::Vector2f centerOffset)
{
    float halfDiag = otherRect.dimentions.norm() / 2.0f;
    float baseAngle = atan2f(otherRect.dimentions[1], otherRect.dimentions[0]);
    float relRot = otherRect.rotation - axisRect.rotation;

    std::array<float, 4> cornerAngles = {{
         baseAngle,
         (float) M_PI - baseAngle,
         (float) M_PI + baseAngle,
        -baseAngle
    }};

    float cosR = cosf(axisRect.rotation);
    float sinR = sinf(axisRect.rotation);

    float sepX =  centerOffset[0] * cosR + centerOffset[1] * sinR;
    float sepY = -centerOffset[0] * sinR + centerOffset[1] * cosR;

    float extentX = axisRect.dimentions[0] * 0.5f;
    float extentY = axisRect.dimentions[1] * 0.5f;

    auto closestAngle = [&](float target) {
        return *std::min_element(cornerAngles.begin(), cornerAngles.end(),
            [&](float a, float b) {
                return fabsf(wrapAngle(a + relRot - target)) < fabsf(wrapAngle(b + relRot - target));
            });
    };

    float extentOtherX = halfDiag * cosf(closestAngle(0)       + relRot);
    float extentOtherY = halfDiag * cosf(closestAngle(M_PI/2)  + relRot - M_PI/2);

    if (fabsf(sepX) > extentX + fabsf(extentOtherX)) return true;
    if (fabsf(sepY) > extentY + fabsf(extentOtherY)) return true;

    return false;
}

bool satRectRect(Rectangle& a, Rectangle& b)
{
    Eigen::Vector2f centerOffset = a.position - b.position;
    if (hasGap(b, a, centerOffset))  return false;
    if (hasGap(a, b, -centerOffset)) return false;
    return true;
}

bool circleRectCollision(Circle& c, Rectangle& r)
{
    float cosR = cosf(r.rotation);
    float sinR = sinf(r.rotation);

    Eigen::Vector2f offset = c.position - r.position;

    // Project circle center onto rect's local axes
    float localX =  offset[0] * cosR + offset[1] * sinR;
    float localY = -offset[0] * sinR + offset[1] * cosR;

    // Clamp to rect's extents
    float hw = r.dimentions[0] * 0.5f;
    float hh = r.dimentions[1] * 0.5f;
    float clampedX = std::clamp(localX, -hw, hw);
    float clampedY = std::clamp(localY, -hh, hh);

    // Distance from circle center to closest point on rect
    float dx = localX - clampedX;
    float dy = localY - clampedY;
    return (dx*dx + dy*dy) <= c.radius * c.radius;
}

bool check_collision(PhysicsObject& obj1, PhysicsObject& obj2) {
    if (obj1.type == CIRCLE && obj2.type == CIRCLE) {
        return true;
    }
    if (obj1.type == RECTANGLE && obj2.type == RECTANGLE) {
        return satRectRect(static_cast<Rectangle&>(obj1), static_cast<Rectangle&>(obj2));
    }
    if (obj1.type == CIRCLE && obj2.type == RECTANGLE) {
        return circleRectCollision(static_cast<Circle&>(obj1), static_cast<Rectangle&>(obj2));
    }
    if (obj1.type == RECTANGLE && obj2.type == CIRCLE) {
        return circleRectCollision(static_cast<Circle&>(obj2), static_cast<Rectangle&>(obj1));
    }
    return false;
}

std::vector<std::pair<int, int>> PhysicsEngine::colliding_objects(std::vector<std::unique_ptr<PhysicsObject>>& objs) {
    std::vector<std::pair<int, int>> collisions = {}; 
    for (uint i = 0; i < objs.size(); i ++) {
        for (uint j = i + 1; j < objs.size(); j++) {
            if (((*objs[i]).position - (*objs[j]).position).norm() <= (*objs[i]).boundingRadius + (*objs[j]).boundingRadius) { // Can optimise to not use sqrt by squaring both sides
                if (check_collision(*objs[i], *objs[j])) {
                    collisions.push_back(std::pair(i,j));
                }
            }
        }
    }
    return collisions;
}