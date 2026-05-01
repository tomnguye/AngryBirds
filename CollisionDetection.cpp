#include "CollisionDetection.hpp"
#include "CollisionEvent.hpp"
#include <stdio.h>

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

 std::optional<CollisionEvent> check_collision(PhysicsObject& obj1, PhysicsObject& obj2) {
    if (obj1.type == CIRCLE && obj2.type == CIRCLE) {
        CollisionEvent collision (&obj1, &obj2, (obj1.position + obj2.position)/2, (obj1.position - obj2.position).normalized());
        return collision;
    }
    if (obj1.type == RECTANGLE && obj2.type == RECTANGLE) {
        satRectRect(static_cast<Rectangle&>(obj1), static_cast<Rectangle&>(obj2));
        return std::nullopt;
    }
    if (obj1.type == CIRCLE && obj2.type == RECTANGLE) {
        circleRectCollision(static_cast<Circle&>(obj1), static_cast<Rectangle&>(obj2));
        std::nullopt;
    }
    if (obj1.type == RECTANGLE && obj2.type == CIRCLE) {
        circleRectCollision(static_cast<Circle&>(obj2), static_cast<Rectangle&>(obj1));
        std::nullopt;
    }
    return std::nullopt;
}


std::vector<CollisionEvent> colliding_objects(std::vector<std::unique_ptr<PhysicsObject>>& objs) {
    std::vector<CollisionEvent> collisions = {}; 
    for (uint i = 0; i < objs.size(); i ++) {
        for (uint j = i + 1; j < objs.size(); j++) {
            if (((*objs[i]).position - (*objs[j]).position).norm() <= (*objs[i]).boundingRadius + (*objs[j]).boundingRadius) { // Can optimise to not use sqrt by squaring both sides
                std::optional<CollisionEvent> collision = check_collision(*objs[i], *objs[j]);
                if (collision.has_value()) {
                    collisions.push_back(std::move(collision.value()));
                }
            }
        }
    }
    return collisions;
}