#include "CollisionDetection.hpp"
#include "CollisionEvent.hpp"
#include <stdio.h>

float wrapAngle(float a) {
    a = fmodf(a, 2 * M_PI);
    if (a > M_PI)
        a -= 2 * M_PI;
    if (a < -M_PI)
        a += 2 * M_PI;
    return a;
}

std::array<Eigen::Vector2f, 4> getVertices(Rectangle &r) {
    float hw = r.dimentions[0] * 0.5f;
    float hh = r.dimentions[1] * 0.5f;
    float cosA = cosf(r.rotation);
    float sinA = sinf(r.rotation);
    return {{
        r.position +
            Eigen::Vector2f{cosA * hw - sinA * hh, sinA * hw + cosA * hh},
        r.position +
            Eigen::Vector2f{-cosA * hw - sinA * hh, -sinA * hw + cosA * hh},
        r.position +
            Eigen::Vector2f{-cosA * hw + sinA * hh, -sinA * hw - cosA * hh},
        r.position +
            Eigen::Vector2f{cosA * hw + sinA * hh, sinA * hw - cosA * hh},
    }};
}

std::vector<Eigen::Vector2f> clipEdge(Eigen::Vector2f v0, Eigen::Vector2f v1,
                                      Eigen::Vector2f planePoint,
                                      Eigen::Vector2f planeNormal) {
    std::vector<Eigen::Vector2f> result;
    float d0 = planeNormal.dot(v0 - planePoint);
    float d1 = planeNormal.dot(v1 - planePoint);

    if (d0 >= 0 && d1 >= 0) {
        // Both inside
        result.push_back(v0);
        result.push_back(v1);
    } else if (d0 >= 0 && d1 < 0) {
        // v0 inside, v1 outside
        float t = d0 / (d0 - d1);
        result.push_back(v0);
        result.push_back(v0 + t * (v1 - v0));
    } else if (d0 < 0 && d1 >= 0) {
        // v0 outside, v1 inside
        float t = d0 / (d0 - d1);
        result.push_back(v0 + t * (v1 - v0));
        result.push_back(v1);
    }
    // Both outside
    return result;
}

std::vector<Eigen::Vector2f> getContactPoint(Rectangle &axisRect,
                                             Rectangle &otherRect,
                                             Eigen::Vector2f normal) {
    auto vertsA = getVertices(axisRect);
    auto vertsB = getVertices(otherRect);

    // Find reference face on axisRect
    int refIdx = 0;
    float maxDot = -INFINITY;
    for (int i = 0; i < 4; i++) {
        Eigen::Vector2f edge = (vertsA[(i + 1) % 4] - vertsA[i]).normalized();
        Eigen::Vector2f faceNormal = {edge[1], -edge[0]};
        float d = faceNormal.dot(normal);
        if (d > maxDot) {
            maxDot = d;
            refIdx = i;
        }
    }
    Eigen::Vector2f refV0 = vertsA[refIdx];
    Eigen::Vector2f refV1 = vertsA[(refIdx + 1) % 4];
    Eigen::Vector2f refEdge = (refV1 - refV0).normalized();
    Eigen::Vector2f refNormal = {refEdge[1], -refEdge[0]};

    // Find incident face on otherRect
    int incidentIdx = 0;
    float minDot = INFINITY;
    for (int i = 0; i < 4; i++) {
        float d = normal.dot(vertsB[i]);
        if (d < minDot) {
            minDot = d;
            incidentIdx = i;
        }
    }
    int prev = (incidentIdx + 3) % 4;
    int next = (incidentIdx + 1) % 4;
    Eigen::Vector2f edgePrev =
        (vertsB[incidentIdx] - vertsB[prev]).normalized();
    Eigen::Vector2f edgeNext =
        (vertsB[incidentIdx] - vertsB[next]).normalized();
    int incidentEnd =
        fabsf(edgePrev.dot(normal)) < fabsf(edgeNext.dot(normal)) ? prev : next;

    Eigen::Vector2f v0 = vertsB[incidentIdx];
    Eigen::Vector2f v1 = vertsB[incidentEnd];

    // Clip incident edge against side planes of reference face
    auto clipped = clipEdge(v0, v1, refV0, refEdge);
    if (clipped.size() < 2) {
        return {(axisRect.position + otherRect.position) / 2.0f};
    }
    clipped = clipEdge(clipped[0], clipped[1], refV1, -refEdge);
    if (clipped.size() < 2) {
        return {(axisRect.position + otherRect.position) / 2.0f};
    }

    // Keep only points behind reference face
    std::vector<Eigen::Vector2f> contacts;
    for (auto &p : clipped) {
        if (refNormal.dot(p - refV0) <= 0.01f) {
            contacts.push_back(p);
        }
    }

    if (contacts.empty()) {
        return {(axisRect.position + otherRect.position) / 2.0f};
    }
    return contacts;
}

std::optional<CollisionEvent> testAxes(Rectangle &axisRect,
                                       Rectangle &otherRect,
                                       Eigen::Vector2f centerOffset) {
    float halfDiag = otherRect.dimentions.norm() / 2.0f;
    float baseAngle = atan2f(otherRect.dimentions[1], otherRect.dimentions[0]);
    float relRot = otherRect.rotation - axisRect.rotation;

    std::array<float, 4> cornerAngles = {{baseAngle, (float)M_PI - baseAngle,
                                          (float)M_PI + baseAngle, -baseAngle}};

    float cosR = cosf(axisRect.rotation);
    float sinR = sinf(axisRect.rotation);

    float sepX = centerOffset[0] * cosR + centerOffset[1] * sinR;
    float sepY = -centerOffset[0] * sinR + centerOffset[1] * cosR;

    float extentX = axisRect.dimentions[0] * 0.5f;
    float extentY = axisRect.dimentions[1] * 0.5f;

    auto closestAngle = [&](float target) {
        return *std::min_element(
            cornerAngles.begin(), cornerAngles.end(), [&](float a, float b) {
                return fabsf(wrapAngle(a + relRot - target)) <
                       fabsf(wrapAngle(b + relRot - target));
            });
    };

    float extentOtherX = halfDiag * cosf(closestAngle(0) + relRot);
    float extentOtherY =
        halfDiag * cosf(closestAngle(M_PI / 2) + relRot - M_PI / 2);

    float penX = extentX + fabsf(extentOtherX) - fabsf(sepX);
    float penY = extentY + fabsf(extentOtherY) - fabsf(sepY);

    if (penX < 0 || penY < 0)
        return std::nullopt;

    Eigen::Vector2f normal;
    float penetration;
    if (penX < penY) {
        normal = {cosR, sinR};
        if (sepX < 0)
            normal = -normal;
        penetration = penX;
    } else {
        normal = {-sinR, cosR};
        if (sepY < 0)
            normal = -normal;
        penetration = penY;
    }

    std::vector<Eigen::Vector2f> contacts =
        getContactPoint(axisRect, otherRect, normal);
    std::vector<float> penetrations(contacts.size(), penetration);
    return CollisionEvent(&axisRect, &otherRect, penetrations, contacts,
                          normal);
}
// SAT method for collision detection
std::optional<CollisionEvent> satRectRect(Rectangle &a, Rectangle &b) {
    Eigen::Vector2f centerOffset = a.position - b.position;

    auto resultA = testAxes(b, a, centerOffset);
    auto resultB = testAxes(a, b, -centerOffset);

    if (!resultA.has_value())
        return std::nullopt;
    if (!resultB.has_value())
        return std::nullopt;

    // Return the axis with smallest penetration
    auto minPen = [](const CollisionEvent &e) {
        return *std::min_element(e.penetrationDepths.begin(),
                                 e.penetrationDepths.end());
    };

    if (minPen(resultA.value()) < minPen(resultB.value()))
        return resultA.value();
    return resultB.value();
}

std::optional<CollisionEvent> circleRectCollision(Circle &c, Rectangle &r) {
    float cosR = cosf(r.rotation);
    float sinR = sinf(r.rotation);

    Eigen::Vector2f offset = c.position - r.position;

    // Project circle center onto rect's local axescontactWorld
    float localX = offset[0] * cosR + offset[1] * sinR;
    float localY = -offset[0] * sinR + offset[1] * cosR;

    // Clamp to rect's extents
    float hw = r.dimentions[0] * 0.5f;
    float hh = r.dimentions[1] * 0.5f;
    float clampedX = std::clamp(localX, -hw, hw);
    float clampedY = std::clamp(localY, -hh, hh);

    // Distance from circle center to closest point on rect
    float dx = localX - clampedX;
    float dy = localY - clampedY;
    if ((dx * dx + dy * dy) > c.radius * c.radius) {
        return std::nullopt;
    }
    float penetrationDepth = c.radius - sqrtf(dx * dx + dy * dy);
    Eigen::Vector2f contactWorld = {
        r.position[0] + clampedX * cosR - clampedY * sinR,
        r.position[1] + clampedX * sinR + clampedY * cosR};
    Eigen::Vector2f normal = (contactWorld - c.position);
    float len = normal.norm();
    if (len > 1e-6f)
        normal /= len;
    else
        normal = {cosR, sinR};

    return CollisionEvent(&c, &r, {penetrationDepth}, {contactWorld}, normal);
}

std::optional<CollisionEvent> check_collision(PhysicsObject &obj1,
                                              PhysicsObject &obj2) {
    if (obj1.stationary && obj2.stationary) {
        return std::nullopt;
    }
    if (obj1.type == CIRCLE && obj2.type == CIRCLE) {
        Circle circle1 = static_cast<Circle &>(obj1);
        Circle circle2 = static_cast<Circle &>(obj2);
        float penetrationDepth = circle1.radius + circle2.radius -
                                 (circle2.position - circle1.position).norm();
        Eigen::Vector2f normal =
            (circle2.position - circle1.position).normalized();
        Eigen::Vector2f contactPoint =
            circle1.position +
            normal * circle1.radius; // Assume that the collision point is the
                                     // edge of circle 1.
        CollisionEvent collision(&obj1, &obj2, {penetrationDepth},
                                 {contactPoint}, normal);
        return collision;
    }
    if (obj1.type == RECTANGLE && obj2.type == RECTANGLE) {
        return satRectRect(static_cast<Rectangle &>(obj1),
                           static_cast<Rectangle &>(obj2));
    }
    if (obj1.type == CIRCLE && obj2.type == RECTANGLE) {
        return circleRectCollision(static_cast<Circle &>(obj1),
                                   static_cast<Rectangle &>(obj2));
    }
    if (obj1.type == RECTANGLE && obj2.type == CIRCLE) {
        return circleRectCollision(static_cast<Circle &>(obj2),
                                   static_cast<Rectangle &>(obj1));
    }
    return std::nullopt;
}

std::vector<CollisionEvent>
colliding_objects(std::vector<std::unique_ptr<PhysicsObject>> &objs) {
    std::vector<CollisionEvent> collisions = {};
    for (uint i = 0; i < objs.size(); i++) {
        for (uint j = i + 1; j < objs.size(); j++) {
            if (((*objs[i]).position - (*objs[j]).position).norm() <=
                (*objs[i]).boundingRadius +
                    (*objs[j]).boundingRadius) { // Can optimise to not use sqrt
                                                 // by squaring both sides
                std::optional<CollisionEvent> collision =
                    check_collision(*objs[i], *objs[j]);
                if (collision.has_value()) {
                    collisions.push_back(std::move(collision.value()));
                }
            }
        }
    }
    return collisions;
}