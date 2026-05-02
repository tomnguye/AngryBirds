#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "PhysicsEngine.hpp"

// =====================================================
// SCREEN SETTINGS
// =====================================================
const int WINDOW_WIDTH  = 600;
const int WINDOW_HEIGHT = 600;
const float SIM_WIDTH   = 10.0f;
const float SIM_HEIGHT  = 10.0f;

cv::Point2i toPixel(float x, float y)
{
    int px = (int)(x / SIM_WIDTH  * WINDOW_WIDTH);
    int py = (int)((1.0f - y / SIM_HEIGHT) * WINDOW_HEIGHT);
    return {px, py};
}

int toPixelRadius(float r)
{
    return (int)(r / SIM_WIDTH * WINDOW_WIDTH);
}

// =====================================================
// DRAWING
// =====================================================
void drawFloor(cv::Mat& frame)
{
    cv::line(frame,
             {0, WINDOW_HEIGHT - 1},
             {WINDOW_WIDTH, WINDOW_HEIGHT - 1},
             cv::Scalar(150, 150, 150), 2);
}

void drawBall(cv::Mat& frame, float x, float y, float radius)
{
    auto center = toPixel(x, y);
    int r = toPixelRadius(radius);
    cv::circle(frame, center, r, cv::Scalar(0, 220, 255), cv::FILLED);
}

void drawRect(cv::Mat& frame, float x, float y, Eigen::Vector2f dims, float rotation, float boundingRadius)
{
    float hw = dims[0] * 0.5f;
    float hh = dims[1] * 0.5f;

    float cosA = cosf(rotation);
    float sinA = sinf(rotation);

    Eigen::Vector2f corners[4] = {
        { -hw, -hh },
        {  hw, -hh },
        {  hw,  hh },
        { -hw,  hh }
    };

    std::vector<cv::Point> pts(4);
    for (int i = 0; i < 4; i++) {
        float rx = corners[i][0] * cosA - corners[i][1] * sinA;
        float ry = corners[i][0] * sinA + corners[i][1] * cosA;
        pts[i] = toPixel(x + rx, y + ry);
    }

    cv::fillPoly(frame, std::vector<std::vector<cv::Point>>{pts},
                 cv::Scalar(0, 220, 255));
    cv::polylines(frame, pts, true, cv::Scalar(0, 180, 210), 1);

    // Debug bounding sphere
    cv::circle(frame, toPixel(x, y), toPixelRadius(boundingRadius),
               cv::Scalar(0, 0, 255), 1);
}

    void drawCollisionEvents(cv::Mat& frame, std::vector<CollisionEvent>& events)
    {
        for (auto& event : events)
        {
            cv::Point2i contact = toPixel(event.contactPoint[0], event.contactPoint[1]);

            // Draw contact point
            cv::circle(frame, contact, 4, cv::Scalar(0, 0, 255), cv::FILLED);

            // Draw normal arrow — scale it so it's visible
            float arrowLen = 1.0f;
            cv::Point2i arrowEnd = toPixel(
                event.contactPoint[0] + event.contactNormal[0] * arrowLen,
                event.contactPoint[1] + event.contactNormal[1] * arrowLen
            );
            cv::arrowedLine(frame, contact, arrowEnd, cv::Scalar(0, 0, 255), 2);

            // Draw opposite normal for objB
            cv::Point2i arrowEndB = toPixel(
                event.contactPoint[0] - event.contactNormal[0] * arrowLen,
                event.contactPoint[1] - event.contactNormal[1] * arrowLen
            );
            cv::arrowedLine(frame, contact, arrowEndB, cv::Scalar(255, 0, 0), 2);
        }
    }
// =====================================================
// MAIN
// =====================================================
int main()
{
    auto eng = PhysicsEngine();
    std::vector<std::unique_ptr<PhysicsObject>> objs;

    // A circle fired at a stack of rectangles
    // objs.push_back(std::make_unique<Circle>(0.4f, Eigen::Vector2f{0.5f, 2.0f}, Eigen::Vector2f{3.5f, 0.5f}));

    // Stack of rectangles in the middle
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{5.0f, 0.5f}, Eigen::Vector2f::Zero(), 0.0f));
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{5.0f, 1.0f}, Eigen::Vector2f::Zero(), 0.0f));
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{5.0f, 1.5f}, Eigen::Vector2f::Zero(), 0.0f));

    // A second circle coming from above at an angle
    // objs.push_back(std::make_unique<Circle>(0.3f, Eigen::Vector2f{6.0f, 9.0f}, Eigen::Vector2f{-1.0f, -2.0f}));

    // A spinning rectangle coming from the right
    // objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.2f, 0.4f}, Eigen::Vector2f{9.0f, 6.0f}, Eigen::Vector2f{-2.0f, -0.5f}, 0.8f, 1.5f));

    // A slow heavy rectangle drifting across
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{0.8f, 0.8f}, Eigen::Vector2f{8.0f, 3.0f}, Eigen::Vector2f{-0.5f, 0.2f}, 0.0f, 0.0f, false, 5.0f));

    // Bottom
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{SIM_WIDTH, 0.2f}, Eigen::Vector2f{SIM_WIDTH/2, 0.1f}, Eigen::Vector2f::Zero(), 0, 0, true));
    // Top
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{SIM_WIDTH, 0.2f}, Eigen::Vector2f{SIM_WIDTH/2, SIM_HEIGHT - 0.1f}, Eigen::Vector2f::Zero(), 0, 0, true));
    // Left
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{0.2f, SIM_HEIGHT}, Eigen::Vector2f{0.1f, SIM_HEIGHT/2}, Eigen::Vector2f::Zero(), 0, 0, true));
    // Right
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{0.2f, SIM_HEIGHT}, Eigen::Vector2f{SIM_WIDTH - 0.1f, SIM_HEIGHT/2}, Eigen::Vector2f::Zero(), 0, 0, true));

    eng.setState(std::move(objs));

    cv::namedWindow("Physics Sim", cv::WINDOW_AUTOSIZE);

    while (true)
    {
        cv::Mat frame(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(12, 12, 12));

        eng.advanceState();
        drawFloor(frame);

        const auto& s = eng.getState();
        for (const auto& obj : s)
        {
            if (obj->type == CIRCLE) {
                auto* c = static_cast<Circle*>(obj.get());
                drawBall(frame, c->position[0], c->position[1], c->radius);
            }
            else if (obj->type == RECTANGLE) {
                auto* r = static_cast<Rectangle*>(obj.get());
                drawRect(frame, r->position[0], r->position[1],
                        r->dimentions, r->rotation, r->boundingRadius);
            }
        }
        auto collisions = eng.getCollisions();
        drawCollisionEvents(frame, collisions);

        cv::imshow("Physics Sim", frame);
        int key = cv::waitKey(16);
        if (key == 'q' || key == 27) break;
    }

    cv::destroyAllWindows();
    return 0;
}