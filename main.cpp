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


// =====================================================
// MAIN
// =====================================================
int main()
{
    auto eng = PhysicsEngine();
    std::vector<std::unique_ptr<PhysicsObject>> objs;

    // Circles
    objs.push_back(std::make_unique<Circle>(0.5f, Eigen::Vector2f{2, 2}, Eigen::Vector2f{0.35f, 0}));
    // objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.0f, 0.5f}, Eigen::Vector2f{2, 2}, Eigen::Vector2f{0.4f, 0}, -0.3f)); // 0.3 radians
    // objs.push_back(std::make_unique<Circle>(0.5f, Eigen::Vector2f{4, 4}, Eigen::Vector2f{0, -0.4f}));

    // Rectangles — adjust your Rectangle constructor as needed
    objs.push_back(std::make_unique<Rectangle>(Eigen::Vector2f{1.0f, 0.5f}, Eigen::Vector2f{6, 4}, Eigen::Vector2f{-0.2f, -0.4f}, 0.3f)); // 0.3 radians

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

        cv::imshow("Physics Sim", frame);
        int key = cv::waitKey(16);
        if (key == 'q' || key == 27) break;
    }

    cv::destroyAllWindows();
    return 0;
}