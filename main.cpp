// Controls:
//   Click + drag the ball to pull the slingshot, release to fire
//   R       — reset slingshot
//   Q / Esc — quit
//   T       — reset entire scene

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "PhysicsEngine.hpp"

// =====================================================
// WINDOW / SIM SETTINGS
// =====================================================
const int   WINDOW_WIDTH  = 1800;
const int   WINDOW_HEIGHT = 600;
const float SIM_WIDTH     = 30.0f;
const float SIM_HEIGHT    = 10.0f;

inline float ndcX(float x) { return (x / SIM_WIDTH)  * 2.0f - 1.0f; }
inline float ndcY(float y) { return (y / SIM_HEIGHT) * 2.0f - 1.0f; }

// =====================================================
// SLINGSHOT SETTINGS
// =====================================================
const Eigen::Vector2f SLING_POS    = { 4.0f, 3.0f };
const float           SLING_RADIUS = 0.2f;
const float           MAX_PULL     = 2.5f;
const float           LAUNCH_FORCE = 8.0f;

struct Slingshot {
    bool            dragging = false;
    bool            launched = false;
    Eigen::Vector2f pullPos  = SLING_POS;
};

// Globals needed by GLFW callbacks
static GLFWwindow*    g_window = nullptr;
static PhysicsEngine* g_eng    = nullptr;
static Slingshot*     g_sling  = nullptr;

// pause functionality
static bool g_paused = false;


// =====================================================
// COORDINATE HELPERS
// =====================================================
Eigen::Vector2f mouseToSim(GLFWwindow* window)
{
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    return {
        (float)mx / WINDOW_WIDTH  * SIM_WIDTH,
        (1.0f - (float)my / WINDOW_HEIGHT) * SIM_HEIGHT
    };
}

// =====================================================
// MOUSE CALLBACK
// =====================================================
void mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    if (action == GLFW_PRESS && !g_sling->launched) {
        Eigen::Vector2f m = mouseToSim(window);
        if ((m - SLING_POS).norm() < SLING_RADIUS + 0.4f)
            g_sling->dragging = true;
    }

    if (action == GLFW_RELEASE && g_sling->dragging) {
        g_sling->dragging = false;
        g_sling->launched = true;

        Eigen::Vector2f pull = g_sling->pullPos - SLING_POS;
        Eigen::Vector2f vel  = -pull * LAUNCH_FORCE;

        g_eng->addObject(std::make_unique<Circle>(
            SLING_RADIUS,
            g_sling->pullPos,
            vel
        ));
    }
}

// =====================================================
// SHADERS
// =====================================================
static const char* VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

static const char* FRAG_SRC = R"(
#version 330 core
uniform vec3 uColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(uColor, 1.0);
}
)";

GLuint compileShader(GLenum type, const char* src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "Shader error: " << log << "\n";
    }
    return s;
}

GLuint buildProgram()
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, VERT_SRC);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, FRAG_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// =====================================================
// GEOMETRY HELPERS
// =====================================================
void drawCircle(std::vector<float>& verts,
                float cx, float cy, float radius, int segments = 32)
{
    float nx = ndcX(cx), ny = ndcY(cy);
    float rx = radius / SIM_WIDTH  * 2.0f;
    float ry = radius / SIM_HEIGHT * 2.0f;
    for (int i = 0; i < segments; ++i) {
        float a0 = (float)i       / segments * 2.0f * M_PI;
        float a1 = (float)(i + 1) / segments * 2.0f * M_PI;
        verts.push_back(nx);               verts.push_back(ny);
        verts.push_back(nx + cosf(a0)*rx); verts.push_back(ny + sinf(a0)*ry);
        verts.push_back(nx + cosf(a1)*rx); verts.push_back(ny + sinf(a1)*ry);
    }
}

void drawRect(std::vector<float>& verts,
              float cx, float cy, float hw, float hh, float rotation)
{
    float cosA = cosf(rotation), sinA = sinf(rotation);
    Eigen::Vector2f c[4] = {
        {-hw,-hh}, {hw,-hh}, {hw,hh}, {-hw,hh}
    };
    float px[4], py[4];
    for (int i = 0; i < 4; ++i) {
        px[i] = ndcX(c[i][0]*cosA - c[i][1]*sinA + cx);
        py[i] = ndcY(c[i][0]*sinA + c[i][1]*cosA + cy);
    }
    verts.insert(verts.end(), {px[0],py[0], px[1],py[1], px[2],py[2]});
    verts.insert(verts.end(), {px[0],py[0], px[2],py[2], px[3],py[3]});
}

void drawLine(std::vector<float>& verts,
              float x0, float y0, float x1, float y1, float thickness = 0.004f)
{
    float dx = x1-x0, dy = y1-y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 1e-6f) return;
    float nx = -dy/len*thickness, ny = dx/len*thickness;

    float ax=x0+nx, ay=y0+ny, bx=x0-nx, by=y0-ny;
    float cx=x1+nx, cy=y1+ny, dx2=x1-nx, dy2=y1-ny;

    verts.insert(verts.end(), {ax,ay, bx,by, cx,cy});
    verts.insert(verts.end(), {bx,by, dx2,dy2, cx,cy});
}

// =====================================================
// DRAW HELPERS
// =====================================================
void drawBatch(GLuint vao, GLuint vbo,
               const std::vector<float>& verts,
               GLuint prog, float r, float g, float b)
{
    if (verts.empty()) return;
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 verts.size() * sizeof(float),
                 verts.data(), GL_STREAM_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glUniform3f(glGetUniformLocation(prog, "uColor"), r, g, b);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(verts.size() / 2));
}

// draw initial scene
std::vector<std::unique_ptr<PhysicsObject>> createInitialState()
{
    std::vector<std::unique_ptr<PhysicsObject>> objs;

    // Stack of rectangles to knock over
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{13.0f, 1.5f},
        Eigen::Vector2f::Zero(), M_PI/2));
    
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{20.0f, 1.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{21.0f, 1.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{22.0f, 1.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{23.0f, 1.0f},
        Eigen::Vector2f::Zero(), M_PI/2));

    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{20.5f, 1.5f},
        Eigen::Vector2f::Zero(), 0.0f));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{22.5f, 1.5f},
        Eigen::Vector2f::Zero(), 0.0f));

    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{20.0f, 3.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{21.0f, 3.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{22.0f, 3.0f},
        Eigen::Vector2f::Zero(), M_PI/2));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{23.0f, 3.0f},
        Eigen::Vector2f::Zero(), M_PI/2));

    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{20.5f, 4.5f},
        Eigen::Vector2f::Zero(), 0.0f));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{1.5f, 0.3f}, Eigen::Vector2f{22.5f, 4.5f},
        Eigen::Vector2f::Zero(), 0.0f));

     // Boundary walls
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{SIM_WIDTH, 0.2f},
        Eigen::Vector2f{SIM_WIDTH/2, 0.1f},
        Eigen::Vector2f::Zero(), 0, 0, true, 10, true));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{SIM_WIDTH, 0.2f},
        Eigen::Vector2f{SIM_WIDTH/2, SIM_HEIGHT - 0.1f},
        Eigen::Vector2f::Zero(), 0, 0, true, 10, true));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{0.2f, SIM_HEIGHT},
        Eigen::Vector2f{0.1f, SIM_HEIGHT/2},
        Eigen::Vector2f::Zero(), 0, 0, true, 10, true));
    objs.push_back(std::make_unique<Rectangle>(
        Eigen::Vector2f{0.2f, SIM_HEIGHT},
        Eigen::Vector2f{SIM_WIDTH - 0.1f, SIM_HEIGHT/2},
        Eigen::Vector2f::Zero(), 0, 0, true, 10, true));

    return objs;
}
    

// =====================================================
// MAIN
// =====================================================
static bool pPressedLastFrame = false;
int main()
{

   


    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
                                          "Physics Sim", nullptr, nullptr);
    if (!window) { std::cerr << "Window creation failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);


    

    // run at screen refresh rate 
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::cerr << "GLEW init failed\n"; return 1; }
    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    GLuint prog = buildProgram();
    glUseProgram(prog);

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    // --- Scene ---
    PhysicsEngine eng;
    // std::vector<std::unique_ptr<PhysicsObject>> objs;
    // eng.setState(std::move(objs));

    eng.setState(createInitialState());

    // --- Slingshot ---
    Slingshot sling;
    g_window = window;
    g_eng    = &eng;
    g_sling  = &sling;
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    // =====================================================
    // MAIN LOOP
    // =====================================================
    double lastTime = glfwGetTime();
    double accumulator = 0.0;

    const double fixedDt = 1.0 / 60.0; // 60 Hz physics
    while (!glfwWindowShouldClose(window))
    {  

        double currentTime = glfwGetTime();
        double frameTime = currentTime - lastTime;
        lastTime = currentTime;

        // avoid spiral of death
        if (frameTime > 0.25)
            frameTime = 0.25;

        // accumulator += frameTime;
        if (!g_paused){
            accumulator += frameTime;
        }
        else{
            // freeze time while paused
            lastTime = currentTime;
        }


        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_Q)      == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // Reset slingshot
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
            sling.launched = false;
            sling.dragging = false;
            sling.pullPos  = SLING_POS;
        }

        // Update pull position while dragging
        // need to fix power and direction
        if (sling.dragging) {
            Eigen::Vector2f m    = mouseToSim(window);
            Eigen::Vector2f pull = m - SLING_POS;
            if (pull.norm() > MAX_PULL)
                pull = pull.normalized() * MAX_PULL;
            sling.pullPos = SLING_POS + pull;
        }

        //pause functionality
        static bool g_paused = false;
        bool pPressed = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;

        if (pPressed && !pPressedLastFrame)
        {
            g_paused = !g_paused;

            accumulator = 0.0;
            lastTime = glfwGetTime(); // prevents time jump

            std::cout << (g_paused ? "Paused\n" : "Resumed\n");
        }

        pPressedLastFrame = pPressed;

        if (!g_paused) {
            while (accumulator >= fixedDt) {
                eng.advanceState();
                accumulator -= fixedDt;
            }
        }

        // reset functionality
        bool tPressed = glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS;
        static bool tPressedLast = false;

        if (tPressed && !tPressedLast)
        {
            eng.setState(createInitialState());

            // reset slingshot
            sling.launched = false;
            sling.dragging = false;
            sling.pullPos  = SLING_POS;

            // reset timing
            accumulator = 0.0;

            g_paused = false;

            std::cout << "Game Reset\n";
        }

        tPressedLast = tPressed;



        auto collisions = eng.getCollisions();

        glClearColor(0.047f, 0.047f, 0.047f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(prog);

        // std::vector<float> shapeVerts;
        std::vector<float> birdVerts;
        std::vector<float> blockVerts;
        std::vector<float> groundVerts;
        std::vector<float> bandVerts;
        std::vector<float> contactVerts;
        std::vector<float> normalVertsA;
        std::vector<float> normalVertsB;

        // Physics objects
        for (const auto& obj : eng.getState())
        {
            if (obj->type == CIRCLE) {
                auto* c = static_cast<Circle*>(obj.get());
                drawCircle(birdVerts, c->position[0], c->position[1], c->radius);

                float cx = c->position[0];
                float cy = c->position[1];

                float angle = c->rotation;

                float mx = cx + cosf(angle) * c->radius;
                float my = cy + sinf(angle) * c->radius;

                drawCircle(birdVerts, mx, my, 0.05f);
            }
            else if (obj->type == RECTANGLE) {
                auto* r = static_cast<Rectangle*>(obj.get());

                if (r -> isBoundary){
                    drawRect(groundVerts,
                    r->position[0], r->position[1],
                    r->dimentions[0] * 0.5f,
                    r->dimentions[1] * 0.5f,
                    r->rotation);
                } 
                else{
                    drawRect(blockVerts,
                    r->position[0], r->position[1],
                    r->dimentions[0] * 0.5f,
                    r->dimentions[1] * 0.5f,
                    r->rotation);
                }
            }
        }

        // slingshot — ball + elastic bands
        if (!sling.launched)
        {
            drawCircle(birdVerts, sling.pullPos[0], sling.pullPos[1], SLING_RADIUS);

            // Two fork tips slightly left/right of the anchor
            float forkLX = ndcX(SLING_POS[0] - 0.15f);
            float forkRX = ndcX(SLING_POS[0] + 0.15f);
            float forkY  = ndcY(SLING_POS[1] + 0.2f);
            float ballX  = ndcX(sling.pullPos[0]);
            float ballY  = ndcY(sling.pullPos[1]);

            drawLine(bandVerts, forkLX, forkY, ballX, ballY, 0.006f);
            drawLine(bandVerts, forkRX, forkY, ballX, ballY, 0.006f);
        }

        

        // Collision debug
        // float arrowLen = 0.8f;
        // for (auto& ev : collisions)
        // {
        //     float cx = ev.contactPoint[0], cy = ev.contactPoint[1];
        //     float nx = ev.contactNormal[0], ny = ev.contactNormal[1];
        //     drawCircle(contactVerts, cx, cy, 0.08f, 12);
        //     drawLine(normalVertsA,
        //              ndcX(cx), ndcY(cy),
        //              ndcX(cx + nx*arrowLen), ndcY(cy + ny*arrowLen));
        //     drawLine(normalVertsB,
        //              ndcX(cx), ndcY(cy),
        //              ndcX(cx - nx*arrowLen), ndcY(cy - ny*arrowLen));
        // }

        // drawBatch(vao, vbo, shapeVerts,   prog, 0.0f, 0.863f, 1.0f); // cyan   — objects
        drawBatch(vao, vbo, birdVerts,  prog, 0.9f, 0.2f, 0.2f); // birds (red)
        drawBatch(vao, vbo, blockVerts, prog, 0.6f, 0.4f, 0.2f); // blocks (brown)
        drawBatch(vao, vbo, groundVerts, prog, 0.2f, 0.7f, 0.2f);// ground (green)
        drawBatch(vao, vbo, bandVerts,    prog, 0.9f, 0.7f,   0.2f); // yellow — elastic bands
        drawBatch(vao, vbo, contactVerts, prog, 1.0f, 0.0f,   0.0f); // red    — contact points
        drawBatch(vao, vbo, normalVertsA, prog, 1.0f, 0.0f,   0.0f); // red    — normal A
        drawBatch(vao, vbo, normalVertsB, prog, 0.0f, 0.0f,   1.0f); // blue   — normal B

        glfwSwapBuffers(window);
    }

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(prog);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}