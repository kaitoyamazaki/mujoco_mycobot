#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

// グローバル変数
mjModel* m = nullptr;        // MuJoCo model
mjData* d = nullptr;         // MuJoCo data
mjvCamera cam;               // カメラ
mjvOption opt;               // オプション
mjvScene scn;                // シーン
mjrContext con;              // コンテキスト

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// シミュレーションパラメータ
const double DEG_TO_RAD = M_PI / 180.0;
const double INCREMENT = 0.25 * DEG_TO_RAD;  // 2°ずつ変化
double joint_targets[3] = {0.0, 0.0, 0.0};
double joint_final[3] = {-50 * DEG_TO_RAD, -120 * DEG_TO_RAD, 80 * DEG_TO_RAD};

// マウスボタンコールバック
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update last click position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// カーソル位置コールバック
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // determine action based on mouse button
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// スクロールコールバック
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// キーコールバック
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_R) {
            // シミュレーションをリセット
            mj_resetData(m, d);
            joint_targets[0] = 0.0;
            joint_targets[1] = 0.0;
            joint_targets[2] = 0.0;
        }
    }
}

void move_joints() {
    // Set control targets
    d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "joint2")] = joint_targets[0];
    d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "joint3")] = joint_targets[1];
    d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "joint4")] = joint_targets[2];
}

void set_initial_camera() {
    cam.azimuth = -135;   // 方位角
    cam.elevation = -20; // 高度
    cam.distance = 1.25;   // 距離
    cam.lookat[0] = 0;  // 注視点のx座標
    cam.lookat[1] = 0;  // 注視点のy座標
    cam.lookat[2] = 0;  // 注視点のz座標
}

void simulate(const char* filename) {
    char error[1000] = "Could not load model";
    m = mj_loadXML(filename, nullptr, error, 1000);
    if (!m) {
        std::cerr << "Error loading model: " << error << std::endl;
        return;
    }
    d = mj_makeData(m);
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    set_initial_camera();  // 初期カメラ位置を設定

    // コールバック関数を設定
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key_callback);

    while (!glfwWindowShouldClose(window)) {
        bool reached_target = true;
        for (int i = 0; i < 3; ++i) {
            if (joint_targets[i] < joint_final[i]) {
                joint_targets[i] += INCREMENT;
                if (joint_targets[i] > joint_final[i]) joint_targets[i] = joint_final[i];
                reached_target = false;
            } else if (joint_targets[i] > joint_final[i]) {
                joint_targets[i] -= INCREMENT;
                if (joint_targets[i] < joint_final[i]) joint_targets[i] = joint_final[i];
                reached_target = false;
            }
        }

        if (!reached_target) {
            // 関節を移動
            move_joints();
        }

        // シミュレーションステップ
        mj_step(m, d);
        
        // レンダリング
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
        return 1;
    }
    simulate(argv[1]);
    return 0;
}
