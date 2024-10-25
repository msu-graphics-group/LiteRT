#ifndef CBVH_STF_GLFW_WINDOW_H
#define CBVH_STF_GLFW_WINDOW_H

#include "render_common.h"
#include "render_gui.h"

#include "GLFW/glfw3.h"
#include <memory>
#include <unordered_map>


void onKeyboardPressedBasic(GLFWwindow* window, int key, int scancode, int action, int mode);
void onMouseButtonClickedBasic(GLFWwindow* window, int button, int action, int mods);
void onMouseMoveBasic(GLFWwindow* window, double xpos, double ypos);
void onMouseScrollBasic(GLFWwindow* window, double xoffset, double yoffset);

GLFWwindow * initWindow(int width, int height,
                        GLFWkeyfun keyboard = onKeyboardPressedBasic,
                        GLFWcursorposfun mouseMove = onMouseMoveBasic,
                        GLFWmousebuttonfun mouseBtn = onMouseButtonClickedBasic,
                        GLFWscrollfun mouseScroll = onMouseScrollBasic);

void mainLoop(std::shared_ptr<IRender> &app, GLFWwindow* window, bool displayGUI = false);

void setupImGuiContext(GLFWwindow* a_window);

std::unordered_map<std::string, std::string> readCommandLineParams(int argc, const char** argv);

#endif //CBVH_STF_GLFW_WINDOW_H
