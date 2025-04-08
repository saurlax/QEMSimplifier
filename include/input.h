#ifndef INPUT_H
#define INPUT_H

#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include "camera.h"

// Mouse control variables
extern float lastX;
extern float lastY;
extern bool firstMouse;
extern bool leftMousePressed;
extern bool rightMousePressed;

// Simplification control
extern float simplificationRatio;
extern bool simplificationRatioChanged;
extern bool decreaseRatioKeyPressed;
extern bool increaseRatioKeyPressed;
extern bool exportModelKeyPressed;

// Camera reference
extern Camera* currentCamera;

// Timing
extern float deltaTime;

// Error callback
void error_callback(int error, const char* description);

// Process keyboard input
void processInput(GLFWwindow* window, const std::string& modelPath);

// Mouse movement callback
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

// Mouse button callback
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

// Scroll callback
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// Implementation

void error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

void processInput(GLFWwindow* window, const std::string& modelPath)
{
    // Add keyboard controls
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
        
    // WASD movement controls
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        currentCamera->ProcessKeyboard(1.0f, 0.0f, 0.0f, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        currentCamera->ProcessKeyboard(-1.0f, 0.0f, 0.0f, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        currentCamera->ProcessKeyboard(0.0f, 0.0f, -1.0f, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        currentCamera->ProcessKeyboard(0.0f, 0.0f, 1.0f, deltaTime);
    
    // Decrease simplification ratio with minus key (increase detail)
    if(glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS || 
       glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) {
        if(!decreaseRatioKeyPressed) {
            simplificationRatio = std::max(0.05f, simplificationRatio - 0.05f);
            decreaseRatioKeyPressed = true;
            simplificationRatioChanged = true;
            std::cout << "Simplification ratio: " << simplificationRatio << std::endl;
        }
    } else {
        decreaseRatioKeyPressed = false;
    }
    
    // Increase simplification ratio with equal/plus key (reduce detail)
    if(glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS || 
       glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS) {
        if(!increaseRatioKeyPressed) {
            simplificationRatio = std::min(1.0f, simplificationRatio + 0.05f);
            increaseRatioKeyPressed = true;
            simplificationRatioChanged = true;
            std::cout << "Simplification ratio: " << simplificationRatio << std::endl;
        }
    } else {
        increaseRatioKeyPressed = false;
    }
    
    // Export model with E key
    if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        if(!exportModelKeyPressed) {
            exportModelKeyPressed = true;
            
            // Generate output filename
            std::string outputPath;
            size_t dotPos = modelPath.find_last_of('.');
            if (dotPos != std::string::npos) {
                outputPath = modelPath.substr(0, dotPos) + ".min" + modelPath.substr(dotPos);
            } else {
                outputPath = modelPath + ".min.obj";
            }
            
            std::cout << "Exporting simplified model to: " << outputPath << std::endl;
        }
    } else {
        exportModelKeyPressed = false;
    }
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // Reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    if (leftMousePressed) {
        currentCamera->ProcessMouseMovement(xoffset, yoffset);
    }
    else if (rightMousePressed) {
        currentCamera->ProcessMousePan(xoffset, yoffset);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS)
            leftMousePressed = true;
        else if (action == GLFW_RELEASE)
            leftMousePressed = false;
    }
    
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS)
            rightMousePressed = true;
        else if (action == GLFW_RELEASE)
            rightMousePressed = false;
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    currentCamera->ProcessMouseScroll(static_cast<float>(yoffset));
}

#endif // INPUT_H
