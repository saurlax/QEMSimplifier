#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include "camera.h"
#include "model.h"
#include "input.h"
#include "renderer.h"
#include "qem.h"

// Window dimensions
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// Camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
Camera* currentCamera = &camera;

// Mouse control variables
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
bool leftMousePressed = false;
bool rightMousePressed = false;

// Simplification control
float simplificationRatio = 1.0f;  // Start with original model
bool simplificationRatioChanged = false;
bool decreaseRatioKeyPressed = false;
bool increaseRatioKeyPressed = false;
bool exportModelKeyPressed = false;

// Timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main(int argc, char* argv[])
{
    // Check command line arguments
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_obj_file>" << std::endl;
        return -1;
    }
    
    std::string modelPath = argv[1];
    
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    
    // Set error callback for GLFW
    glfwSetErrorCallback(error_callback);
    
    // Set OpenGL version and compatibility profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "QEM Model Simplification", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }
    
    // Setup renderer
    setupRenderer();

    // Set initial camera position and orientation
    camera.Position = glm::vec3(0.0f, 0.0f, 3.0f);
    camera.Pitch = 0.0f;  
    camera.Yaw = -90.0f;  
    camera.Target = glm::vec3(0.0f, 0.0f, 0.0f);
    camera.updateCameraVectors();

    // Load 3D model
    Model originalModel(modelPath);
    
    // Create QEM simplifier
    QEM qem(originalModel);
    
    // Current model to display (start with original)
    Model currentModel = originalModel;
    
    // Print control info
    std::cout << "Model loaded with " << originalModel.meshes[0].vertices.size() << " vertices and " 
              << originalModel.meshes[0].indices.size()/3 << " faces.\n\n";
    std::cout << "Controls:\n";
    std::cout << "- Mouse left button: Rotate camera around model\n";
    std::cout << "- Mouse right button: Pan camera\n";
    std::cout << "- Mouse wheel: Zoom in/out\n";
    std::cout << "- WASD keys: Move camera\n";
    std::cout << "- Minus key (-): Reduce model detail\n";
    std::cout << "- Plus key (+): Increase model detail\n";
    std::cout << "- E key: Export simplified model\n";
    std::cout << "- ESC key: Exit application\n";
    
    // Rendering loop
    while (!glfwWindowShouldClose(window)) {
        // Per-frame time logic
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Process input
        processInput(window, modelPath);
        
        // Check if simplification ratio changed
        if (simplificationRatioChanged && simplificationRatio < 1.0f) {
            std::cout << "Simplifying model to " << simplificationRatio * 100 << "% of original faces..." << std::endl;
            currentModel = qem.simplify(simplificationRatio);
            simplificationRatioChanged = false;
        } else if (simplificationRatioChanged && simplificationRatio >= 1.0f) {
            currentModel = originalModel;
            simplificationRatioChanged = false;
            std::cout << "Showing original model" << std::endl;
        }
        
        // Export model if requested
        if (exportModelKeyPressed) {
            std::string outputPath;
            size_t dotPos = modelPath.find_last_of('.');
            if (dotPos != std::string::npos) {
                outputPath = modelPath.substr(0, dotPos) + ".min" + modelPath.substr(dotPos);
            } else {
                outputPath = modelPath + ".min.obj";
            }
            
            if (qem.exportToObj(outputPath, simplificationRatio)) {
                std::cout << "Model exported successfully to " << outputPath << std::endl;
            } else {
                std::cerr << "Failed to export model to " << outputPath << std::endl;
            }
            exportModelKeyPressed = false;
        }

        // Render the current model
        renderFrame(currentModel, camera);

        // Swap buffers and poll IO events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Terminate GLFW
    glfwTerminate();
    return 0;
}
