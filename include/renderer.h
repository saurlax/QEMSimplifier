#ifndef RENDERER_H
#define RENDERER_H

#include <GL/glew.h>
#include <GL/glu.h>
#include <glm/glm.hpp>
#include <iostream>
#include "model.h"
#include "camera.h"

// Window dimensions
extern const unsigned int SCR_WIDTH;
extern const unsigned int SCR_HEIGHT;

// Setup the renderer
void setupRenderer();

// Render a frame
void renderFrame(const Model& model, const Camera& camera);

// Framebuffer size callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

// Implementation

void setupRenderer() {
    // Configure global OpenGL state
    glEnable(GL_DEPTH_TEST);
    
    // Check for OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cout << "OpenGL error during setup: " << err << std::endl;
    }
}

void renderFrame(const Model& model, const Camera& camera) {
    // Dark gray background
    const GLfloat bgColor[] = {0.2f, 0.2f, 0.2f, 1.0f};
    
    // Clear buffers with dark gray background
    glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set fixed pipeline matrices
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Apply camera transformations
    glm::mat4 viewMatrix = camera.GetViewMatrix();
    glMultMatrixf(&viewMatrix[0][0]);
    
    // Setup lighting
    GLfloat lightPos[] = { 0.0f, 5.0f, 5.0f, 1.0f };
    GLfloat lightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat lightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat lightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    
    // Set material properties
    GLfloat matAmbient[] = { 0.3f, 0.4f, 0.5f, 1.0f };
    GLfloat matDiffuse[] = { 0.4f, 0.5f, 0.6f, 1.0f };
    GLfloat matSpecular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat matShininess[] = { 32.0f };
    
    glMaterialfv(GL_FRONT, GL_AMBIENT, matAmbient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, matDiffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, matSpecular);
    glMaterialfv(GL_FRONT, GL_SHININESS, matShininess);
    
    glColor3f(0.4f, 0.5f, 0.6f);
    
    // Render filled model first
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPushMatrix();
    model.Draw();
    glPopMatrix();
    
    // Render wireframe on top
    glDisable(GL_LIGHTING);
    glDepthFunc(GL_LEQUAL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(1.0f);
    glColor3f(0.0f, 0.0f, 0.0f);
    
    glPushMatrix();
    model.Draw();
    glPopMatrix();
    
    // Reset states
    glDepthFunc(GL_LESS);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);

    // Check for errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cout << "OpenGL error during rendering: " << err << std::endl;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

#endif // RENDERER_H
