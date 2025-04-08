#ifndef CAMERA_H
#define CAMERA_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.2f;
const float ZOOM = 45.0f;
const float PAN_SPEED = 0.001f;
const float ORBIT_DISTANCE = 3.0f;
const glm::vec3 WORLD_UP = glm::vec3(0.0f, 1.0f, 0.0f);

class Camera
{
public:
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    glm::vec3 Target;

    float Yaw;
    float Pitch;

    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    float Distance;

    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f),
           float yaw = YAW, float pitch = PITCH) : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
                                                   MovementSpeed(SPEED),
                                                   MouseSensitivity(SENSITIVITY),
                                                   Zoom(ZOOM),
                                                   WorldUp(WORLD_UP),
                                                   Target(target)
    {
        Position = position;
        Yaw = yaw;
        Pitch = pitch;
        Distance = glm::distance(Position, Target);
        updateCameraVectors();
    }

    glm::mat4 GetViewMatrix() const
    {
        return glm::lookAt(Position, Target, Up);
    }

    void ProcessKeyboard(float deltaX, float deltaY, float deltaZ, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        glm::vec3 movement = deltaX * Right + deltaY * Up - deltaZ * Front;
        Target += movement * velocity;
        Position += movement * velocity;
    }

    void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw += xoffset;
        Pitch += yoffset;

        if (constrainPitch)
        {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        updateCameraVectors();
    }

    void ProcessMouseScroll(float yoffset)
    {
        Distance -= yoffset * 0.2f;
        if (Distance < 0.1f)
            Distance = 0.1f;

        updateCameraVectors();
    }

    void ProcessMousePan(float xoffset, float yoffset)
    {
        float panSpeed = PAN_SPEED * Distance;
        glm::vec3 movement = -xoffset * Right - yoffset * Up;
        Target += movement * panSpeed;
        Position += movement * panSpeed;
    }

    void updateCameraVectors()
    {
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Pitch));
        front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = glm::normalize(front);

        Right = glm::normalize(glm::cross(Front, WorldUp));
        Up = glm::normalize(glm::cross(Right, Front));

        Position = Target - Front * Distance;
    }
};
#endif
