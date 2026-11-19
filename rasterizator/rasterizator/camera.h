#pragma once

#include "geometry.h"

class Camera {
public:
    Vec3f position;
    Vec3f target;
    Vec3f up;

    float fov;
    float nearPlane;
    float farPlane;
    float aspect;

    Camera(Vec3f pos, Vec3f tgt, Vec3f upDir, float f, float aspectRatio, float n, float f_dist)
        : position(pos), target(tgt), up(upDir), fov(f), aspect(aspectRatio), nearPlane(n), farPlane(f_dist) {
    }

    Mat4 getViewMatrix() const {
        return Mat4::LookAt(position, target, up);
    }

    Mat4 getProjectionMatrix() const {
        return Mat4::Perspective(fov, aspect, nearPlane, farPlane);
    }
};