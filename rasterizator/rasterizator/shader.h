#pragma once

#include <algorithm>
#include <cmath>

#include "geometry.h"
#include <assimp/scene.h>

struct IShader {
    virtual ~IShader() {}
    virtual Vec3f vertex(int iface, int ivert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor& color) = 0;
};

struct GouraudShader : public IShader {
    aiMesh* mesh;
    Vec3f light_dir;

    Vec3f varying_intensity;

    Mat4 model;
    Mat4 view;
    Mat4 projection;

    TGAImage* texture;
    Vec2f varying_uv[3];
    float varying_reciprocal_w[3];

    GouraudShader(aiMesh* m, Vec3f light, Mat4 mod, Mat4 v, Mat4 proj, TGAImage* tex = nullptr)
        : mesh(m), light_dir(light), model(mod), view(v), projection(proj), texture(tex) {
        light_dir.normalize();
    }

    Vec3f vertex(int iface, int ivert) override {
        unsigned int idx = mesh->mFaces[iface].mIndices[ivert];
        aiVector3D v = mesh->mVertices[idx];

        Vec3f normal(0, 0, 1);
        if (mesh->HasNormals()) {
            aiVector3D n = mesh->mNormals[idx];
            normal = Vec3f(n.x, n.y, n.z);
            normal.normalize();
        }

        float diff = std::max(0.f, dot(normal, light_dir));
        float intensity = diff * 0.9f + 0.1f;
        intensity = std::min(1.f, std::max(0.f, intensity));

        if (ivert == 0) varying_intensity.x = intensity;
        if (ivert == 1) varying_intensity.y = intensity;
        if (ivert == 2) varying_intensity.z = intensity;

        if (mesh->HasTextureCoords(0)) {
            aiVector3D uv = mesh->mTextureCoords[0][idx];
            varying_uv[ivert] = Vec2f(uv.x, uv.y);
        }
        else varying_uv[ivert] = Vec2f(0, 0);

        Vec4 gl_Vertex(v.x, v.y, v.z, 1.0f);
        Vec4 clip = projection * view * model * gl_Vertex;

        varying_reciprocal_w[ivert] = 1.0f / clip.w;

        return Vec3f(
            clip.x / clip.w,
            clip.y / clip.w,
            -clip.z / clip.w
        );
    }

    bool fragment(Vec3f bar, TGAColor& color) override {
        float intensity = varying_intensity.x * bar.x +
            varying_intensity.y * bar.y +
            varying_intensity.z * bar.z;
        intensity = std::min(1.f, std::max(0.f, intensity));

        float w0 = varying_reciprocal_w[0];
        float w1 = varying_reciprocal_w[1];
        float w2 = varying_reciprocal_w[2];

        float denominator =
            bar.x * w0 +
            bar.y * w1 +
            bar.z * w2;

        Vec2f uv = (varying_uv[0] * (bar.x * w0) +
                varying_uv[1] * (bar.y * w1) +
                varying_uv[2] * (bar.z * w2)) / denominator;

        if (texture != nullptr) {
            int tex_x = (int)(uv.x * texture->width());
            int tex_y = (int)((1 - uv.y) * texture->height());

            TGAColor tex_color = texture->get(tex_x, tex_y);

            color = TGAColor{ {
                (unsigned char)(tex_color[0] * intensity),
                (unsigned char)(tex_color[1] * intensity),
                (unsigned char)(tex_color[2] * intensity),
                255
            } };
            return false;
        }

        return true;
    }
};