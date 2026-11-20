#pragma once

#include <algorithm>
#include <cmath>

#include "geometry.h"
#include "camera.h"
#include <assimp/scene.h>

struct IShader {
    virtual ~IShader() {}

    virtual Vec3f vertex(int iface, int ivert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor& color) = 0;
};

struct PhongShader : public IShader {
    aiMesh* mesh;

    Vec3f light_dir;

    Mat4 model;
    Mat4 view;
    Mat4 projection;

    TGAImage* texture;
    Camera& camera;

    Vec2f varying_uv[3];
    Vec3f varying_normal[3];
    Vec3f varying_position[3];

    float varying_reciprocal_w[3];

    PhongShader(aiMesh* m, Vec3f light, Mat4 mod, Mat4 v, Mat4 proj, Camera& cam, TGAImage* tex = nullptr)
        : mesh(m), light_dir(light), model(mod), view(v), projection(proj), texture(tex), camera(cam) {
        light_dir.normalize();
    }

    Vec3f vertex(int iface, int ivert) override {
        unsigned int idx = mesh->mFaces[iface].mIndices[ivert];

        aiVector3D v = mesh->mVertices[idx];

        Vec4 gl_Vertex(v.x, v.y, v.z, 1.0f);
        Vec4 world_vertex = model * gl_Vertex;
        Vec3f position(world_vertex.x, world_vertex.y, world_vertex.z);

        varying_position[ivert] = position;

        Vec3f normal(0.0f, 0.0f, 1.0f);

        if (mesh->HasNormals()) {
            aiVector3D n = mesh->mNormals[idx];

            normal = Vec3f(n.x, n.y, n.z);
            normal.normalize();
        }

        Vec4 world_normal = model * Vec4(normal.x, normal.y, normal.z, 0.0f);

        Vec3f N(world_normal.x, world_normal.y, world_normal.z);
        N.normalize();

        varying_normal[ivert] = N;

        if (mesh->HasTextureCoords(0)) {
            aiVector3D uv = mesh->mTextureCoords[0][idx];

            varying_uv[ivert] = Vec2f(uv.x, uv.y);
        }
        else varying_uv[ivert] = Vec2f(0.0f, 0.0f);

        Vec4 clip = projection * view * world_vertex;

        varying_reciprocal_w[ivert] = 1.0f / clip.w;

        return Vec3f(
            clip.x / clip.w,
            clip.y / clip.w,
            -clip.z / clip.w
        );
    }

    bool fragment(Vec3f bar, TGAColor& color) override {
        float w0 = varying_reciprocal_w[0];
        float w1 = varying_reciprocal_w[1];
        float w2 = varying_reciprocal_w[2];

        float denominator = bar.x * w0 + bar.y * w1 + bar.z * w2;

        if (std::abs(denominator) < 1e-8f)
            return true;

        float b0 = bar.x * w0 / denominator;
        float b1 = bar.y * w1 / denominator;
        float b2 = bar.z * w2 / denominator;

        Vec3f P = varying_position[0] * b0 + varying_position[1] * b1 + varying_position[2] * b2;
        Vec3f N = varying_normal[0] * b0 + varying_normal[1] * b1 + varying_normal[2] * b2;
        N.normalize();

        Vec2f uv = varying_uv[0] * b0 + varying_uv[1] * b1 + varying_uv[2] * b2;

        Vec3f V = camera.position - P;
        V.normalize();

        Vec3f L = light_dir;
        L.normalize();

        float ndotl = std::max(0.0f, dot(N, L));

        float ambient = 0.3f;
        float diffuse = 0.65f * ndotl;
        float specular = 0.0f;

        if (ndotl > 0.0f) {

            Vec3f R = N * (2.0f * ndotl) - L;
            R.normalize();

            float spec_angle = std::max(0.0f, dot(R, V));
            specular = 0.4f * std::pow(spec_angle, 32.0f);
        }

        float tex_r = 1.0f;
        float tex_g = 1.0f;
        float tex_b = 1.0f;

        if (texture != nullptr) {

            int tex_x = std::clamp((int)(uv.x * texture->width()), 0, texture->width() - 1);
            int tex_y = std::clamp((int)((1.0f - uv.y) * texture->height()), 0, texture->height() - 1);

            TGAColor tex_color = texture->get(tex_x, tex_y);

            tex_r = tex_color[0] / 255.0f;
            tex_g = tex_color[1] / 255.0f;
            tex_b = tex_color[2] / 255.0f;
        }

        float final_r = tex_r * (ambient + diffuse) + specular;
        float final_g = tex_g * (ambient + diffuse) + specular;
        float final_b = tex_b * (ambient + diffuse) + specular;

        final_r = std::clamp(final_r, 0.0f, 1.0f);
        final_g = std::clamp(final_g, 0.0f, 1.0f);
        final_b = std::clamp(final_b, 0.0f, 1.0f);

        color = TGAColor{ {
            (unsigned char)(final_r * 255.0f),
            (unsigned char)(final_g * 255.0f),
            (unsigned char)(final_b * 255.0f),
            255
        } };

        return false;
    }
};