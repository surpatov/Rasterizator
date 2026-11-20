
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "tgaimage.h"
#include "geometry.h"
#include "shader.h"
#include "camera.h"

void triangle(Vec3f* pts, IShader& shader, TGAImage& image, float* zbuffer, int width, int height) {
    Vec2f A(pts[0].x, pts[0].y);
    Vec2f B(pts[1].x, pts[1].y);
    Vec2f C(pts[2].x, pts[2].y);

    int minX = std::max(0, (int)std::floor(std::min({ A.x, B.x, C.x })));
    int maxX = std::min(width - 1, (int)std::ceil(std::max({ A.x, B.x, C.x })));
    int minY = std::max(0, (int)std::floor(std::min({ A.y, B.y, C.y })));
    int maxY = std::min(height - 1, (int)std::ceil(std::max({ A.y, B.y, C.y })));

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            Vec3f bary = barycentric(A, B, C, Vec2f((float)x, (float)y));

            if (bary.x < -0.0001f || bary.y < -0.0001f || bary.z < -0.0001f) continue;

            float z = bary.x * pts[0].z + bary.y * pts[1].z + bary.z * pts[2].z;
            int idx = x + y * width;

            if (zbuffer[idx] >= z) continue;

            TGAColor color;
            if (!shader.fragment(bary, color)) {
                zbuffer[idx] = z;
                image.set(x, y, color);
            }
        }
    }
}

int main() {
    const int width = 800;
    const int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    float* zbuffer = new float[width * height];
    for (int i = 0; i < width * height; i++) zbuffer[i] = -1e9f;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile("model.obj", 
        aiProcess_Triangulate | aiProcess_FlipUVs | 
        aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices
    );

    TGAImage texture;
    texture.read_tga_file("texture.tga");

    Camera camera(Vec3f(0, 0.7f, 1.2f), Vec3f(0, 0, 0), Vec3f(0, 1, 0), 70.0f, (float)width / height, 0.1f, 100.0f);

    Mat4 view = camera.getViewMatrix();
    Mat4 proj = camera.getProjectionMatrix();
    Mat4 model = Mat4::Scale(Vec3f(1.0f, 1.0f, 1.0f)) * Mat4::RotationY(90.0f);
    Mat4 viewport = Mat4::Viewport(width, height);

    Vec3f light(-1.3f, 0.9f, 0.6f);

    for (unsigned m = 0; m < scene->mNumMeshes; m++) {
        aiMesh* mesh = scene->mMeshes[m];
        PhongShader shader(mesh, light, model, view, proj, camera, &texture);

        for (unsigned i = 0; i < mesh->mNumFaces; i++) {
            if (mesh->mFaces[i].mNumIndices != 3) continue;

            aiVector3D v0 = mesh->mVertices[mesh->mFaces[i].mIndices[0]];
            aiVector3D v1 = mesh->mVertices[mesh->mFaces[i].mIndices[1]];
            aiVector3D v2 = mesh->mVertices[mesh->mFaces[i].mIndices[2]];

            Vec3f p0(v0.x, v0.y, v0.z);
            Vec3f p1(v1.x, v1.y, v1.z);
            Vec3f p2(v2.x, v2.y, v2.z);

            Vec4 wp0 = model * Vec4(p0.x, p0.y, p0.z, 1.0f);
            Vec4 wp1 = model * Vec4(p1.x, p1.y, p1.z, 1.0f);
            Vec4 wp2 = model * Vec4(p2.x, p2.y, p2.z, 1.0f);

            Vec3f world_p0(wp0.x, wp0.y, wp0.z);
            Vec3f world_p1(wp1.x, wp1.y, wp1.z);
            Vec3f world_p2(wp2.x, wp2.y, wp2.z);

            Vec3f edge1 = world_p1 - world_p0;
            Vec3f edge2 = world_p2 - world_p0;
            Vec3f normal = cross(edge1, edge2);
            normal.normalize();

            Vec3f center = (world_p0 + world_p1 + world_p2) * (1.0f / 3.0f);
            Vec3f view_dir = camera.position - center;
            view_dir.normalize();

            if (dot(normal, view_dir) < 0.0f) continue;

            Vec3f screen_coords[3];

            for (int j = 0; j < 3; j++) {
                Vec3f ndc = shader.vertex(i, j);
                Vec4 screen = viewport * Vec4(ndc.x, ndc.y, ndc.z, 1.0f);
                screen_coords[j] = Vec3f(screen.x, screen.y, screen.z);
            }

            triangle(screen_coords, shader, image, zbuffer, width, height);
        }
    }

    image.write_tga_file("output.tga");
    delete[] zbuffer;
    return 0;
}
