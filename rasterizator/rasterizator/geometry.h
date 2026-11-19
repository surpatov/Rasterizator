#pragma once
#include <cmath>
#include <algorithm>
#include "tgaimage.h"

struct IShader;

template<typename T>
struct Vec2 {
    T x, y;
    Vec2() : x(0), y(0) {}
    Vec2(T _x, T _y) : x(_x), y(_y) {}

	Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
	Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
	Vec2 operator*(const float& scalar) const { return Vec2(x * scalar, y * scalar); }
	Vec2 operator/(const float& scalar) const { return Vec2(x / scalar, y / scalar); }

    float length() const { return std::sqrt(x * x + y * y); }
};

template <typename T>
struct Vec3 {
    T x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(T x, T y, T z) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator*(T s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(T s) const { return Vec3(x / s, y / s, z / s); }
    T& operator[](int i) { if (i == 0) return x; if (i == 1) return y; return z; }
    void normalize() {
        T len = std::sqrt(x * x + y * y + z * z);
        if (len > 0) { x /= len; y /= len; z /= len; }
    }
	float length() const { return std::sqrt(x * x + y * y + z * z); }
};

typedef Vec2<int> Vec2i;
typedef Vec2<float> Vec2f;
typedef Vec3<int> Vec3i;
typedef Vec3<float> Vec3f;

inline float dot(const Vec3f& a, const Vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return Vec3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

float triangleArea(Vec2f A, Vec2f B, Vec2f C)
{
    Vec3f AB(B.x - A.x, B.y - A.y, 0.0f);
    Vec3f AC(C.x - A.x, C.y - A.y, 0.0f);

    return cross(AB, AC).z * 0.5f;
}
Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    float totalArea = triangleArea(A, B, C);

    if (std::abs(totalArea) < 0.0001f)
        return Vec3f(-1, 1, 1);

    float areaA = triangleArea(P, B, C);
    float areaB = triangleArea(A, P, C);
    float areaC = triangleArea(A, B, P);

    float b0 = areaA / totalArea;
    float b1 = areaB / totalArea;
    float b2 = areaC / totalArea;

    return Vec3f(b0, b1, b2);
}

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(1) {}
    Vec4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
    Vec4 operator*(float s) const { return Vec4(x * s, y * s, z * s, w * s); }
	Vec4 operator+(const Vec4& o) const { return Vec4(x + o.x, y + o.y, z + o.z, w + o.w); }
};

struct Mat4 {
    float m[4][4] = { 0 };

    Mat4() { for (int i = 0; i < 4; i++) m[i][i] = 1; }

    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        r.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w;
        r.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w;
        r.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w;
        r.w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w;
        return r;
    }

    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j] + m[i][3] * o.m[3][j];
        return r;
	}

    static Mat4 Perspective(float fov, float aspect, float near, float far) {
        Mat4 res;
        float tanHalfFov = std::tan(fov / 2.0f * 3.14159265f / 180.0f);

        res.m[0][0] = 1.0f / (aspect * tanHalfFov);
        res.m[1][1] = 1.0f / tanHalfFov;
        res.m[2][2] = -(far + near) / (far - near);
        res.m[2][3] = -(2.0f * far * near) / (far - near);
        res.m[3][2] = -1.0f;
        res.m[3][3] = 0.0f;
        return res;
    }

    static Mat4 Viewport(int width, int height) {
        Mat4 res;
        res.m[0][0] = width / 2.0f;
        res.m[1][1] = height / 2.0f;
        res.m[2][2] = 1.0f;
        res.m[0][3] = width / 2.0f;
        res.m[1][3] = height / 2.0f;
        return res;
    }

    static Mat4 LookAt(Vec3f eye, Vec3f center, Vec3f up) {
        Vec3f f = center - eye; f.normalize();
        Vec3f s = cross(f, up); s.normalize();
        Vec3f u = cross(s, f);
        Mat4 res;
        res.m[0][0] = s.x; res.m[0][1] = s.y; res.m[0][2] = s.z; res.m[0][3] = -dot(s, eye);
        res.m[1][0] = u.x; res.m[1][1] = u.y; res.m[1][2] = u.z; res.m[1][3] = -dot(u, eye);
        res.m[2][0] = -f.x; res.m[2][1] = -f.y; res.m[2][2] = -f.z; res.m[2][3] = dot(f, eye);
        return res;
    }

    static Mat4 Scale(Vec3f s) {
        Mat4 res;
        res.m[0][0] = s.x;
        res.m[1][1] = s.y;
        res.m[2][2] = s.z;
        return res;
    }
};

void triangle(Vec3f* pts, IShader& shader, TGAImage& image, float* zbuffer, int width, int height);