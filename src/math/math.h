#ifndef MATH_H
#define MATH_H

#include <cmath>

struct Vec3 {
    float X, Y, Z;

    Vec3() : X(0), Y(0), Z(0) {}
    Vec3(float X, float Y, float Z) : X(X), Y(Y), Z(Z) {}

    Vec3 operator+(const Vec3& Other) const { return Vec3(X + Other.X, Y + Other.Y, Z + Other.Z); }
    Vec3 operator-(const Vec3& Other) const { return Vec3(X - Other.X, Y - Other.Y, Z - Other.Z); }
    Vec3 operator*(float Scalar) const { return Vec3(X * Scalar, Y * Scalar, Z * Scalar); }

    float Length() const { return std::sqrt(X * X + Y * Y + Z * Z); }

    Vec3 Normalize() const {
        float Len = Length();
        return (Len > 1e-6f) ? Vec3(X / Len, Y / Len, Z / Len) : Vec3();
    }
};

inline float Min(float A, float B) { return (A < B) ? A : B; }
inline float Dot(const Vec3& A, const Vec3& B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }

#endif // MATH_H
