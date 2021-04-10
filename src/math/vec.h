#pragma once
#include <math.h>
#include <string.h>

namespace tracer {
template<typename T>
struct __attribute__((aligned(16))) vec4 {

    union {
        struct {
            T x, y, z, w;
        };
        struct {
            T r, g, b, a;
        };
        struct {
            T data[4];
        };
    };

    vec4(const T& value = 0.f): x(value), y(value), z(value), w(value) {}
    vec4(const T& x, const T& y, const T& z, const T& w): x(x), y(y), z(z), w(w) {}
    vec4(const vec4<T>& other) { memcpy(data, other.data, sizeof(T) * 4); }
    // vec4(vec4<T> &&other) { std::swap(data, other.data); }

    T& operator[](const unsigned& idx) { return data[idx]; }
    T operator[](const unsigned& idx) const { return data[idx]; }
};

template<typename T>
struct __attribute__((aligned(16))) vec3 {

    union {
        struct {
            T x, y, z;
        };
        struct {
            T r, g, b;
        };
        struct {
            T data[3];
        };
    };

    vec3(const T& value = 0.f): x(value), y(value), z(value) {}
    vec3(const T& x, const T& y, const T& z): x(x), y(y), z(z) {}
    vec3(const vec3<T>& other) { memcpy(data, other.data, sizeof(T) * 3); }
    // vec3(vec3<T> &&other) { std::swap(data, other.data); }

    T& operator[](const unsigned& idx) { return data[idx]; }
    T operator[](const unsigned& idx) const { return data[idx]; }
};

template<typename T>
struct __attribute__((aligned(16))) vec2 {

    union {
        struct {
            T x, y;
        };
        struct {
            T r, g;
        };
        struct {
            T data[2];
        };
    };

    vec2(const T& value = 0.f): x(value), y(value) {}
    vec2(const T& x, const T& y): x(x), y(y) {}
    vec2(const vec2<T>& other) { memcpy(data, other.data, sizeof(T) * 2); }
    // vec2(vec2<T> &&other) { std::swap(data, other.data); }

    T& operator[](const unsigned& idx) const { return data[idx]; }
};

template<typename T>
vec4<T> operator/(const vec4<T>& v, const T& scalar) {
    vec4<T> r;
    for (int i = 0; i < 4; i++) {
        r.data[i] = v.data[i] / scalar;
    }
    return r;
}

template<typename T>
T dot(const vec4<T>& v0, const vec4<T>& v1) {
    T sum2 = 0;
    for (int i = 0; i < 4; i++) {
        sum2 += (v0.data[i] * v1.data[i]);
    }
    return sum2;
}

template<typename T>
vec4<T> normalize(const vec4<T>& v) {
    return v / dot(v, v);
}

template<typename T>
T dot(const vec3<T>& v0, const vec3<T>& v1) {
    T sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += (v0.data[i] * v1.data[i]);
    }
    return sum;
}

template<typename T>
vec3<T> cross(const vec3<T>& v1, const vec3<T>& v2) {
    vec3<T> dest;
    dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
    dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
    dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
    return dest;
}

template<typename T>
vec3<T> operator+(const vec3<T>& v1, const vec3<T>& v2) {
    return vec3<T>(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]);
}

template<typename T>
vec3<T> operator-(const vec3<T>& v1, const vec3<T>& v2) {
    return vec3<T>(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
}

template<typename T>
vec3<T> operator/(const vec3<T>& v, const T& scalar) {
    vec3<T> r;
    for (int i = 0; i < 3; i++) {
        r.data[i] = v.data[i] / scalar;
    }
    return r;
}

template<typename T>
vec3<T> operator*(const vec3<T>& v, const T& scalar) {
    vec3<T> r;
    for (int i = 0; i < 3; i++) {
        r.data[i] = v.data[i] * scalar;
    }
    return r;
}

template<typename T>
vec3<T> normalize(const vec3<T>& v) {
    return v / sqrt(dot(v, v));
}

template<typename T>
T length(const vec3<T>& v) {
    return sqrt(dot(v, v));
}

} // namespace tracer
