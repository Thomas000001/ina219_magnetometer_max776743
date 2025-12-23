#include "xyzFloat_c.h"

void xyzFloat_init(xyzFloat_t *vec) {
    if (vec) {
        vec->x = 0.0f;
        vec->y = 0.0f;
        vec->z = 0.0f;
    }
}

void xyzFloat_init_values(xyzFloat_t *vec, float x, float y, float z) {
    if (vec) {
        vec->x = x;
        vec->y = y;
        vec->z = z;
    }
}

xyzFloat_t xyzFloat_unary_plus(const xyzFloat_t *vec) {
    return *vec;
}

xyzFloat_t xyzFloat_unary_minus(const xyzFloat_t *vec) {
    xyzFloat_t result;
    result.x = -vec->x;
    result.y = -vec->y;
    result.z = -vec->z;
    return result;
}

xyzFloat_t xyzFloat_add(const xyzFloat_t *v1, const xyzFloat_t *v2) {
    xyzFloat_t result;
    result.x = v1->x + v2->x;
    result.y = v1->y + v2->y;
    result.z = v1->z + v2->z;
    return result;
}

xyzFloat_t xyzFloat_subtract(const xyzFloat_t *v1, const xyzFloat_t *v2) {
    xyzFloat_t result;
    result.x = v1->x - v2->x;
    result.y = v1->y - v2->y;
    result.z = v1->z - v2->z;
    return result;
}

xyzFloat_t xyzFloat_multiply_scalar(const xyzFloat_t *vec, float scalar) {
    xyzFloat_t result;
    result.x = vec->x * scalar;
    result.y = vec->y * scalar;
    result.z = vec->z * scalar;
    return result;
}

xyzFloat_t xyzFloat_divide_scalar(const xyzFloat_t *vec, float scalar) {
    xyzFloat_t result;
    if (scalar != 0.0f) {
        result.x = vec->x / scalar;
        result.y = vec->y / scalar;
        result.z = vec->z / scalar;
    } else {
        // Handle division by zero, e.g., return zero vector or NaNs
        xyzFloat_init(&result);
    }
    return result;
}

void xyzFloat_add_assign(xyzFloat_t *v1, const xyzFloat_t *v2) {
    v1->x += v2->x;
    v1->y += v2->y;
    v1->z += v2->z;
}

void xyzFloat_subtract_assign(xyzFloat_t *v1, const xyzFloat_t *v2) {
    v1->x -= v2->x;
    v1->y -= v2->y;
    v1->z -= v2->z;
}

void xyzFloat_multiply_assign_scalar(xyzFloat_t *vec, float scalar) {
    vec->x *= scalar;
    vec->y *= scalar;
    vec->z *= scalar;
}

void xyzFloat_divide_assign_scalar(xyzFloat_t *vec, float scalar) {
    if (scalar != 0.0f) {
        vec->x /= scalar;
        vec->y /= scalar;
        vec->z /= scalar;
    } else {
        // Handle division by zero if necessary, e.g. set to 0 or NaN
        vec->x = 0.0f; // Or NAN
        vec->y = 0.0f;
        vec->z = 0.0f;
    }
}