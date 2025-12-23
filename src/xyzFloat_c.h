#ifndef XYZ_FLOAT_C_H_
#define XYZ_FLOAT_C_H_

#include <math.h> // For sqrtf, fabsf if needed for operations

// 原本的 xyzFloat 結構保持不變
typedef struct {
    float x;
    float y;
    float z;
} xyzFloat_t;

// 建構函式替代品
void xyzFloat_init(xyzFloat_t *vec);
void xyzFloat_init_values(xyzFloat_t *vec, float x, float y, float z);

// 運算子重載的 C 語言替代函式
xyzFloat_t xyzFloat_unary_plus(const xyzFloat_t *vec);
xyzFloat_t xyzFloat_unary_minus(const xyzFloat_t *vec);
xyzFloat_t xyzFloat_add(const xyzFloat_t *v1, const xyzFloat_t *v2);
xyzFloat_t xyzFloat_subtract(const xyzFloat_t *v1, const xyzFloat_t *v2);
xyzFloat_t xyzFloat_multiply_scalar(const xyzFloat_t *vec, float scalar);
xyzFloat_t xyzFloat_divide_scalar(const xyzFloat_t *vec, float scalar);

void xyzFloat_add_assign(xyzFloat_t *v1, const xyzFloat_t *v2);
void xyzFloat_subtract_assign(xyzFloat_t *v1, const xyzFloat_t *v2);
void xyzFloat_multiply_assign_scalar(xyzFloat_t *vec, float scalar);
void xyzFloat_divide_assign_scalar(xyzFloat_t *vec, float scalar);

#endif // XYZ_FLOAT_C_H_