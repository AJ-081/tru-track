#ifndef ESKF_MATH_H
#define ESKF_MATH_H

#include <stdint.h>

// Fixed-size math for 15-state ESKF
// No STL, no dynamic memory, float only.

namespace EskfMath {

    // ---------- Basic helpers ----------
    void matZero(float* A, int r, int c);
    void matEye(float* A, int n, float diag = 1.0f);
    void matCopy(const float* A, float* B, int r, int c);

    // C = A * B
    void matMul(const float* A, int Ar, int Ac,
                const float* B, int Br, int Bc,
                float* C);

    // C = A * B^T
    void matMulBT(const float* A, int Ar, int Ac,
                  const float* B, int Br, int Bc,
                  float* C);

    // C = A^T * B
    void matMulAT(const float* A, int Ar, int Ac,
                  const float* B, int Br, int Bc,
                  float* C);

    // C = A + B
    void matAdd(const float* A, const float* B, float* C, int r, int c);

    // C = A - B
    void matSub(const float* A, const float* B, float* C, int r, int c);

    // A = A + alpha*B
    void matAddInPlace(float* A, const float* B, int r, int c, float alpha = 1.0f);

    // C = A^T
    void matTranspose(const float* A, int r, int c, float* AT);

    // Invert 3x3 and 6x6 (we need for GNSS updates)
    bool inv3(const float A[9], float Ainv[9]);   // row-major 3x3
    bool inv6(const float A[36], float Ainv[36]); // row-major 6x6

}

#endif
