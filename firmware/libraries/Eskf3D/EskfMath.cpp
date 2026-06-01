#include "EskfMath.h"
#include <math.h>

namespace EskfMath {

    void matZero(float* A, int r, int c) {
        for (int i = 0; i < r*c; i++) A[i] = 0.0f;
    }

    void matEye(float* A, int n, float diag) {
        matZero(A, n, n);
        for (int i = 0; i < n; i++) A[i*n + i] = diag;
    }

    void matCopy(const float* A, float* B, int r, int c) {
        for (int i = 0; i < r*c; i++) B[i] = A[i];
    }

    void matMul(const float* A, int Ar, int Ac,
                const float* B, int Br, int Bc,
                float* C)
    {
        (void)Br;
        matZero(C, Ar, Bc);
        for (int i = 0; i < Ar; i++) {
            for (int k = 0; k < Ac; k++) {
                float aik = A[i*Ac + k];
                for (int j = 0; j < Bc; j++) {
                    C[i*Bc + j] += aik * B[k*Bc + j];
                }
            }
        }
    }

    void matMulBT(const float* A, int Ar, int Ac,
                  const float* B, int Br, int Bc,
                  float* C)
    {
        matZero(C, Ar, Br);
        for (int i = 0; i < Ar; i++) {
            for (int j = 0; j < Br; j++) {
                float s = 0.0f;
                for (int k = 0; k < Ac; k++) {
                    s += A[i*Ac + k] * B[j*Bc + k];
                }
                C[i*Br + j] = s;
            }
        }
    }

    void matMulAT(const float* A, int Ar, int Ac,
                  const float* B, int Br, int Bc,
                  float* C)
    {
        (void)Br;
        matZero(C, Ac, Bc);
        for (int i = 0; i < Ac; i++) {
            for (int k = 0; k < Ar; k++) {
                float aki = A[k*Ac + i];
                for (int j = 0; j < Bc; j++) {
                    C[i*Bc + j] += aki * B[k*Bc + j];
                }
            }
        }
    }

    void matAdd(const float* A, const float* B, float* C, int r, int c) {
        for (int i = 0; i < r*c; i++) C[i] = A[i] + B[i];
    }

    void matSub(const float* A, const float* B, float* C, int r, int c) {
        for (int i = 0; i < r*c; i++) C[i] = A[i] - B[i];
    }

    void matAddInPlace(float* A, const float* B, int r, int c, float alpha) {
        for (int i = 0; i < r*c; i++) A[i] += alpha * B[i];
    }

    void matTranspose(const float* A, int r, int c, float* AT) {
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                AT[j*r + i] = A[i*c + j];
            }
        }
    }

    // ---------- inv3 ----------
    bool inv3(const float A[9], float Ainv[9]) {
        float a00=A[0], a01=A[1], a02=A[2];
        float a10=A[3], a11=A[4], a12=A[5];
        float a20=A[6], a21=A[7], a22=A[8];

        float c00 =  a11*a22 - a12*a21;
        float c01 = -(a10*a22 - a12*a20);
        float c02 =  a10*a21 - a11*a20;

        float det = a00*c00 + a01*c01 + a02*c02;
        if (fabsf(det) < 1e-12f) return false;
        float invdet = 1.0f / det;

        Ainv[0] = c00 * invdet;
        Ainv[1] = (-(a01*a22 - a02*a21)) * invdet;
        Ainv[2] = ( a01*a12 - a02*a11) * invdet;

        Ainv[3] = c01 * invdet;
        Ainv[4] = ( a00*a22 - a02*a20) * invdet;
        Ainv[5] = (-(a00*a12 - a02*a10)) * invdet;

        Ainv[6] = c02 * invdet;
        Ainv[7] = (-(a00*a21 - a01*a20)) * invdet;
        Ainv[8] = ( a00*a11 - a01*a10) * invdet;

        return true;
    }

    // ---------- inv6 (Gauss-Jordan) ----------
    bool inv6(const float A[36], float Ainv[36]) {
        // Augment [A | I] into a 6x12 buffer
        float aug[6*12];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) aug[i*12 + j] = A[i*6 + j];
            for (int j = 0; j < 6; j++) aug[i*12 + (6+j)] = (i==j)?1.0f:0.0f;
        }

        // Pivot elimination
        for (int col = 0; col < 6; col++) {
            // Find pivot
            int piv = col;
            float maxv = fabsf(aug[col*12 + col]);
            for (int r = col+1; r < 6; r++) {
                float v = fabsf(aug[r*12 + col]);
                if (v > maxv) { maxv = v; piv = r; }
            }
            if (maxv < 1e-12f) return false;

            // Swap rows
            if (piv != col) {
                for (int j = 0; j < 12; j++) {
                    float tmp = aug[col*12 + j];
                    aug[col*12 + j] = aug[piv*12 + j];
                    aug[piv*12 + j] = tmp;
                }
            }

            // Normalize pivot row
            float diag = aug[col*12 + col];
            float invd = 1.0f / diag;
            for (int j = 0; j < 12; j++) aug[col*12 + j] *= invd;

            // Eliminate other rows
            for (int r = 0; r < 6; r++) {
                if (r == col) continue;
                float f = aug[r*12 + col];
                if (f == 0.0f) continue;
                for (int j = 0; j < 12; j++) {
                    aug[r*12 + j] -= f * aug[col*12 + j];
                }
            }
        }

        // Extract inverse
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                Ainv[i*6 + j] = aug[i*12 + (6+j)];
            }
        }
        return true;
    }

}
