#ifndef _MISC_H_
#define _MISC_H_

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define     X               (0)
#define     Y               (1)
#define     Z               (2)
#define     CHN             (3)

#ifndef     PI
#define     PI              ((FLT)3.14159265358979323846)
#endif
#define     DEG2RAD         ((FLT)PI/(FLT)180.0)
#define     RAD2DEG         ((FLT)180.0/(FLT)PI)
#define     GRAVITY         ((FLT)9.80665)

    U32 computeMeanStd(FLT* const mean, FLT* const std, const FLT array[][CHN], U32 count);
    void dcm2euler(const FLT cbn[3][3], FLT* const pyaw, FLT* const ppitch, FLT* const proll);
    void euler2dcm(FLT cbn[3][3], FLT fyaw, FLT fpitch, FLT froll);
    void euler2q(FLT q[4] ,FLT fyaw, FLT fpitch, FLT froll);
    void q2dcm(FLT q[4], FLT cbn[3][3]);
    void qNorm(FLT q[4]);
    void f3x3matrixTranspose(FLT matrix[3][3]);
    void f3x3matrixEqI(FLT matrix[3][3]);
    void f3x3matrixEqScalar(FLT matrix[3][3], FLT scalar);

#ifdef __cplusplus
}      /* extern "C" */
#endif

#endif