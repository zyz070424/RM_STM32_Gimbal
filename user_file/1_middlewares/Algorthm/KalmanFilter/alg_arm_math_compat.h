#ifndef __ALG_ARM_MATH_COMPAT_H__
#define __ALG_ARM_MATH_COMPAT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef enum
{
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_NANINF = -4,
    ARM_MATH_SINGULAR = -5,
    ARM_MATH_TEST_FAILURE = -6
} arm_status;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData;
} arm_matrix_instance_f32;

static inline void arm_mat_init_f32(arm_matrix_instance_f32 *S,
                                    uint16_t nRows,
                                    uint16_t nColumns,
                                    float32_t *pData)
{
    if (S == 0)
    {
        return;
    }

    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData = pData;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_ARM_MATH_COMPAT_H__ */
