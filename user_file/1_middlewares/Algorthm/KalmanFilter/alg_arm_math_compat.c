#include "alg_arm_math_compat.h"

#include <math.h>
#include <string.h>

static uint8_t ArmMatCheckSameSize(const arm_matrix_instance_f32 *pSrcA,
                                   const arm_matrix_instance_f32 *pSrcB,
                                   const arm_matrix_instance_f32 *pDst)
{
    if ((pSrcA == 0) || (pSrcB == 0) || (pDst == 0))
    {
        return 0u;
    }

    if ((pSrcA->numRows != pSrcB->numRows) ||
        (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows) ||
        (pSrcA->numCols != pDst->numCols))
    {
        return 0u;
    }

    return 1u;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    uint32_t index;
    uint32_t total;

    if (ArmMatCheckSameSize(pSrcA, pSrcB, pDst) == 0u)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    total = (uint32_t)pSrcA->numRows * (uint32_t)pSrcA->numCols;
    for (index = 0u; index < total; index++)
    {
        pDst->pData[index] = pSrcA->pData[index] + pSrcB->pData[index];
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    uint32_t index;
    uint32_t total;

    if (ArmMatCheckSameSize(pSrcA, pSrcB, pDst) == 0u)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    total = (uint32_t)pSrcA->numRows * (uint32_t)pSrcA->numCols;
    for (index = 0u; index < total; index++)
    {
        pDst->pData[index] = pSrcA->pData[index] - pSrcB->pData[index];
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    uint16_t row;
    uint16_t col;
    uint16_t inner;

    if ((pSrcA == 0) || (pSrcB == 0) || (pDst == 0))
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    if ((pSrcA->numCols != pSrcB->numRows) ||
        (pDst->numRows != pSrcA->numRows) ||
        (pDst->numCols != pSrcB->numCols))
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (row = 0u; row < pSrcA->numRows; row++)
    {
        for (col = 0u; col < pSrcB->numCols; col++)
        {
            float32_t sum = 0.0f;

            for (inner = 0u; inner < pSrcA->numCols; inner++)
            {
                sum += pSrcA->pData[(row * pSrcA->numCols) + inner] *
                       pSrcB->pData[(inner * pSrcB->numCols) + col];
            }

            pDst->pData[(row * pDst->numCols) + col] = sum;
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    uint16_t row;
    uint16_t col;

    if ((pSrc == 0) || (pDst == 0))
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    if ((pDst->numRows != pSrc->numCols) || (pDst->numCols != pSrc->numRows))
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (row = 0u; row < pSrc->numRows; row++)
    {
        for (col = 0u; col < pSrc->numCols; col++)
        {
            pDst->pData[(col * pDst->numCols) + row] =
                pSrc->pData[(row * pSrc->numCols) + col];
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
    uint16_t row;
    uint16_t col;
    uint16_t pivot_row;
    uint16_t size;
    float32_t pivot_abs;
    float32_t factor;

    if ((pSrc == 0) || (pDst == 0))
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    if ((pSrc->numRows == 0u) ||
        (pSrc->numRows != pSrc->numCols) ||
        (pDst->numRows != pDst->numCols) ||
        (pDst->numRows != pSrc->numRows))
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    size = pSrc->numRows;

    {
        float32_t work[size * size];
        float32_t inverse[size * size];

        memcpy(work, pSrc->pData, sizeof(float32_t) * size * size);
        memset(inverse, 0, sizeof(float32_t) * size * size);

        for (row = 0u; row < size; row++)
        {
            inverse[(row * size) + row] = 1.0f;
        }

        for (col = 0u; col < size; col++)
        {
            pivot_row = col;
            pivot_abs = fabsf(work[(col * size) + col]);

            for (row = (uint16_t)(col + 1u); row < size; row++)
            {
                float32_t candidate_abs = fabsf(work[(row * size) + col]);

                if (candidate_abs > pivot_abs)
                {
                    pivot_abs = candidate_abs;
                    pivot_row = row;
                }
            }

            if (pivot_abs == 0.0f)
            {
                return ARM_MATH_SINGULAR;
            }

            if (pivot_row != col)
            {
                for (row = 0u; row < size; row++)
                {
                    float32_t swap_value = work[(col * size) + row];
                    work[(col * size) + row] = work[(pivot_row * size) + row];
                    work[(pivot_row * size) + row] = swap_value;

                    swap_value = inverse[(col * size) + row];
                    inverse[(col * size) + row] = inverse[(pivot_row * size) + row];
                    inverse[(pivot_row * size) + row] = swap_value;
                }
            }

            factor = work[(col * size) + col];
            for (row = 0u; row < size; row++)
            {
                work[(col * size) + row] /= factor;
                inverse[(col * size) + row] /= factor;
            }

            for (row = 0u; row < size; row++)
            {
                uint16_t inner_col;

                if (row == col)
                {
                    continue;
                }

                factor = work[(row * size) + col];
                if (factor == 0.0f)
                {
                    continue;
                }

                for (inner_col = 0u; inner_col < size; inner_col++)
                {
                    work[(row * size) + inner_col] -= factor * work[(col * size) + inner_col];
                    inverse[(row * size) + inner_col] -= factor * inverse[(col * size) + inner_col];
                }
            }
        }

        memcpy(pDst->pData, inverse, sizeof(float32_t) * size * size);
    }

    return ARM_MATH_SUCCESS;
}
