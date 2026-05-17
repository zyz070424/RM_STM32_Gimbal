/**
 ******************************************************************************
 * @file    alg_kalman_filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   卡尔曼滤波基础层实现
 ******************************************************************************
 */

#include "alg_kalman_filter.h"

uint16_t sizeof_float;
uint16_t sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

static void *Kalman_Filter_Alloc_Zero(size_t size)
{
    void *ptr;

    if (size == 0u)
    {
        return NULL;
    }

    ptr = user_malloc(size);
    if (ptr != NULL)
    {
        memset(ptr, 0, size);
    }

    return ptr;
}

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    kf->MeasurementMap = (uint8_t *)Kalman_Filter_Alloc_Zero(sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize);
    kf->StateMinVariance = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize);
    kf->temp = (uint8_t *)Kalman_Filter_Alloc_Zero(sizeof(uint8_t) * zSize);

    kf->FilteredValue = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize);
    kf->ControlVector = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * uSize);

    kf->xhat_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    kf->xhatminus_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0u)
    {
        kf->u_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }
    else
    {
        kf->u_data = NULL;
    }

    kf->z_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    kf->P_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    kf->Pminus_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    kf->F_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0u)
    {
        kf->B_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }
    else
    {
        kf->B_data = NULL;
    }

    kf->H_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    kf->Q_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    kf->R_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    kf->K_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)Kalman_Filter_Alloc_Zero(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0u;
    kf->SkipEq2 = 0u;
    kf->SkipEq3 = 0u;
    kf->SkipEq4 = 0u;
    kf->SkipEq5 = 0u;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    if (kf->UseAutoAdjustment != 0u)
    {
        H_K_R_Adjustment(kf);
    }
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    if (kf->uSize != 0u)
    {
        memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
    }
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (kf->SkipEq1 != 0u)
    {
        return;
    }

    if (kf->uSize > 0u)
    {
        kf->temp_vector.numRows = kf->xhatSize;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
        kf->temp_vector1.numRows = kf->xhatSize;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
        kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
    }
    else
    {
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (kf->SkipEq2 != 0u)
    {
        return;
    }

    kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
    kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->FT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
}

void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (kf->SkipEq3 != 0u)
    {
        return;
    }

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
}

void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (kf->SkipEq4 != 0u)
    {
        return;
    }

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);
    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (kf->SkipEq5 != 0u)
    {
        return;
    }

    kf->temp_matrix.numRows = kf->K.numRows;
    kf->temp_matrix.numCols = kf->H.numCols;
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1);
    kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
}

float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    uint8_t i;

    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
    {
        kf->User_Func0_f(kf);
    }

    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
    {
        kf->User_Func1_f(kf);
    }

    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
    {
        kf->User_Func2_f(kf);
    }

    if ((kf->MeasurementValidNum != 0u) || (kf->UseAutoAdjustment == 0u))
    {
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
        {
            kf->User_Func3_f(kf);
        }

        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
        {
            kf->User_Func4_f(kf);
        }

        Kalman_Filter_P_Update(kf);
    }
    else
    {
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL)
    {
        kf->User_Func5_f(kf);
    }

    for (i = 0u; i < kf->xhatSize; i++)
    {
        if (kf->P_data[(i * kf->xhatSize) + i] < kf->StateMinVariance[i])
        {
            kf->P_data[(i * kf->xhatSize) + i] = kf->StateMinVariance[i];
        }
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
    {
        kf->User_Func6_f(kf);
    }

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    uint8_t i;

    kf->MeasurementValidNum = 0u;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (i = 0u; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0.0f)
        {
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            kf->H_data[(kf->xhatSize * kf->MeasurementValidNum) + kf->MeasurementMap[i] - 1u] =
                kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }

    for (i = 0u; i < kf->MeasurementValidNum; i++)
    {
        kf->R_data[(i * kf->MeasurementValidNum) + i] =
            kf->MatR_DiagonalElements[kf->temp[i]];
    }

    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
