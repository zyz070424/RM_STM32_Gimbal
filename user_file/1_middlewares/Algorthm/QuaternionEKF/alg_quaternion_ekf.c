/**
 ******************************************************************************
 * @file    alg_quaternion_ekf.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   四元数 EKF 姿态解算实现
 ******************************************************************************
 */
#include "alg_quaternion_ekf.h"

#include <string.h>

/*
 * 并行 EKF 的默认参数。
 * 这组值偏保守，优先减弱云台机械振动和运动加速度对姿态结果的干扰，
 * 适合作为第一轮联调基线。
 */
#define IMU_QUATERNION_EKF_DEFAULT_Q1              (10.0f)
#define IMU_QUATERNION_EKF_DEFAULT_Q2              (0.0015f)
#define IMU_QUATERNION_EKF_DEFAULT_R               (5000000.0f)
#define IMU_QUATERNION_EKF_DEFAULT_LAMBDA          (1.0f)
#define IMU_QUATERNION_EKF_DEFAULT_ACCEL_LPF_COEF  (0.0085f)

QEKF_INS_t QEKF_INS;

static const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};
static const float IMU_QuaternionEKF_P_Init[36] = {
    100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 100, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100
};
static float IMU_QuaternionEKF_K[18];
static float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_InitDefault(void);
static void IMU_QuaternionEKF_ResetState(void);

void IMU_QuaternionEKF_Init(float process_noise1,
                            float process_noise2,
                            float measure_noise,
                            float lambda,
                            float lpf)
{
    QEKF_INS.Initialized = 1u;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8f;
    if (lambda > 1.0f)
    {
        lambda = 1.0f;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6u, 0u, 3u);
    Matrix_Init(&QEKF_INS.ChiSquare, 1u, 1u, (float *)QEKF_INS.ChiSquare_Data);

    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    IMU_QuaternionEKF_ResetState();
}

void IMU_QuaternionEKF_Reset(void)
{
    if (QEKF_INS.Initialized == 0u)
    {
        IMU_QuaternionEKF_InitDefault();
        return;
    }

    IMU_QuaternionEKF_ResetState();
}

void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float halfgxdt;
    static float halfgydt;
    static float halfgzdt;
    static float accelInvNorm;
    uint8_t i;

    if (QEKF_INS.Initialized == 0u)
    {
        IMU_QuaternionEKF_InitDefault();
    }

    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data,
           IMU_QuaternionEKF_F,
           sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    if (QEKF_INS.UpdateCount == 0u)
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }

    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) +
                        az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
                           QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
                           QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (i = 0u; i < 3u; i++)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;
    }

    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                        QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    if ((QEKF_INS.gyro_norm < 0.3f) &&
        (QEKF_INS.accl_norm > 9.8f - 0.5f) &&
        (QEKF_INS.accl_norm < 9.8f + 0.5f))
    {
        QEKF_INS.StableFlag = 1u;
    }
    else
    {
        QEKF_INS.StableFlag = 0u;
    }

    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0.0f;

    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
                          2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) *
                    57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
                            2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) *
                      57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) *
                     57.295779513f;

    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++;
}

static void IMU_QuaternionEKF_InitDefault(void)
{
    IMU_QuaternionEKF_Init(IMU_QUATERNION_EKF_DEFAULT_Q1,
                           IMU_QUATERNION_EKF_DEFAULT_Q2,
                           IMU_QUATERNION_EKF_DEFAULT_R,
                           IMU_QUATERNION_EKF_DEFAULT_LAMBDA,
                           IMU_QUATERNION_EKF_DEFAULT_ACCEL_LPF_COEF);
}

static void IMU_QuaternionEKF_ResetState(void)
{
    size_t cov_size;
    size_t state_size;
    size_t meas_size;
    KalmanFilter_t *kf;

    kf = &QEKF_INS.IMU_QuaternionEKF;
    state_size = sizeof_float * QEKF_INS.IMU_QuaternionEKF.xhatSize;
    cov_size = sizeof_float * QEKF_INS.IMU_QuaternionEKF.xhatSize * QEKF_INS.IMU_QuaternionEKF.xhatSize;
    meas_size = sizeof_float * QEKF_INS.IMU_QuaternionEKF.zSize;

    QEKF_INS.ConvergeFlag = 0u;
    QEKF_INS.StableFlag = 0u;
    QEKF_INS.ErrorCount = 0u;
    QEKF_INS.UpdateCount = 0u;
    QEKF_INS.AdaptiveGainScale = 1.0f;
    QEKF_INS.gyro_norm = 0.0f;
    QEKF_INS.accl_norm = 0.0f;
    QEKF_INS.Roll = 0.0f;
    QEKF_INS.Pitch = 0.0f;
    QEKF_INS.Yaw = 0.0f;
    QEKF_INS.YawTotalAngle = 0.0f;
    QEKF_INS.dt = 0.0f;
    QEKF_INS.YawRoundCount = 0;
    QEKF_INS.YawAngleLast = 0.0f;
    memset(QEKF_INS.q, 0, sizeof(QEKF_INS.q));
    QEKF_INS.q[0] = 1.0f;
    memset(QEKF_INS.GyroBias, 0, sizeof(QEKF_INS.GyroBias));
    memset(QEKF_INS.Gyro, 0, sizeof(QEKF_INS.Gyro));
    memset(QEKF_INS.Accel, 0, sizeof(QEKF_INS.Accel));
    memset(QEKF_INS.OrientationCosine, 0, sizeof(QEKF_INS.OrientationCosine));
    memset(QEKF_INS.ChiSquare_Data, 0, sizeof(QEKF_INS.ChiSquare_Data));
    memset(IMU_QuaternionEKF_K, 0, sizeof(IMU_QuaternionEKF_K));
    memset(IMU_QuaternionEKF_H, 0, sizeof(IMU_QuaternionEKF_H));

    memset(kf->FilteredValue, 0, state_size);
    memset(kf->MeasuredVector, 0, meas_size);
    memset(kf->xhat_data, 0, state_size);
    memset(kf->xhatminus_data, 0, state_size);
    memset(kf->P_data, 0, cov_size);
    memset(kf->Pminus_data, 0, cov_size);
    memset(kf->F_data, 0, cov_size);
    memset(kf->FT_data, 0, cov_size);
    memset(kf->Q_data, 0, cov_size);
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);
    memset(kf->HT_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->K_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    memset(kf->S_data, 0, cov_size);
    memset(kf->temp_matrix_data, 0, cov_size);
    memset(kf->temp_matrix_data1, 0, cov_size);
    memset(kf->temp_vector_data, 0, state_size);
    memset(kf->temp_vector_data1, 0, state_size);

    if ((kf->uSize != 0u) && (kf->ControlVector != NULL))
    {
        memset(kf->ControlVector, 0, sizeof_float * kf->uSize);
    }
    if ((kf->uSize != 0u) && (kf->u_data != NULL))
    {
        memset(kf->u_data, 0, sizeof_float * kf->uSize);
    }

    memcpy(kf->F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(kf->P_data, IMU_QuaternionEKF_P_Init, sizeof(IMU_QuaternionEKF_P_Init));
    memcpy(kf->Pminus_data, IMU_QuaternionEKF_P_Init, sizeof(IMU_QuaternionEKF_P_Init));
    kf->xhat_data[0] = 1.0f;
    kf->xhatminus_data[0] = 1.0f;
    kf->FilteredValue[0] = 1.0f;
}

static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;
    static float qInvNorm;
    uint8_t i;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (i = 0u; i < 4u; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }

    kf->F_data[4] = q1 * QEKF_INS.dt / 2.0f;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2.0f;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2.0f;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2.0f;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2.0f;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2.0f;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2.0f;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2.0f;

    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    if (kf->P_data[28] > 10000.0f)
    {
        kf->P_data[28] = 10000.0f;
    }
    if (kf->P_data[35] > 10000.0f)
    {
        kf->P_data[35] = 10000.0f;
    }
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0;
    static float doubleq1;
    static float doubleq2;
    static float doubleq3;

    doubleq0 = 2.0f * kf->xhatminus_data[0];
    doubleq1 = 2.0f * kf->xhatminus_data[1];
    doubleq2 = 2.0f * kf->xhatminus_data[2];
    doubleq3 = 2.0f * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;
    uint8_t i;
    uint8_t j;

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

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1u;
    kf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (i = 0u; i < 3u; i++)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1u;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1u;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);
    kf->temp_vector.numRows = 1u;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);

    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1u;
    }

    if ((QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold) &&
        (QEKF_INS.ConvergeFlag != 0u))
    {
        if (QEKF_INS.StableFlag != 0u)
        {
            QEKF_INS.ErrorCount++;
        }
        else
        {
            QEKF_INS.ErrorCount = 0u;
        }

        if (QEKF_INS.ErrorCount > 50u)
        {
            QEKF_INS.ConvergeFlag = 0u;
            kf->SkipEq5 = FALSE;
        }
        else
        {
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            return;
        }
    }
    else
    {
        if ((QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold) &&
            (QEKF_INS.ConvergeFlag != 0u))
        {
            QEKF_INS.AdaptiveGainScale =
                (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) /
                (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1.0f;
        }
        QEKF_INS.ErrorCount = 0u;
        kf->SkipEq5 = FALSE;
    }

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    for (i = 0u; i < (uint8_t)(kf->K.numRows * kf->K.numCols); i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (i = 4u; i < 6u; i++)
    {
        for (j = 0u; j < 3u; j++)
        {
            kf->K_data[(i * 3u) + j] *= QEKF_INS.OrientationCosine[i - 4u] / 1.5707963f;
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1u;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

    if (QEKF_INS.ConvergeFlag != 0u)
    {
        for (i = 4u; i < 6u; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    kf->temp_vector.pData[3] = 0.0f;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

static float invSqrt(float x)
{
    float halfx;
    float y;
    long i;

    halfx = 0.5f * x;
    y = x;
    i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
