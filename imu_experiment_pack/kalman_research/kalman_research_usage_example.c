/*
 * Kalman 研究版打包示例。
 *
 * 原始研究代码目前还没有公共头文件，
 * 所以这里为了临时实验，直接 include 源文件。
 * 这种方式只建议用于快速试验，不建议用于正式工程。
 */

#include "alg_quaternion_paper_research.c"

static quat_paper_research_observe_t g_research_observe;

void KalmanResearch_ExampleInit(void)
{
    quat_paper_research_observe_reset(&g_research_observe, 0.001f);
}

void KalmanResearch_ExampleStep(void)
{
    quat_paper_research_imu_t imu = {0};

    imu.gyro.x = 0.0f;
    imu.gyro.y = 0.0f;
    imu.gyro.z = 10.0f;
    imu.acc.x = 0.0f;
    imu.acc.y = 0.0f;
    imu.acc.z = 1.0f;
    imu.temp = 25.0f;
    imu.dt = 0.001f;

    quat_paper_research_observe_update(&g_research_observe, imu, 0.5f, 0.001f);

    /*
     * 更新后可重点观察：
     * - g_research_observe.paper_euler
     * - g_research_observe.paper_debug
     * - g_research_observe.paper_state
     */
}
