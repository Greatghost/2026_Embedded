float GimbalPitchComp()
{
    // 多项式系数 (Poly4拟合 - 2026/06/19, DM电机标定, RMSE=57.1, R²=0.8907)
    const static float p1 = -0.020176f;
    const static float p2 = -0.449467f;
    const static float p3 = -1.461691f;
    const static float p4 = 24.290357f;
    const static float p5 = 514.928641f;

    float x = gimbal_controller.gyro_pitch_angle;

    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float comp_current = p1 * x4 + p2 * x3 + p3 * x2 + p4 * x + p5;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}