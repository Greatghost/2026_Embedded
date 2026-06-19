float GimbalPitchComp()
{
    // Poly4拟合, 上下行平均消摩擦, RMSE=22.0, R²=0.9708
    const static float p1 = 0.007644f;  // x^4
    const static float p2 = 0.320182f;  // x^3
    const static float p3 = 4.230975f;  // x^2
    const static float p4 = 30.792684f;  // x
    const static float p5 = 198.620846f;  // const

    float x = gimbal_controller.gyro_pitch_angle;
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float comp_current = p1 * x4 + p2 * x3 + p3 * x2 + p4 * x + p5;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}