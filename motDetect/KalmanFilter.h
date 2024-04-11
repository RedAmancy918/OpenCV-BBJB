class KalmanFilter {
public:
    KalmanFilter(double process_noise = 0.001, double sensor_noise = 0.03, double estimated_error = 0.1, double initial_value = 0) {
        q = process_noise;     // 过程噪声协方差
        r = sensor_noise;      // 测量噪声协方差
        p = estimated_error;   // 估计错误
        x = initial_value;     // 估计值
    }

    double updateEstimate(double mea) {
        p += q;
        k = p / (p + r);
        x += k * (mea - x);
        p *= (1 - k);
        return x;
    }

    void setMeasurementError(double me) {
        r = me;
    }

    void setEstimateError(double ee) {
        p = ee;
    }

    void setProcessNoise(double pn) {
        q = pn;
    }

    void setInitialValue(double iv) {
        x = iv;
    }

private:
    double q; // 过程噪声协方差
    double r; // 测量噪声协方差
    double p; // 估计错误
    double x; // 值
    double k; // 卡尔曼增益
};
