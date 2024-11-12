class PIDController {
private:
    float kp_;
    float ki_;
    float kd_;
    float dead_zone_;
    float prev_error_;
    float error_integral_;

public:
    PIDController();
    ~PIDController();

    void tune(float kp, float ki, float kd, float dead_zone = 0.0);
    float update(float desired, float measured, float delta_time);
    void reset();
};
