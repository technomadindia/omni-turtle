class Drv8833 {
private:
    int motor1_pin1_;
    int motor1_pin2_;
    int motor2_pin1_;
    int motor2_pin2_;
    int sleep_pin_;
    int fault_pin_;

public:
    void init_motor(int motor_id, int pin1, int pin2);
    void init_control(int sleep_pin, int fault_pin);
    void set_motor(int motor_id, int direction, int power);
    void sleep();
    void wake();
};
