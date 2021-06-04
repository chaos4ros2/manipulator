// 参考：
// https://github.com/rt-net/crane_plus/blob/use_new_ros2_control/crane_plus_control/include/crane_plus_control/crane_plus_driver.hpp

// ドライバーパッケージ
// https://github.com/kerry-t-johnson/i2c_pwm

// 疑問：この２行の意味は不明（外したらどうなる？完成後テストする）
#ifndef MANIPULATOR_CONTROL__MANIPULATOR_DRIVER_HPP_
#define MANIPULATOR_CONTROL__MANIPULATOR_DRIVER_HPP_

// ドライバーパッケージを読み込む
#include <i2c_pwm/Pca9685.cpp>
// 以下三つの固有モジュールの使い方を要調査
#include <memory>
#include <string>
#include <vector>

class Manipulator {
    public:
        // https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685.cpp
        // 必要な引数は1.deviceFile / 2.address / 3.autoInitialize
        ManipulatorDriver(const std::string &deviceFile,
                          const int address,
                          bool autoInitialize);
        // Pca9685.cppで提供されてるshared_ptrのため、デストラクターは不要のはず
        // ~ManipulatorDriver();

    // autoInitializeをtrueにすることでインスタンス化時に必ず初期化を保証するため不要
    // bool open_port(void);
    void close_port(void);

    // https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L146
    // https://github.com/chaos4ros2/servo/blob/main/servo/output_servo.py#L48
    void set_feq(uint16_t value);
    // last_error_log_を使わないため一旦コメントアウト
    // std::string get_last_error_log(void);

    // 疑問：bool型で宣言する理由（データの代入は処理過程中で済ませたからだ joint_positions）
    bool write_goal_joint_positions(const std::vector<double> & goal_positions);
    bool read_present_joint_positions(std::vector<double> * joint_positions);

    private:
        std::shared_ptr<Pca9685> pca9685_handler_;
        std::vector<uint8_t> id_list_;
        // std::string last_error_log_;
        // write_goal_joint_positions()用関数群
        // sg90のパルス指定には角度が使用されるため、角度を軸にする変換関数を用意する 6/3
        uint16_t radian_to_angle(const double position);
        uint16_t angle_to_pluse(const uint16_t angle);
        // uint16_tもいらない気がする、自作ではFloat32を使ってる
        // https://github.com/chaos4ros2/servo/blob/main/servo/controller.py#L13
        // 引数型参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/src/crane_plus_driver.cpp#L214
        uint16_t radian_to_pluse(const double position);

        // read_present_joint_positions()用関数群
        uint16_t pluse_to_angle(const uint16_t pluse);
        // radianをdoubleにしなくても大丈夫かな
        uint16_t angle_to_radian(const uint16_t angle);
        uint16_t pluse_to_radian(const uint16_t pluse);
}
