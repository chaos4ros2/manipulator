// 参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/src/crane_plus_driver.cpp
#include <algorithm>
#include <cmath> // M_PI
#include <memory>
#include <string>
#include <vector>

#include "manipulator_control/manipulator_driver.hpp"

// https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685.cpp#L28
ManipulatorDriver::ManipulatorDriver(
    const std::string &deviceFile,
    const int address,
    const int frequency_hz,
    std::vector<uint8_t> id_list)
  : frequency_hz_(frequency_hz), id_list_(id_list) 
{
    // 必ず初期化する（分ける必要があれば初期化用メンバー関数を追加する）
    bool autoInitialize = true;
    // 青木さんと同じくshared_ptrを使う
    // shared_ptrは「https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685.cpp」で提供
    // shared_ptrをまだ理解できてない
    // shared_ptrのため、このファイルではデストラクターを実装する必要はなさそう。
    pca9685_handler_ = i2c_pwm::Pca9685::create(
        deviceFile,
        address,
        autoInitialize);
    // https://github.com/chaos4ros2/servo/blob/main/servo/output_servo.py#L48
    // どの値を設定すべきかは要検討
    set_feq(frequency_hz);
}

// https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L131
// 引数を受け付けない関数は、引数リストにおいて void を明示的に宣言する必要がある
void ManipulatorDriver::close_port(void)
{
    pca9685_handler_->allStop();
}

// https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L146
// https://github.com/chaos4ros2/servo/blob/main/servo/output_servo.py#L48
void ManipulatorDriver::set_feq(uint16_t value)
{
    pca9685_handler_->setFrequencyHz(value);
}

uint16_t ManipulatorDriver::radian_to_angle(const double position)
{
    // http://www.fumiononaka.com/Drafts/ActionScript30Reference/01_01_Math_PI.html
    return position * (180 / M_PI); 
}

uint16_t ManipulatorDriver::angle_to_pluse(const uint16_t angle)
{
    // https://github.com/chaos4ros2/servo/blob/main/servo/output_servo.py#L95
    return (600 - 150) / 180 * (angle + 90) + 150;
}

uint16_t ManipulatorDriver::radian_to_pluse(const double position)
{
    return angle_to_pluse(radian_to_angle(position));   
} 

// 参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/src/crane_plus_driver.cpp#L97
bool ManipulatorDriver::write_goal_joint_positions(const std::vector<double> & goal_positions)
{
    for (size_t i = 0; i < goal_positions.size(); i++) {
        uint16_t pluse = radian_to_pluse(goal_positions[i]);
        auto servo_id = id_list_[i];
        pca9685_handler_->writeChannel(servo_id, pluse);
    }

    return true;
}


uint16_t ManipulatorDriver::pluse_to_angle(const uint16_t pluse)
{
    return 180 / (600 - 150) * (pluse - 150) - 90;       
}

uint16_t ManipulatorDriver::angle_to_radian(const uint16_t angle)
{
    return angle * (M_PI / 180);      
}

uint16_t ManipulatorDriver::pluse_to_radian(const uint16_t pluse)
{
    return angle_to_radian(pluse_to_angle(pluse));      
}

// https://docs.microsoft.com/ja-jp/cpp/cpp/namespaces-cpp?view=msvc-160#anonymous-or-unnamed-namespaces
// https://github.com/chaos4ros2/manipulator/blob/master/i2c_pwm/src/Pca9685Impl.cpp#L46
// 元のパッケージ内では匿名namespaceとなってるため、外部では参照できないため一時的に定義する。
// Todo:いずれ元の定義先をグローバルnamespaceに変える
namespace {
    const uint8_t REGISTER_CHANNEL0_OFF_HIGH = 0x09;
    const uint8_t UPPER_BYTE_SHIFT = 8;
}
// 参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/src/crane_plus_driver.cpp#L168
bool ManipulatorDriver::read_present_joint_positions(std::vector<double> * joint_positions)
{
    for (auto servo_id : id_list_) {
          // サーボのidで取得すべきアドレスを計算する
          const int channel_offset = servo_id * 4;
          // https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L281
          auto register_address = REGISTER_CHANNEL0_OFF_HIGH + channel_offset; 
          // https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L196
          auto off_high_value = pca9685_handler_->read(register_address);
          // https://github.com/kerry-t-johnson/i2c_pwm/blob/master/src/Pca9685Impl.cpp#L282
          auto pluse = off_high_value << UPPER_BYTE_SHIFT; 
          joint_positions->push_back(pluse_to_radian(pluse));
    }

    return true; 
}