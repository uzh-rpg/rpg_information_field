#include "unrealcv_bridge/ue_utils.hpp"

#include <random>

#include <rpg_common/test_main.h>

using namespace unrealcv_bridge;

TEST(PoseConversionTest, RotationConversion)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> deg_dis(10.0, 40.0);

  for (size_t i = 0; i < 10; i++)
  {
    const double ryaw = deg_dis(gen);
    const double rpitch = deg_dis(gen);
    const double rroll = deg_dis(gen);

    rpg::Rotation rot;
    esim::unrealEulerToQuaternion(rpitch, ryaw, rroll, &rot);
    double yaw, pitch, roll;
    esim::quaternionToEulerUnrealEngine(rot, yaw, pitch, roll);

    EXPECT_DOUBLE_EQ(ryaw, yaw);
    EXPECT_DOUBLE_EQ(rpitch, pitch);
    EXPECT_DOUBLE_EQ(rroll, roll);
  }

  //  std::cout << "yaw: " << ryaw << " vs " << yaw << std::endl;
  //  std::cout << "pitch: " << rpitch << " vs " << pitch << std::endl;
  //  std::cout << "roll: " << rroll << " vs " << roll << std::endl;
}

TEST(PoseConversionTest, PoseConversion)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> rot_dis(10.0, 40.0);
  std::uniform_real_distribution<> pos_dis(10.0, 40.0);

  for (size_t i = 0; i < 10; i++)
  {
    const double ryaw = rot_dis(gen);
    const double rpitch = rot_dis(gen);
    const double rroll = rot_dis(gen);
    const double rx = pos_dis(gen);
    const double ry = pos_dis(gen);
    const double rz = pos_dis(gen);

    UEPose uep {rpitch, ryaw, rroll, rx, ry, rz};
    rpg::Pose Twc;
    uep.toTwc(&Twc);

    UEPose uep_r;
    TwcToUEPose(Twc, &uep_r);

    EXPECT_TRUE(uep.isSame(uep_r));
  }
}

RPG_COMMON_TEST_MAIN
{
}
