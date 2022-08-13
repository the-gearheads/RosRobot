#pragma once

#include "AHRS.h"
#include "ctre/Phoenix.h"
#include "DrivetrainConstants.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/simulation/DifferentialDrivetrainSim.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/SPI.h"
#include "hal/SimDevice.h"
#include "hal/simulation/SimDeviceData.h"
namespace frc {
class DriveSubsystem {
public:
  DriveSubsystem();
  void drive(double xSpeed, double rot);
  ChassisSpeeds getSpeeds();
  void zeroEncoders();
  void periodic();

  DifferentialDriveOdometry odometry{ Rotation2d{}, Pose2d{} };

private:
  void setSpeeds(DifferentialDriveWheelSpeeds speeds);

  units::meter_t getLPos();
  units::meter_t getRPos();

  units::meters_per_second_t getLVel();
  units::meters_per_second_t getRVel();

  int metersToNative(units::meter_t position);
  int velocityToNative(units::meters_per_second_t velocity);

  units::meter_t nativeToMeters(double nativeUnits);
  units::meters_per_second_t nativeVelocityToMeters(double velocity);

  void simPeriodic();

  WPI_TalonFX lfMotor{ LF_MOTOR_ID };
  WPI_TalonFX lbMotor{ LB_MOTOR_ID };

  WPI_TalonFX rfMotor{ RF_MOTOR_ID };
  WPI_TalonFX rbMotor{ RB_MOTOR_ID };

  AHRS gyro{ SPI::Port::kMXP };
  std::shared_ptr<Field2d> field = std::make_shared<Field2d>();

  DifferentialDriveKinematics kinematics{ TRACK_WIDTH };
  SimpleMotorFeedforward<units::meters> lFeedforward{L_FF_KS, L_FF_KV};
  SimpleMotorFeedforward<units::meters> rFeedforward{R_FF_KS, R_FF_KV};

  sim::DifferentialDrivetrainSim sim {
    LinearSystemId::IdentifyDrivetrainSystem(SIM_LINEAR_KV, SIM_LINEAR_KA, SIM_ANGULAR_KV, SIM_ANGULAR_KA),
    TRACK_WIDTH, DCMotor::Falcon500(2), SHAFT_TO_WHEEL_GEAR_RATIO, WHEEL_RADIUS,
    {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}
  };


  TalonFXSimCollection lSim = lfMotor.GetSimCollection();
  TalonFXSimCollection rSim = rfMotor.GetSimCollection();
  
  /* Why is the sensor id 4???? */
  hal::SimDouble simAngle = HALSIM_GetSimValueHandle(HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw");
};
}  // namespace frc