#include "DriveSubsystem.h"
#include "Robot.h"
#include <stdio.h>
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace frc;

DriveSubsystem::DriveSubsystem() {
  lfMotor.ConfigFactoryDefault();
  lbMotor.ConfigFactoryDefault();
  rfMotor.ConfigFactoryDefault();
  rbMotor.ConfigFactoryDefault();

  rbMotor.Follow(this->rfMotor);
  lbMotor.Follow(this->lfMotor);

	rfMotor.SetInverted(TalonFXInvertType::Clockwise);
	rbMotor.SetInverted(TalonFXInvertType::FollowMaster);
	lfMotor.SetInverted(TalonFXInvertType::CounterClockwise);
	lbMotor.SetInverted(TalonFXInvertType::FollowMaster);

  gyro.Reset();

  SmartDashboard::PutData("Field", field.get());
}

void DriveSubsystem::setSpeeds(DifferentialDriveWheelSpeeds speeds) {
  units::volt_t lVolts = lFeedforward.Calculate(speeds.left);
  units::volt_t rVolts = rFeedforward.Calculate(speeds.right);
  printf("%f, %f :%f. %f\n", speeds.left.value(), speeds.right.value(), lVolts.value(), rVolts.value());
  lfMotor.SetVoltage(lVolts);
  rfMotor.SetVoltage(rVolts);
}

void DriveSubsystem::drive(double xSpeed, double rot) {
  setSpeeds(kinematics.ToWheelSpeeds(ChassisSpeeds{units::meters_per_second_t{xSpeed},
                                     0_mps, units::radians_per_second_t{rot}}));
}

void DriveSubsystem::zeroEncoders() {
  gyro.ZeroYaw();

  lfMotor.SetSelectedSensorPosition(0);
  lbMotor.SetSelectedSensorPosition(0);
  rfMotor.SetSelectedSensorPosition(0);
  rbMotor.SetSelectedSensorPosition(0);
  odometry.ResetPosition(Pose2d(), this->gyro.GetRotation2d());
}

void DriveSubsystem::periodic() {
  if(Robot::IsSimulation())
    simPeriodic();
  auto lPos = nativeToMeters(lfMotor.GetSelectedSensorPosition());
  auto rPos = nativeToMeters(rfMotor.GetSelectedSensorPosition());
  SmartDashboard::PutNumber("Left Encoder", lPos.value());
  SmartDashboard::PutNumber("Right Encoder", rPos.value());
  auto pose = odometry.Update(gyro.GetRotation2d(), lPos, rPos);
  field->SetRobotPose(pose);
}

void DriveSubsystem::simPeriodic() {
  lSim.SetBusVoltage(RobotController::GetInputVoltage());
  rSim.SetBusVoltage(RobotController::GetInputVoltage());

  sim.SetInputs(units::volt_t{ lSim.GetMotorOutputLeadVoltage() }, 
                units::volt_t{ -rSim.GetMotorOutputLeadVoltage() });

  sim.Update(20_ms);

  lSim.SetIntegratedSensorRawPosition(
    metersToNative(sim.GetLeftPosition())
  );
  lSim.SetIntegratedSensorVelocity(
    velocityToNative(sim.GetLeftVelocity())
  );
  rSim.SetIntegratedSensorRawPosition(
    -metersToNative(sim.GetRightPosition())
  );
  rSim.SetIntegratedSensorVelocity(
    -velocityToNative(sim.GetRightVelocity())
  );

  SmartDashboard::PutNumber("LPos", sim.GetLeftPosition().value());
  SmartDashboard::PutNumber("LVel", sim.GetLeftVelocity().value());
  SmartDashboard::PutNumber("RPos", sim.GetRightPosition().value());
  SmartDashboard::PutNumber("RVel", sim.GetRightVelocity().value());
  SmartDashboard::PutNumber("LVolts", lSim.GetMotorOutputLeadVoltage());
  SmartDashboard::PutNumber("RVolts", rSim.GetMotorOutputLeadVoltage());
  SmartDashboard::PutNumber("SimAngle", -sim.GetHeading().Degrees().value());
  simAngle.Set(-sim.GetHeading().Degrees().value());
}

/** Converts meters to native units */
int DriveSubsystem::metersToNative(units::meter_t position) {
  double wheelRotations = (position / WHEEL_CIRCUMFERENCE).value();
  double motorRotations = wheelRotations * SHAFT_TO_WHEEL_GEAR_RATIO;
  double nativeUnits = motorRotations * TALON_UNITS_PER_ROTATION;
  return (int)(nativeUnits);
}

/** Converts m/s velocity to native units */
int DriveSubsystem::velocityToNative(units::meters_per_second_t velocity) {
  return (int)(metersToNative(units::meter_t{velocity.value()}) / 10);  // Convert m/s to m/100ms
}

/** Converts native encoder units to meters moved, accounting for gearing and wheels */
units::meter_t DriveSubsystem::nativeToMeters(double nativeUnits) {
  SmartDashboard::PutNumber("NU", nativeUnits);
  double rotations = nativeUnits / TALON_UNITS_PER_ROTATION;         // convert native units to rotations
  double wheelRotations = rotations / SHAFT_TO_WHEEL_GEAR_RATIO;     // convert rotations of motor shaft to rotations of wheel
  double linearDisplacement = wheelRotations * units::meter_t{WHEEL_CIRCUMFERENCE}.value();  // convert wheel rotations to linear displacement
  SmartDashboard::PutNumber("LDisp", linearDisplacement);
  return units::meter_t{linearDisplacement};
}

/* Converts native velocity units to m/s */
units::meter_t DriveSubsystem::nativeVelocityToMeters(double velocity) {
  return nativeToMeters(velocity) * 10; // converts m/100ms to m/s
}
