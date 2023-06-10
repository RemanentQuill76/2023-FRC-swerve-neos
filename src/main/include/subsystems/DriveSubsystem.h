/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogGyro.h>//needed???
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

//#include <wpi/numbers>
#include <AHRS.h>
#include <iostream>

#include "Constants.h"
#include "SwerveModule.h"
#include <frc2/command/SubsystemBase.h>

//added in 2022
#include <units/angular_velocity.h>
//#include <units/time.h>
//#include <units/velocity.h>
//#include <units/voltage.h>
#include <numbers>

using namespace DriveConstants;


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc::Rotation2d GetAngle(); //had to remove "const" after GetAngle() to get it to work with Navx
    // Negating the angle because WPILib Gyros are CW positive.
    

  void Drive(double xSpeed, double ySpeed, double rot, bool trigger);
  void UpdateOdometry();
  void ResetGyro();
  void ToggleFieldRel();
  void FieldRelTrue();
  void FieldRelFalse();
  bool GetFieldRel();
  double GetWheelPosition();
  double GetAngleAsDouble();
  
  

 private:
//int driveMotorChannel, int turningMotorChannel, double encoderoffset, bool invertturn, bool sensorset
  SwerveModule m_frontLeft{kFLDriveMotorPort,kFLTurningMotorPort,280/*encoder offset*/,false, false}; //probly need to add turning encoder numbers as separate than sparkmax
  SwerveModule m_frontRight{kFRDriveMotorPort,kFRTurningMotorPort,2530,false, false};
  SwerveModule m_backLeft{kBLDriveMotorPort,kBLTurningMotorPort,1523,false, false};
  SwerveModule m_backRight{kBRDriveMotorPort,kBRTurningMotorPort,2835,false, false};//Do these need to be public???????
 

  frc::Translation2d m_frontLeftLocation{+0.2953_m, +0.2953_m};//I think +x is forward and +y is left
  frc::Translation2d m_frontRightLocation{+0.2953_m, -0.2953_m};
  frc::Translation2d m_backLeftLocation{-0.2953_m, +0.2953_m};
  frc::Translation2d m_backRightLocation{-0.2953_m, -0.2953_m};


  AHRS m_gyro{frc::SPI::Port::kMXP};//added frc:: with update

  
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr units::meters_per_second_t kMaxSpeed =
      0.5_mps;  // put into constants later
  static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi};  //put into constants later


  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),//did say .GetPosition()
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
  

  double quadx;
  double quady;
  double quadrot;
  double throttle;
  double minX;//no motion if joystick isn't being used.
  double minY;
  double minRot;
  bool trigger;
  bool fieldRelative;
  double xSpeed;//change if necessary
  double ySpeed;
  double rot;
  double triggermult;
  
  

double MaxS=0.5;
//double MaxR=M_PI;


};
