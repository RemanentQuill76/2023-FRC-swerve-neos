/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
//#include <wpi/numbers>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>
#include "ctre/Phoenix.h"
#include "Constants.h"

class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, double encoderoffset, bool invertturn, bool sensorset);
  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();//got rid of "const" and it worked
  void SetDesiredState(const frc::SwerveModuleState& state, bool m_sbwrite, int wheel);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  double GetEncoderPos();
  
  double npitopi(double adjustangle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  
  bool sensorset;
  bool invertturn;
  double turnfeed;
  double angle;
  double mult;
  double k_encoderPos;
  double encoderoffset;
  double absolutePosition;
  double drive_speed;
  //int multiple;
  double error;
  int adjust;
  //int crossover;
  
 
  rev::CANSparkMax m_driveMotor;
 // ctre::phoenix::motorcontrol::can::WPI_TalonFX m_driveMotor;
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_turningMotor;
  //This might work:  WPI_TalonSRX * m_turningMotor;
  //rev::CANEncoder m_driveEncoder;  //Old way
  rev::SparkMaxRelativeEncoder m_driveEncoder;// = m_driveMotor.GetEncoder(rev::CANEncoder::EncoderType::kQuadrature, 4096);//added 2022
  //rev::SparkMaxRelativeEncoder m_encoder

};

