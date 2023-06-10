/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() {
  
  // Implementation of subsystem constructor goes here.
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

frc::Rotation2d DriveSubsystem::GetAngle() {//had to remove "const" after GetAngle() to get it to work with Navx
    // Negating the angle because WPILib Gyros are CW positive.
    
    return frc::Rotation2d(units::degree_t(-m_gyro.GetAngle())); 
    
  }

void DriveSubsystem::Drive(double xSpeed, double ySpeed, double rot, bool trigger) {
  frc::SmartDashboard::PutNumber("currentgyro",m_gyro.GetAngle());
  if (fabs(xSpeed)<0.10){minX=0.0;}
  else {minX = 1.0;
    xSpeed = copysign((fabs(xSpeed) - 0.1)*10/9,xSpeed);}//gives a 10% dead zone and then stretches the rest from 0-100%

  if (fabs(ySpeed)<0.10){minY=0.0;}
  else {minY = 1.0;
    ySpeed = copysign((fabs(ySpeed) - 0.1)*10/9,ySpeed);}

  if (fabs(rot)<0.10){minRot=0.0;}
  else {minRot = 1.0;
    rot = copysign((fabs(rot) - 0.1)*10/9,rot);}

    quadx = copysign(pow(xSpeed,2),xSpeed);
    xSpeed = -quadx*minX;//used to have *MaxS

    quady = copysign(pow(ySpeed,2),ySpeed);
    ySpeed = -quady*minY;//used to have *MaxS

    quadrot = copysign(pow(rot,2),rot);
    rot=-quadrot*minRot;//try multiplying this by maxs too if linear speed is too slow when rotating.

    
  fieldRelative = GetFieldRel();
  
  
                     
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          units::meters_per_second_t (xSpeed), units::meters_per_second_t (ySpeed),
                          units::radians_per_second_t (rot), GetAngle())
                    : frc::ChassisSpeeds{units::meters_per_second_t (xSpeed), units::meters_per_second_t (ySpeed),
                          units::radians_per_second_t (rot)});

  if (trigger && triggermult >= 1.0) {
    triggermult = triggermult + .02;
    if (triggermult>2.0){triggermult=2.0;}
    }
  else {triggermult = triggermult - .02;
      if (triggermult < 1.0) {triggermult = 1.0;}}

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed*triggermult);

  auto [fl, fr, bl, br] = states;
//const frc::SwerveModuleState& state, bool m_sbwrite, double kthrottle, bool ktrigger
  m_frontLeft.SetDesiredState(fl,false,1); 
  m_frontRight.SetDesiredState(fr,false,2);
  m_backLeft.SetDesiredState(bl, true,3);
  m_backRight.SetDesiredState(br,false,4);
  frc::SmartDashboard::PutBoolean("fieldRel",fieldRelative);
  frc::SmartDashboard::PutNumber("Gyro", fmod(GetAngleAsDouble(),360));
}


void DriveSubsystem::ResetGyro() {
  m_gyro.Reset();
  frc::SmartDashboard::PutNumber("gyroreset", m_gyro.GetAngle());
}



void DriveSubsystem::FieldRelTrue(){fieldRelative=true;}

void DriveSubsystem::FieldRelFalse(){fieldRelative=false;}

void DriveSubsystem::ToggleFieldRel() {
  if (!fieldRelative) {
    ResetGyro();
    fieldRelative=true;
    }
  else {fieldRelative=false;}
    
}

bool DriveSubsystem::GetFieldRel() {
  return fieldRelative;
}


double DriveSubsystem::GetWheelPosition(){
  return m_frontLeft.GetEncoderPos();
}

double DriveSubsystem::GetAngleAsDouble(){
  return m_gyro.GetAngle();
}

//void DriveSubsystem::UpdateOdometry() {
//  m_odometry.Update(GetAngle(), m_frontLeft.GetState(), m_frontRight.GetState(),
//   m_backLeft.GetState(), m_backRight.GetState());
//}

void DriveSubsystem::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}