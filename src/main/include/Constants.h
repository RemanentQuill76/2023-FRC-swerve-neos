/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
//#include <units/units.h>
//#include <wpi/numbers>


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
constexpr int kFLDriveMotorPort = 10;
constexpr int kFRDriveMotorPort = 11;
constexpr int kBLDriveMotorPort = 12;
constexpr int kBRDriveMotorPort = 13;

constexpr int kFLTurningMotorPort = 20;
constexpr int kFRTurningMotorPort = 21;
constexpr int kBLTurningMotorPort = 22;
constexpr int kBRTurningMotorPort = 23;
/*
constexpr int kFLTurningEncoderPort1 = 5;
constexpr int kFRTurningEncoderPort1 = 8;
constexpr int kBLTurningEncoderPort1 = 0;
constexpr int kBRTurningEncoderPort1 = 2;

constexpr int kFLTurningEncoderPort2 = 6;
constexpr int kFRTurningEncoderPort2 = 9;
constexpr int kBLTurningEncoderPort2 = 1;
constexpr int kBRTurningEncoderPort2 = 3;
*/
constexpr bool kFLTurningEncoderReversed = false;
constexpr bool kFRTurningEncoderReversed = false;
constexpr bool kBLTurningEncoderReversed = false;
constexpr bool kBRTurningEncoderReversed = true;

}  // namespace DriveConstants

namespace SwerveConstants {

/*static*/ constexpr double kWheelRadius = 0.0508;
/*  static*/ constexpr int kEncoderResolution = 415;
  
 /* static*/ constexpr double kmperrev= 2.* M_PI * kWheelRadius /6.67 ;
  /*static*/ constexpr double kmaxrevpersec= 5676./60. ;
  constexpr double kmaxvelocity=.6;//m/s of the axle i think
  //double kmaxrevpermin=kmaxvelocity*60/kmperrev;//gives max rev/min 

  /*static*/ constexpr auto kModuleMaxAngularVelocity = M_PI;
    //  wpi::numbers::pi * 1_rad_per_s;  // radians per second
  /*static*/ constexpr auto kModuleMaxAngularAcceleration = 2.0 * M_PI;
   //   wpi::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

}

namespace ContainerConstants {
  constexpr double AutoDriveDistance =100.0;
  constexpr double AutoSetHeading = 180.0;
  constexpr double ButtonSetHeading = 180.0;
}