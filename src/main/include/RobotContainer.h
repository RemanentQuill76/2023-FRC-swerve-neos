/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once



//#include "commands/FieldRelative.h" // in .cpp already
#include "subsystems/DriveSubsystem.h"
#include "subsystems/DistanceSensor.h"
#include "subsystems/LimeLight.h"
#include "subsystems/IntakeSubsystem.h"
#include "commands/IntakeUpDown.h"
#include "commands/SenseDistance.h"
#include "commands/ComplexAuto.h"
#include "commands/DriveDistance.h"
#include "commands/BasicAuto.h"
#include "commands/SetHeading.h"
#include "commands/FieldRelative.h"
#include "commands/AutoLime.h"
#include "Constants.h"
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>//future use??
#include <frc2/command/Command.h>//not sure

#include <numbers>

#include <frc2/command/RunCommand.h>//I think this is necessary to run joysticks continually

using namespace ContainerConstants;
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();//gets chosen command from smartdashboard
  void DefaultCommand();
  void InitResetGyro();
  void ConfigureButtonBindings();
  void RCTeleop();

 private:
  
  
/*
  static constexpr units::meters_per_second_t kMaxSpeed =
      0.6_mps;  // put into constants later
  static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::numbers::pi};  //put into constants later
*/

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_swerve;
  IntakeSubsystem m_intake;
  DistanceSensor m_distanceSense;
  frc2::Command* m_autoCommand;
  DriveDistance m_simpleAuto{AutoDriveDistance, &m_swerve};//{AutoConstants::kAutoDriveDistanceInches,
                          //   AutoConstants::kAutoDriveSpeed, &m_drive};
  ComplexAuto m_complexAuto{AutoDriveDistance, AutoSetHeading, &m_swerve};//{&m_drive, &m_hatch};
  BasicAuto m_basicAuto;
  
  LimeLight m_limelight;
  frc::Joystick m_gamepad{0};
  frc::Joystick m_gamepad2{1};

  //double heading;
  double xSpeed;
  double ySpeed;
  bool Straight;
  double AutoDist;
  double m_setheading;
  nt::NetworkTableEntry e_AutoDistance;
  nt::NetworkTableEntry e_setheading;
  //double rot;
// The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

};