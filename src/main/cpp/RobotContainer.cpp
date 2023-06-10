/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>


using namespace ContainerConstants;

RobotContainer::RobotContainer() /*: m_autonomousCommand(&m_swerve) not sure what it does but it needs it*/ {
  // Initialize all of your commands and subsystems here

 // Add commands to the autonomous command chooser
  m_chooser.SetDefaultOption("Simple Auto", &m_simpleAuto);
  m_chooser.AddOption("Complex Auto", &m_complexAuto);
  m_chooser.AddOption("BasicAuto", &m_basicAuto);
 
  
  // Put the chooser on the dashboard
 frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);


  m_intake.in();
  //this fails more  frc::SmartDashboard::PutData("Autonomous", &m_chooser);
  // Configure the button bindings
  ConfigureButtonBindings();

  if(m_limelight.IsAutoLimeLightOn()){m_limelight.ToggleLight();}
  m_swerve.FieldRelFalse();
  Straight = false;
}//get rid of this to move setdefaultcommand back into constructor
  //maybe move next section into a function called at teleop init.
void RobotContainer::DefaultCommand(){ 
  m_swerve.SetDefaultCommand(frc2::RunCommand(
      [this] {
        if(frc::DriverStation::GetStickButtonReleased(0,3)){m_swerve.ResetGyro();}
        if(frc::DriverStation::GetStickButtonPressed(1,2)){Straight = true;}
        if(frc::DriverStation::GetStickButtonReleased(1,2)){Straight = false;}
        if (!Straight){
        m_swerve.Drive(m_gamepad.GetRawAxis(1), m_gamepad.GetRawAxis(0),m_gamepad.GetRawAxis(4),
           m_gamepad.GetRawAxis(2));
         
        //heading = m_swerve.GetAngleAsDouble();
        xSpeed = m_gamepad.GetRawAxis(1);
        ySpeed = m_gamepad.GetRawAxis(0);
      
        //rot = m_controller.GetTwist();
        }
        else {m_swerve.Drive(xSpeed, ySpeed,m_gamepad.GetRawAxis(4),m_gamepad.GetRawAxis(2));}
      },
      {&m_swerve}));
  
}

void RobotContainer::InitResetGyro(){m_swerve.ResetGyro();}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&m_gamepad,3).WhenPressed(new FieldRelative(&m_swerve));//have to have &m_gamepad and not a number??
  frc2::JoystickButton(&m_gamepad,1).WhenPressed(new IntakeUpDown(&m_intake));
  //frc2::JoystickButton(&m_gamepad,4).WhenPressed(new SetHeading(ButtonSetHeading, &m_swerve));//need m_swerve to get angle
  //frc2::JoystickButton{&m_gamepad,2}.ToggleWhenPressed(new SenseDistance(&m_distanceSense, &m_swerve));
  //frc2::JoystickButton(&m_gamepad,10).ToggleWhenPressed(new AutoLime(&m_limelight, &m_swerve, &m_distanceSense));
}//try getting rid of "new"

void RobotContainer::RCTeleop(){
  m_distanceSense.GetDistance();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
 
  // An example command will be run in autonomous
  
  return m_chooser.GetSelected();
  //return &m_simpleAuto;
}

