/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
//FYI--I moved rundefaultcommand into its own fuction called on teleop init. try making this do nothing.
//ideas:  fieldrelative needs an "end" command so it can be interrupted.  Does it need to be actively interrupted?
//try getting rid of "runcommand" for default in container and adding a command to send joysticks to drive (like exmaple command)
//try moving command scheduler from robot periodic to teleop periodic.  see basictest4
//try getting rid of field relative and see if that command is messing up auto command.
//try putting a check to see if in teleop just before default command is called in container.
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_autonomousCommand = nullptr;
  m_container.InitResetGyro();

  }

/*stuff to do yet.  Add autonomous choice(drive forward.  driver forward and spin?)add more subsystems and commands (face at 0 degrees. face at 180 degrees) drive straight)*/
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
   m_container.InitResetGyro();
    m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();//I added the false to make it uninterruptable.
    
  }
  
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  m_container.InitResetGyro();//change eventually.
  m_container.DefaultCommand();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {m_container.RCTeleop();}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {

 
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
