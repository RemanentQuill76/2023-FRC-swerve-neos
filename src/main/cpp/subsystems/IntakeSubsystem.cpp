// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::out(){
    m_extender.Set(true);
    extended=true;

}

void IntakeSubsystem::in(){
    m_extender.Set(false);
    extended=false;
}

bool IntakeSubsystem::isout(){
    return extended;
}

void IntakeSubsystem::togglecylinder(){
    if(!extended){out();}
    else {in();}
}
