/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FieldRelative.h"
#include <frc/smartdashboard/SmartDashboard.h>
FieldRelative::FieldRelative(DriveSubsystem* subsystem)
    : m_Field(subsystem) {
        AddRequirements({subsystem});
    }

void FieldRelative::Initialize() { 
    m_Field->ToggleFieldRel(); }

bool FieldRelative::IsFinished() { return true; }
