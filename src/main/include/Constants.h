// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include <units/length.h>


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace SwerveConstants {

    const int kFrontRightDriveMotorPort = 2;
    const int kFrontRightTurnMotorPort = 3;
    const int kFrontRightAbsoluteEncoderPort = 22;

    const int kFrontLeftDriveMotorPort = 12;
    const int kFrontLeftTurnMotorPort = 13;
    const int kFrontLeftAbsoluteEncoderPort = 23;


    const int kBackLeftDriveMotorPort = 14;
    const int kBackLeftTurnMotorPort = 15;
    const int kBackLeftAbsoluteEncoderPort = 24;

    const int kBackRightDriveMotorPort = 0;
    const int kBackRightTurnMotorPort = 1;
    const int kBackRightAbsoluteEncoderPort = 21; 

    const int kDriveGyroPort = 25;  

    const units::angle::turn_t kDriveArbitraryOffset = 0_tr;

    const units::length::inch_t kTrackWidth = 27_in;
    const units::length::inch_t kWheelBase = 30_in;

    //Locations for the swerve drive modules relative to the robot center. In this case they can be derived from the trackWidth and wheelBase
    const frc::Translation2d kFrontRightLocation{(kWheelBase/2), -(kTrackWidth/2)};
    const frc::Translation2d kFrontLeftLocation{(kWheelBase/2), (kTrackWidth/2)};
    const frc::Translation2d kBackLeftLocation{-(kWheelBase/2), (kTrackWidth/2)};
    const frc::Translation2d kBackRightLocation{-(kWheelBase/2), -(kTrackWidth/2)};

    const units::length::inch_t kWheelDiameter = 4_in;
}
