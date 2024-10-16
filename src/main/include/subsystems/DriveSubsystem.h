#pragma once

#include "Mechanism Modules/SwerveModule.h"

#include <cmath>

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc2/command/Subsystembase.h>

class DriveSubsystem : public frc2::SubsystemBase{
    public:
        DriveSubsystem();

        void Periodic();

        void Drive(
            units::velocity::feet_per_second_t xSpeed,
            units::velocity::feet_per_second_t ySpeed, 
            units::angular_velocity::turns_per_second_t rot,
            bool fieldRelative);

        void ResetMotorEncoders();

        //frc:: GetPose();


    private:
        std::unique_ptr<SwerveModule> m_frontLeftSwerveModule;
        std::unique_ptr<SwerveModule> m_frontRightSwerveModule;
        std::unique_ptr<SwerveModule> m_backLeftSwerveModule;
        std::unique_ptr<SwerveModule> m_backRightSwerveModule;

        ctre::phoenix6::configs::TalonFXConfiguration DriveMotorConfigs;
        ctre::phoenix6::configs::TalonFXConfiguration RotationMotorConfigs;
        ctre::phoenix6::hardware::Pigeon2 driveGyro;

/*        frc::SwerveDriveKinematics<4> m_kinematics{
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation};
*/

        //Most of the below variables are temporary for calculating the Wheel Speed for each module
        struct wheelVars {
            double wheelSpeed = 0;
            double wheelAngle = 0;
        };

        double pi = 3.14159;//Should probably make a global pi constant in constants.h later

        wheelVars FRMovement;
        wheelVars FLMovement;
        wheelVars BLMovement;
        wheelVars BRMovement;
        double maxSpeed;

        double A = 0;
        double B = 0;
        double C = 0;
        double D = 0;
        double radiusToWheels = 0;
    

};