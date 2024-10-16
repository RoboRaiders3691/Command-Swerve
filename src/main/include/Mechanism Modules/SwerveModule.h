#pragma once

#include <string>
#include <units/length.h>
#include <units/angle.h>


#include "Constants.h"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/CANcoder.hpp"


class SwerveModule {
    public:

    SwerveModule(
    const std::string name,
    const int driveMotorPort,
    const int turnMotorPort,
    const int absoluteEncoderPort,
    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs,
    ctre::phoenix6::configs::TalonFXConfiguration rotationConfigs,
    units::angle::turn_t arbitraryOffset
    );

    void ZeroModuleRotation();

    void ResetDrive();

    units::angle::turn_t GetRotation();

    units::angle::turn_t GetRotationAbs();

    units::angle::turn_t GetDriveRotation();

    //units::angle::turn_t GetDriveDistance();

    void SetTargetRotation(units::angle::turn_t targetRotation);

    //void SetDriveDistance(units::length::foot_t targetDistnace); 

    void SetDriveOutput(double percentOutput);


    private:
        
        //Should be same order as initialization in constructor
        ctre::phoenix6::hardware::TalonFX turnMotor;
        ctre::phoenix6::hardware::TalonFX driveMotor;
        ctre::phoenix6::hardware::CANcoder CANcoder;
        ctre::phoenix6::controls::MotionMagicVoltage m_request;

        double driveMagnitude;
        units::angle::turn_t wheelAngle;
        units::angle::turn_t driveTarget;
        std::string m_name;        

};





