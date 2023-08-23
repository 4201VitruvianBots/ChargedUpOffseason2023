// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CtreUtils.Devices.CANCoder;
import frc.robot.utils.SwerveUtils.SWERVE_MODULE_POSITION;

/** Add your docs here. */
public class SwerveDrive extends SubsystemBase implements AutoCloseable{

    Private final HashMap<SWERVE_MODULE_POSITION, SwerveModule> m_swerveModules =
        new HashMap<>(
            Map.of(
                SWERVE_MODULE_POSITION.FRONT_LEFT, 
                new SwerveModule(
                    SWERVE_MODULE_POSITION.FRONT_LEFT,
                    new TalonFX(CAN.frontLeftTurnMotor),
                    new TalonFX(CAN.frontLeftDriveMotor),
                    new CANCoder(CAN.frontLeftCanCoder),
                    SWERVE_DRIVE.frontLeftCANCoderOffset),

                
            ) );
}
