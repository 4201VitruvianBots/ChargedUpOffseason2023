// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SWERVE;
import frc.robot.utils.CtreUtils.Devices.CANCoder;
import frc.robot.utils.SwerveUtils.SWERVE_MODULE_POSITION;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class SwerveDrive extends SubsystemBase implements AutoCloseable {

  private final HashMap<SWERVE_MODULE_POSITION, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              SWERVE_MODULE_POSITION.FRONT_LEFT,
              new SwerveModule(
                  SWERVE_MODULE_POSITION.FRONT_LEFT,
                  new TalonFX(SWERVE.frontLeftTurnMotor),
                  new TalonFX(SWERVE.frontLeftDriveMotor),
                  new CANCoder(SWERVE.frontLeftCANCoderOffset),
                  SWERVE.frontLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.FRONT_RIGHT,
              new SwerveModule(
                  SWERVE_MODULE_POSITION.FRONT_RIGHT,
                  new TalonFX(SWERVE.frontRightTurnMotor),
                  new TalonFX(SWERVE.frontRightDriveMotor),
                  new CANCoder(SWERVE.frontRightCanCoder),
                  SWERVE.frontLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_LEFT,
              new SwerveModule(
                  new TalonFX(SWERVE.backLeftTurnMotor),
                  new TalonFX(SWERVE.backLeftDriveMotor),
                  new CANCoder(SWERVE.backLeftCanCoder),
                  SWERVE.backLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_RIGHT,
              new SwerveModule(
                  SWERVE_MODULE_POSITION.BACK_RIGHT,
                  new TalonFX(SWERVE.backRightDriveMotor),
                  new TalonFX(SWERVE.backRightDriveMotor),
                  new CANCoder(SWERVE.backLeftCanCoder),
                  SWERVE.backLeftCANCoderOffset)));

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub

  }
}
