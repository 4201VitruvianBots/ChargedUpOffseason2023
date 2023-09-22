// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.CtreUtils.Devices.CANCoder;
import frc.robot.utils.SwerveUtils.SWERVE_MODULE_POSITION;

/** Add your docs here. */
public class SwerveModule {

  public SwerveModule(
      SWERVE_MODULE_POSITION backRight,
      TalonFX talonFX,
      TalonFX talonFX2,
      CANCoder canCoder,
      double backLeftCANCoderOffset) {}

  public SwerveModule(
      TalonFX talonFX, TalonFX talonFX2, CANCoder canCoder, double backLeftCANCoderOffset) {}
}
