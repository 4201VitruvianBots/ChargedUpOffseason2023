package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SwerveUtils.*;
import java.util.Map;

public class SWERVE {
  public static final double kTrackWidth = Units.inchesToMeters(24);
  public static final double kWheelBase = Units.inchesToMeters(24);

  public static final Map<SWERVE_MODULE_POSITION, Translation2d> kModuleTranslations =
      Map.of(
          SWERVE_MODULE_POSITION.FRONT_LEFT,
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          SWERVE_MODULE_POSITION.FRONT_RIGHT,
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          SWERVE_MODULE_POSITION.BACK_LEFT,
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          SWERVE_MODULE_POSITION.BACK_RIGHT,
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final SwerveDriveKinematics kSwerveKinematics =
      new SwerveDriveKinematics(ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

  public static final int frontLeftCanCoder = 10;
  public static final int frontRightCanCoder = 11;
  public static final int backLeftCanCoder = 12;
  public static final int backRightCanCoder = 13;
  public static double frontLeftCANCoderOffset = 125.068;
  public static double frontRightCANCoderOffset = 62.051;
  public static double backLeftCANCoderOffset = 190.635;
  public static double backRightCANCoderOffset = 31.904;
  public static final int frontLeftDriveMotor = 20;
  public static final int frontLeftTurnMotor = 21;
  public static final int frontRightDriveMotor = 22;
  public static final int frontRightTurnMotor = 23;
  public static final int backLeftDriveMotor = 24;
  public static final int backLeftTurnMotor = 25;
  public static final int backRightDriveMotor = 26;
  public static final int backRightTurnMotor = 27;

  public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
  public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
  public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
  public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
  public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;

  public static final double kP_X = 0.6;
  public static final double kI_X = 0;
  public static final double kD_X = 0;
  public static final double kP_Y = 0.6;
  public static final double kI_Y = 0;
  public static final double kD_Y = 0;

  public static double kP_Theta = 4.0;
  public static double kI_Theta = 0;
  public static double kD_Theta = 0.01;
}
