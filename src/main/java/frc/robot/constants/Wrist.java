package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CtreUtils.Devices.Falcon500;

public class Wrist {
  public static final double gearRatio = 1024.0 / 27.0;
  public static final double encoderUnitsToDegrees =
      360.0 / (Falcon500.kSensorUnitsPerRotation * gearRatio);
  public static final DCMotor gearBox = DCMotor.getFalcon500(1);
  public static final double mass = Units.lbsToKilograms(20);
  public static final double length = Units.inchesToMeters(20);
  public static final double fourbarGearboxHeight = Units.inchesToMeters(4);
  public static final double fourbarAngleDegrees = 180;
  public static final int kTimeoutMs = 0;

  // TODO: Upgrade to Phoenix6
  //  public static TalonFXInvertType motorInversionType = TalonFXInvertType.Clockwise;

  public static final double kPercentOutputMultiplier = 0.2;
  public static final double kLimitedPercentOutputMultiplier = 0.1;

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;

  // Values were experimentally determined
  // public static final double kMaxSlowVel = Units.degreesToRadians(400);
  // public static final double kMaxSlowAccel = Units.degreesToRadians(290);
  // public static final double kMaxFastVel = Units.degreesToRadians(400 * 1.25);
  // public static final double kMaxFastAccel = Units.degreesToRadians(290 * 1.25);

  public static final double kMaxVel = Units.degreesToRadians(720);
  public static final double kMaxAccel = Units.degreesToRadians(250);

  public static final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);

  public static final double FFkS = 0.06;
  public static final double kG = 0.54;
  public static final double FFkV = 1.6;
  public static final double kA = 0.02;

  public static final double kP = 0.04;
  public static final double kI = 0.0;
  public static final double kD = 1.0;

  public static final double kMaxPercentOutput = 1.0;
  public static final double kSetpointMultiplier = Units.degreesToRadians(60.0);

  public enum SETPOINT {
    // Units are in Radians
    STOWED(Units.degreesToRadians(95.0)),
    INTAKING_LOW_CUBE(Units.degreesToRadians(-13.5)),
    INTAKING_LOW_CONE(Units.degreesToRadians(13)),
    SCORE_LOW_REVERSE(Units.degreesToRadians(-14.0)),
    SCORE_LOW_CONE(Units.degreesToRadians(120.0)),
    SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
    SCORE_MID_CONE(Units.degreesToRadians(142.0)),
    SCORE_MID_CUBE(Units.degreesToRadians(132.0)),
    SCORE_HIGH_CONE(Units.degreesToRadians(139.0)),
    SCORE_HIGH_CUBE(Units.degreesToRadians(147.0)),
    INTAKING_EXTENDED_CONE(Units.degreesToRadians(121.3)),
    INTAKING_EXTENDED_CUBE(SCORE_HIGH_CUBE.get());

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum THRESHOLD {
    // Units are in radians
    ABSOLUTE_MIN(Units.degreesToRadians(-20.0)),
    ABSOLUTE_MAX(Units.degreesToRadians(180.0)),
    ALPHA_MIN(ABSOLUTE_MIN.get()),
    ALPHA_MAX(Units.degreesToRadians(110.0)),
    BETA_MIN(Units.degreesToRadians(25.0)),
    BETA_MAX(Units.degreesToRadians(146.0)),
    GAMMA_MIN(
        Units.degreesToRadians(
            40.0)), // TODO: Maybe change this to 25.0 like it was before as extended
    GAMMA_MAX(ABSOLUTE_MAX.get()),

    HORIZONTAL_LENGTH_MINUS15_CUBE(Units.inchesToMeters(17.0)),
    HORIZONTAL_LENGTH_MINUS15_CONE(Units.inchesToMeters(20.0)),
    HORIZONTAL_LENGTH_0_CUBE(Units.inchesToMeters(16.0)),
    HORIZONTAL_LENGTH_0_CONE(Units.inchesToMeters(19.5)),
    HORIZONTAL_LENGTH_90_CUBE(Units.inchesToMeters(-4.0)),
    HORIZONTAL_LENGTH_90_CONE(Units.inchesToMeters(-9.0)),
    HORIZONTAL_LENGTH_140_CUBE(Units.inchesToMeters(-17.0)),
    HORIZONTAL_LENGTH_140_CONE(Units.inchesToMeters(-25.0)),
    HORIZONTAL_LENGTH_180_CUBE(Units.inchesToMeters(-22.0)),
    HORIZONTAL_LENGTH_180_CONE(Units.inchesToMeters(-28.0));

    private final double value;

    THRESHOLD(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
