package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CtreUtils.Devices.Falcon500;

public class ELEVATOR {
  // Elevator sim constants
  public static final DCMotor gearbox = DCMotor.getFalcon500(2);
  public static final double gearRatio = 8.82; // Real value 15.7?
  public static final double massKg = 4.0;
  public static final double drumRadiusMeters = Units.inchesToMeters(1.5);
  public static final Rotation2d mountAngleRadians = Rotation2d.fromDegrees(40);
  public static final double centerOffset = Units.inchesToMeters(14);
  public static final double carriageDistance = Units.inchesToMeters(7);
  public static final double carriageOffset = Units.inchesToMeters(11);
  public static final int mech2dAngleDegrees = 35;

  public static final double kMaxReverseOutput = -0.45;

  // PID
  public static final double kMaxVel = Units.inchesToMeters(238);
  public static final double kMaxAccel = Units.inchesToMeters(520);
  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 0;

  public static final double encoderCountsToMeters =
      (drumRadiusMeters * 2 * Math.PI) / (Falcon500.kSensorUnitsPerRotation * gearRatio);

  public static final double kG = 0.02;
  public static final double kV = 20.0; // 12.57;
  public static final double kA = 0.02; // 0.04;

  public static final double kP = 0.05;
  public static final double kI = 0.00;
  public static final double kD = 0.00;

  public static final double kPercentOutputMultiplier = 0.2;
  public static final double kLimitedPercentOutputMultiplier = 0.1;

  // TODO: Upgrade to Phoenix6
  //  public static TalonFXInvertType mainMotorInversionType = TalonFXInvertType.Clockwise;

  public static final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);

  public enum SETPOINT {
    STOWED(Units.inchesToMeters(0.0)),
    INTAKING_LOW(STOWED.get()),
    SCORE_LOW_REVERSE(Units.inchesToMeters(0.0)),
    SCORE_LOW_CONE(Units.inchesToMeters(4.0)),
    SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
    SCORE_MID_CONE(Units.inchesToMeters(24.0)),
    SCORE_MID_CUBE(Units.inchesToMeters(30.0)),
    SCORE_HIGH_CONE(Units.inchesToMeters(44.0)),
    SCORE_HIGH_CUBE(Units.inchesToMeters(45.0)),
    INTAKING_EXTENDED_CONE(Units.inchesToMeters(31.59)),
    INTAKING_EXTENDED_CUBE(Units.inchesToMeters(37.0));

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum THRESHOLD {
    // Units are in meters
    // Used to tell current zone for transitions
    ABSOLUTE_MIN(Units.inchesToMeters(0.0)),
    // switch to reset it
    ABSOLUTE_MAX(Units.inchesToMeters(50.0)),
    // NOTE: Zone limits should overlap to allow for transitions
    // Alpha 0 < x < 3.5 inches
    // Beta 3 < x < 28 inches
    // Gamma 27.5 < x < 50 inches
    ALPHA_MIN(ABSOLUTE_MIN.get()),
    ALPHA_MAX(Units.inchesToMeters(15.5)),
    BETA_MIN(Units.inchesToMeters(15.0)),
    BETA_MAX(Units.inchesToMeters(29)),
    GAMMA_MIN(Units.inchesToMeters(28.5)),
    GAMMA_MAX(ABSOLUTE_MAX.get());

    private final double value;

    THRESHOLD(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
