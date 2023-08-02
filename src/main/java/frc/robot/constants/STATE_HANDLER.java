package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class STATE_HANDLER {

  public enum ZONE {
    UNDEFINED, // Danger
    STATUS, // Danger
    ALPHA,
    BETA,
    GAMMA,
  }

  public static final double elevatorSetpointTolerance = Units.inchesToMeters(2);
  public static final double wristSetpointTolerance = Units.degreesToRadians(4);

  public static final double universalWristLowerLimitRadians = Units.degreesToRadians(25.0);
  public static final double universalWristUpperLimitRadians = Units.degreesToRadians(115.0);

  public static boolean limitCanUtilization = false;

  public static final double mechanism2dXSize = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get() * 2;
  public static final double mechanism2dYSize = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get() * 2;
  public static final double mechanism2dXOffset = Units.inchesToMeters(3);
  public static final double mechanism2dYOffset = Units.inchesToMeters(11);

  public enum SUPERSTRUCTURE_STATE {
    // UNDEFINED
    DANGER_ZONE(ZONE.UNDEFINED),
    // STATUS
    DISABLED(ZONE.STATUS),
    ENABLED(ZONE.STATUS),
    LOW_BATTERY(ZONE.STATUS),
    // LOWs
    STOWED(ZONE.ALPHA),
    INTAKE_LOW_CONE(ZONE.ALPHA),
    INTAKE_LOW_CUBE(ZONE.ALPHA),
    SCORE_LOW_REVERSE(ZONE.ALPHA),
    SCORE_LOW(ZONE.ALPHA),
    SCORE_LOW_CONE(ZONE.ALPHA),
    SCORE_LOW_CUBE(ZONE.ALPHA),
    ALPHA_ZONE(ZONE.ALPHA),
    WRIST_IS_RESET(ZONE.ALPHA),
    // MID
    BETA_ZONE(ZONE.BETA),
    SCORE_MID(ZONE.BETA),
    SCORE_MID_CONE(ZONE.BETA),
    SCORE_MID_CUBE(ZONE.BETA),
    // HIGH
    GAMMA_ZONE(ZONE.GAMMA),
    INTAKE_EXTENDED(ZONE.GAMMA),
    SCORE_HIGH(ZONE.GAMMA),
    SCORE_HIGH_CONE(ZONE.GAMMA),
    SCORE_HIGH_CUBE(ZONE.GAMMA);

    // State Zone is determined by elevator setpoints
    private final ZONE zone;

    SUPERSTRUCTURE_STATE(final ZONE zone) {
      this.zone = zone;
    }

    public ZONE getZone() {
      return zone;
    }
  }

  public enum SETPOINT {
    // Units are in meters, radians
    STOWED(ELEVATOR.SETPOINT.STOWED.get(), WRIST.SETPOINT.STOWED.get()),
    SCORE_LOW(ELEVATOR.SETPOINT.SCORE_LOW_CONE.get(), WRIST.SETPOINT.SCORE_LOW_CONE.get()),
    SCORE_LOW_REVERSE(
        ELEVATOR.SETPOINT.SCORE_LOW_REVERSE.get(), WRIST.SETPOINT.SCORE_LOW_REVERSE.get()),
    SCORE_MID_CONE(ELEVATOR.SETPOINT.SCORE_MID_CONE.get(), WRIST.SETPOINT.SCORE_MID_CONE.get()),

    SCORE_MID_CUBE(ELEVATOR.SETPOINT.SCORE_MID_CUBE.get(), WRIST.SETPOINT.SCORE_MID_CUBE.get()),

    SCORE_HIGH_CONE(ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get(), WRIST.SETPOINT.SCORE_HIGH_CONE.get()),

    SCORE_HIGH_CUBE(ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get(), WRIST.SETPOINT.SCORE_HIGH_CUBE.get()),

    INTAKING_EXTENDED_CONE(
        ELEVATOR.SETPOINT.INTAKING_EXTENDED_CONE.get(),
        WRIST.SETPOINT.INTAKING_EXTENDED_CONE.get()),

    INTAKING_EXTENDED_CUBE(
        ELEVATOR.SETPOINT.INTAKING_EXTENDED_CUBE.get(),
        WRIST.SETPOINT.INTAKING_EXTENDED_CUBE.get()),

    INTAKING_LOW_CONE(ELEVATOR.SETPOINT.INTAKING_LOW.get(), WRIST.SETPOINT.INTAKING_LOW_CONE.get()),
    INTAKING_LOW_CUBE(ELEVATOR.SETPOINT.INTAKING_LOW.get(), WRIST.SETPOINT.INTAKING_LOW_CUBE.get());

    private final double elevatorSetpointMeters;
    private final double wristSetpointRadians;

    SETPOINT(double elevatorSetpointMeters, double wristSetpointRadians) {
      this.elevatorSetpointMeters = elevatorSetpointMeters;
      this.wristSetpointRadians = wristSetpointRadians;
    }

    public double getElevatorSetpointMeters() {
      return elevatorSetpointMeters;
    }

    public double getWristSetpointRadians() {
      return wristSetpointRadians;
    }
  }
}
