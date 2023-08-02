package testUtils;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class CtreUtils {
  public static class Devices {
    public static class Falcon500 {
      public static final int kSensorUnitsPerRotation = 2048;
    }

    public static class CANCoder {
      public static final int kSensorUnitsPerRotation = 4096;
    }
  }

  public static Slot0Configs generateTurnMotorConfig() {
    var motorConfig = new Slot0Configs();

    //    motorConfig.kF = 0.0;
    motorConfig.kP = 0.6; // 0.8;
    motorConfig.kI = 0.0001;
    motorConfig.kD = 12; // 0.0;
    //    motorConfig.allowableClosedloopError = 0.0;

    //    motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);

    return motorConfig;
  }

  public static Slot0Configs generateDriveMotorConfig() {
    var motorConfig = new Slot0Configs();

    //    motorConfig.slot0.kF = 0.0;
    motorConfig.kP = 0.1;
    motorConfig.kI = 0.0;
    motorConfig.kD = 0.0;

    //    motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);

    //    motorConfig.openloopRamp = 0.25;
    //    motorConfig.closedloopRamp = 0.1;

    return motorConfig;
  }

  // TODO: Upgrade to Phoenix6
  //  public static CANCoderConfiguration generateCanCoderConfig() {
  //    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();
  //
  //    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
  //    sensorConfig.sensorDirection = false;
  //    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  //
  //    return sensorConfig;
  //  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
