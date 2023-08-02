package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.InetAddress;
import java.net.NetworkInterface;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Base {
  public static String robotName = "";

  public static final String alphaRobotMAC = "00:80:2F:25:BC:FD";
  public static final String betaRobotMAC = "00:80:2F:19:30:B7";

  private static void initBeta() {
    robotName = "Beta";
    Swerve.frontLeftCANCoderOffset = 125.068; // 85.957;
    Swerve.frontRightCANCoderOffset = 62.051; // 41.748;
    Swerve.backLeftCANCoderOffset = 190.635; // 261.475;
    Swerve.backRightCANCoderOffset = 31.113;
  }

  private static void initAlpha() {
    robotName = "Alpha";

    Swerve.frontLeftCANCoderOffset = 126.914; // 85.957;
    Swerve.frontRightCANCoderOffset = 222.9785; // 41.748;
    Swerve.backLeftCANCoderOffset = 191.25; // 261.475;
    Swerve.backRightCANCoderOffset = 34.7605;

    // TODO: Upgrade to Phoenix6
    //    Elevator.mainMotorInversionType = TalonFXInvertType.CounterClockwise;
    //    Wrist.motorInversionType = TalonFXInvertType.Clockwise;
  }

  private static void initSim() {
    robotName = "Sim";

    Swerve.frontLeftCANCoderOffset = 0;
    Swerve.frontRightCANCoderOffset = 0;
    Swerve.backLeftCANCoderOffset = 0;
    Swerve.backRightCANCoderOffset = 0;

    Swerve.kP_Theta = 0.1;
    Swerve.kI_Theta = 0;
    Swerve.kD_Theta = 0;
  }

  private static void initUnknown() {
    robotName = "Unknown (Default Beta values)";
  }

  public static void initConstants() {
    // TODO: Change to use RoboRIO Serial instead of MAC Addresses
    // (https://www.chiefdelphi.com/t/1678-citrus-circuits-2023-cad-and-code-release/437632/15)
    //  Also, consider moving this to Codex
    String mac = "";
    try {
      var ip = InetAddress.getLocalHost();
      var networkInterfaces = NetworkInterface.getByInetAddress(ip).getHardwareAddress();
      String[] hex = new String[networkInterfaces.length];
      for (int i = 0; i < networkInterfaces.length; i++) {
        hex[i] = String.format("%02X", networkInterfaces[i]);
      }
      mac = String.join(":", hex);
    } catch (Exception e) {
      //      e.printStackTrace();
    }
    if (mac.equals(alphaRobotMAC)) {
      initAlpha();
    } else if (mac.equals(betaRobotMAC)) {
      initBeta();
    } else if (RobotBase.isSimulation()) {
      initSim();
    } else {
      initUnknown();
    }

    if (!DriverStation.isFMSAttached()) {
      StateHandler.limitCanUtilization = false;
    }

    SmartDashboard.putString("Robot Name", robotName);
  }

  public enum SCORING_STATE {
    STOWED,
    AUTO_BALANCE,
    LOW_REVERSE,
    LOW,
    MID,
    MID_CONE,
    MID_CUBE,
    HIGH,
    HIGH_CONE,
    HIGH_CUBE,
    INTAKE,
    INTAKE_EXTENDED
  }
}
