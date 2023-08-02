package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestConstants {

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test. We need to close each subsystem properly, as they all get
  // separately initialized for each individual test, which will trip some errors due to how WPILib
  // is set up (e.g. resource errors from using the same PWM/DIO port)
  void shutdown() throws Exception {}

  // Mark all test functions with @Test
  @Test
  public void TestInitConfig() {
    BASE.initConstants();
    String testName = "Sim";
    double testKp = 0.1;
    // For booleans, you can use assertTrue/assertFalse
    assertEquals(testName, BASE.robotName);
    assertEquals(testKp, SWERVE.kP_Theta);
  }
}
