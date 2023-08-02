// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.USB;
import frc.robot.simulation.MemoryLog;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements AutoCloseable {
  private final DataLog m_logger = DataLogManager.getLog();

  // Initialize used utils
  private final MemoryLog m_memorylog = new MemoryLog();
  //  private final DistanceSensor m_distanceSensor = new DistanceSensor();

  private Elevator m_elevator;

  // The robot's subsystems and commands are defined here...
  //  private SwerveDrive m_swerveDrive;
  //  private Intake m_intake;
  //  private Wrist m_wrist;
  private Controls m_controls;
  //  private Vision m_vision;
  private SendableChooser<Command> m_autoChooser;
  //  private LEDSubsystem m_led;
  //  private StateHandler m_stateHandler;
  //  private FieldSim m_fieldSim;

  private SendableChooser<List<PathPlannerTrajectory>> autoPlotter;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);

  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);

  private final Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
  private final Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons

  public RobotContainer() {
    initializeSubsystems();
    resetSubsystemPositions();
    m_logger.pause();
    configureBindings();

    initializeAutoChooser();
  }

  public void initializeSubsystems() {
    if (RobotBase.isReal()) {
      //      m_swerveDrive = new SwerveDrive();
      //      m_intake = new Intake();
      m_elevator = new Elevator(new ElevatorIOReal());
      //      m_wrist = new Wrist(m_intake);
      m_controls = new Controls();
      //      m_vision = new Vision(m_swerveDrive, m_logger, m_controls, m_intake);
      m_autoChooser = new SendableChooser<>();
      //      m_led = new LEDSubsystem(m_controls);
      //      m_stateHandler =
      //          new StateHandler(m_intake, m_wrist, m_swerveDrive, m_elevator, m_vision);
      //      m_fieldSim =
      //          new FieldSim(m_swerveDrive, m_vision, m_elevator, m_wrist, m_stateHandler,
      // m_controls);
    } else {
      //      m_swerveDrive = new SwerveDrive();
      //      m_intake = new Intake();
      m_elevator = new Elevator(new ElevatorIOSim());
      //      m_wrist = new Wrist(m_intake);
      m_controls = new Controls();
      //      m_vision = new Vision(m_swerveDrive, m_logger, m_controls, m_intake);
      m_autoChooser = new SendableChooser<>();
      //      m_led = new LEDSubsystem(m_controls);
      //      m_stateHandler =
      //          new StateHandler(m_intake, m_wrist, m_swerveDrive, m_elevator, m_vision);
      //      m_fieldSim =
      //          new FieldSim(m_swerveDrive, m_vision, m_elevator, m_wrist, m_stateHandler,
      // m_controls);
    }

    //    m_swerveDrive.setDefaultCommand(
    //        new SetSwerveDrive(
    //            m_swerveDrive,
    //            () -> leftJoystick.getRawAxis(1),
    //            () -> leftJoystick.getRawAxis(0),
    //            () -> rightJoystick.getRawAxis(0)));

    // Control elevator height by moving the joystick up and down
    //    m_elevator.setDefaultCommand(new RunElevatorJoystick(m_elevator,
    // xboxController::getLeftY));
    //    m_wrist.setDefaultCommand(new RunWristJoystick(m_wrist, xboxController::getRightY));
    //    m_led.setDefaultCommand(
    //        new GetSubsystemStates(m_led, m_intake, m_stateHandler, m_wrist, m_elevator));
  }

  private void resetSubsystemPositions() {
    //    if (!m_logManager.initTempExists()) {
    //      m_elevator.setDesiredPositionMeters(0);
    //      m_logManager.createInitTempFile();
    //    } else {
    //      m_wrist.setWristInitialized(true);
    //    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    for (int i = 0; i < leftJoystickTriggers.length; i++)
      leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightJoystickTriggers.length; i++)
      rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));

    //    leftJoystickTriggers[0].whileTrue(new AutoBalance(m_swerveDrive));

    //    leftJoystickTriggers[1].whileTrue(
    //        new IntakeVisionAlignment(
    //            m_vision,
    //            m_swerveDrive,
    //            () -> leftJoystick.getRawAxis(1),
    //            () -> leftJoystick.getRawAxis(0),
    //            () -> rightJoystick.getRawAxis(0)));

    //    rightJoystickTriggers[0].whileTrue(new LimitSwerveJoystickInput(m_swerveDrive));

    //    xboxController
    //        .leftTrigger(0.1)
    //        .whileTrue(
    //            new ConditionalCommand(
    //                new SetIntakeState(m_intake, INTAKE_STATE.INTAKING_CUBE),
    //                new SetIntakeState(m_intake, INTAKE_STATE.SCORING_CONE),
    //                m_stateHandler::isScoring));
    //    xboxController.leftTrigger().onFalse(new SetIntakeState(m_intake, INTAKE_STATE.NONE));
    //    xboxController
    //        .rightTrigger(0.1)
    //        .whileTrue(
    //            new ConditionalCommand(
    //                new SetIntakeState(m_intake, INTAKE_STATE.INTAKING_CONE),
    //                new SetIntakeState(m_intake, INTAKE_STATE.SCORING_CUBE),
    //                m_stateHandler::isScoring));
    //    xboxController.rightTrigger().onFalse(new SetIntakeState(m_intake, INTAKE_STATE.NONE));
    //
    //    // Score button Bindings
    //
    //    // Extended
    //    xboxController
    //        .a()
    //        .whileTrue(
    //            new SetConditionalSetpoint(
    //                m_stateHandler, m_elevator, m_wrist, m_intake,
    // SCORING_STATE.INTAKE_EXTENDED));
    //
    //    // Score MID Setpoints
    //    xboxController
    //        .b()
    //        .whileTrue(
    //            new SetConditionalSetpoint(
    //                m_stateHandler, m_elevator, m_wrist, m_intake, SCORING_STATE.MID));
    //    // Stowed
    //    xboxController
    //        .x()
    //        .whileTrue(
    //            new SetSetpoint(
    //                m_stateHandler, m_elevator, m_wrist, STATE_HANDLER.SETPOINT.STOWED)); // High
    //    xboxController
    //        .y()
    //        .whileTrue(
    //            new SetConditionalSetpoint(
    //                m_stateHandler, m_elevator, m_wrist, m_intake, SCORING_STATE.HIGH));
    //
    //    // Will switch between closed and open loop on button press
    //    xboxController.back().onTrue(new ToggleElevatorControlMode(m_elevator));
    //    xboxController.start().onTrue(new ToggleWristControlMode(m_wrist));
    //    xboxController
    //        .rightBumper()
    //        .whileTrue(
    //            new SetSetpoint(
    //                m_stateHandler, m_elevator, m_wrist,
    // STATE_HANDLER.SETPOINT.INTAKING_LOW_CONE));
    //    xboxController
    //        .leftBumper()
    //        .whileTrue(
    //            new SetSetpoint(
    //                m_stateHandler, m_elevator, m_wrist,
    // STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE));
    //
    //    xboxController.povLeft().whileTrue(new SetUseCubeSetpoint(m_intake));
    //    xboxController.povDown().onTrue(new RunIntakeCone(m_intake, 0.2).withTimeout(.25));
    //
    //    // Will switch our target node on the field sim to the adjacent node on D-pad
    //    // press
    //    xboxController.povRight().whileTrue(new RunIntakeCone(m_intake, -0.2));
    //
    //    // Will limit the speed of our elevator or wrist when the corresponding joystick
    //    // is being pressed down
    //    xboxController.leftStick().whileTrue(new LimitElevatorJoystickInput(m_elevator));
    //    xboxController.rightStick().whileTrue(new LimitWristJoystickInput(m_wrist));
    //
    //    // Add Smartdashboard Buttons
    //    SmartDashboard.putData(new ResetOdometry(m_swerveDrive));
    //    SmartDashboard.putData(new SetSwerveNeutralMode(m_swerveDrive, NeutralMode.Coast));
    //    SmartDashboard.putData(new SetRollOffset(m_swerveDrive));
    //    SmartDashboard.putData(new ZeroAllSensors(m_swerveDrive, m_elevator, m_wrist));
    //    SmartDashboard.putData("ResetWrist90", new ResetWristAngleDegrees(m_wrist, 90));
    //    SmartDashboard.putData(new ToggleElevatorNeutralMode(m_elevator));
    //
    //    if (!DriverStation.isFMSAttached()) {
    //      SmartDashboard.putData(new ToggleElevatorTestMode(m_elevator, m_stateHandler));
    //      SmartDashboard.putData(new ToggleWristTestMode(m_wrist, m_stateHandler));
    //      SmartDashboard.putData(new ToggleTestIntakeState(m_stateHandler));
    //      SmartDashboard.putData(new ToggleSmartScoring(m_stateHandler));
    //    }
    initTestController();
  }

  private void initTestController() { // TODO: Rewrite this to use the new StateHandler system
    if (RobotBase.isSimulation()) {
      CommandPS4Controller testController = new CommandPS4Controller(3);

      //      testController
      //          .axisGreaterThan(3, 0.1)
      //          .whileTrue(new SetIntakeState(m_intake, INTAKE_STATE.INTAKING_CUBE));
      //      testController
      //          .axisGreaterThan(3, 0.1)
      //          .whileTrue(
      //              new ConditionalCommand(
      //                  new SetWristSetpoint(
      //                      m_wrist, WRIST.SETPOINT.INTAKING_LOW_CONE.get(),
      // testController::getRightY),
      //                  new SetWristSetpoint(
      //                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(),
      // testController::getRightY),
      //                  () ->
      //                      m_stateHandler.getCurrentState().getZone()
      //                          == SUPERSTRUCTURE_STATE.ALPHA_ZONE.getZone()));
      //
      //      testController
      //          .axisGreaterThan(4, 0.1)
      //          .whileTrue(new SetIntakeState(m_intake, INTAKE_STATE.INTAKING_CONE));
      //      testController
      //          .axisGreaterThan(4, 0.1)
      //          .whileTrue(
      //              new ConditionalCommand(
      //                  new SetWristSetpoint(
      //                      m_wrist, WRIST.SETPOINT.INTAKING_LOW_CONE.get(),
      // testController::getRightY),
      //                  new SetWristSetpoint(
      //                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(),
      // testController::getRightY),
      //                  () ->
      //                      m_stateHandler.getCurrentState().getZone()
      //                          == SUPERSTRUCTURE_STATE.ALPHA_ZONE.getZone()));
      //
      //      // Score button Bindings
      //
      //      // Score LOW Setpoints
      //      testController
      //          .cross()
      //          .whileTrue(
      //              new SetSetpoint(
      //                  m_stateHandler, m_elevator, m_wrist,
      // STATE_HANDLER.SETPOINT.INTAKING_LOW_CONE));
      //
      //      // Score MID Setpoints
      //      testController
      //          .circle()
      //          .whileTrue(
      //              new SetSetpoint(
      //                  m_stateHandler, m_elevator, m_wrist,
      // STATE_HANDLER.SETPOINT.SCORE_MID_CONE));
      //
      //      // Stowed
      //      testController
      //          .square()
      //          .whileTrue(
      //              new SetSetpoint(m_stateHandler, m_elevator, m_wrist,
      // STATE_HANDLER.SETPOINT.STOWED));
      //
      //      // High
      //      testController
      //          .triangle()
      //          .whileTrue(
      //              new SetSetpoint(
      //                  m_stateHandler, m_elevator, m_wrist,
      // STATE_HANDLER.SETPOINT.SCORE_HIGH_CONE));
      //
      //      // Toggle elevator, wrist control state
      //      testController
      //          .povDown()
      //          .onTrue(new SetElevatorSetpoint(m_elevator, ELEVATOR.SETPOINT.STOWED.get()));
      //      testController.povDown().onTrue(new SetWristSetpoint(m_wrist,
      // WRIST.SETPOINT.STOWED.get()));
      //
      //      // Will limit the speed of our elevator or wrist when the corresponding joystick is
      // being
      //      // pressed down
      //      testController.L3().whileTrue(new LimitElevatorJoystickInput(m_elevator));
      //      testController.R3().whileTrue(new LimitWristJoystickInput(m_wrist));
      //
      //      // Will switch between closed and open loop on button press
      //      testController.share().onTrue(new ToggleElevatorControlMode(m_elevator));
      //      testController.options().onTrue(new ToggleWristControlMode(m_wrist));
    }
  }

  public void disableInit() {
    //    m_swerveDrive.setNeutralMode(NeutralMode.Coast);
  }

  public void teleopInit() {
    //    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
    m_elevator.teleopInit();
    //    m_wrist.setSetpointPositionRadians(m_wrist.getPositionRadians());
    //    m_wrist.resetTrapezoidState();
    //    m_stateHandler.init();
  }

  public void autonomousInit() {
    //    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
    m_elevator.setDesiredPositionMeters(m_elevator.getHeightMeters());
    m_elevator.resetTrapezoidState();
    //    m_wrist.setSetpointPositionRadians(m_wrist.getPositionRadians());
    //    m_wrist.resetTrapezoidState();
    //    m_stateHandler.init();
  }

  /** Use this to pass the autonomous command to the main {@link Robot} class. */
  public void initializeAutoChooser() {}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  //  public SwerveDrive getSwerveDrive() {
  //    return m_swerveDrive;
  //  }

  public Elevator getElevator() {
    return m_elevator;
  }

  //  public Wrist getWrist() {
  //    return m_wrist;
  //  }

  //  public Intake getIntake() {
  //    return m_intake;
  //  }

  //  public Vision getVision() {
  //    return m_vision;
  //  }

  public Controls getControls() {
    return m_controls;
  }

  //  public LEDSubsystem getLEDs() {
  //    return m_led;
  //  }

  //  public StateHandler getStateHandler() {
  //    return m_stateHandler;
  //  }

  //  public FieldSim getFieldSim() {
  //    return m_fieldSim;
  //  }

  public Joystick getLeftJoystick() {
    return leftJoystick;
  }

  //  public DistanceSensor getDistanceSensor() {
  //    return m_distanceSensor;
  //  }

  public void periodic() {
    // m_fieldSim.periodic();
    // Rumbles the controller if the robot is on target based off FieldSim
    //    xboxController.getHID().setRumble(RumbleType.kBothRumble, m_stateHandler.isOnTarget() ? 1
    // : 0);
    //    m_distanceSensor.periodic();
    // m_logManager.periodic();
  }

  public void disabledPeriodic() {}

  //  public void testPeriodic() {
  //    m_stateHandler.testPeriodic();
  //  }

  public void simulationPeriodic() {
    m_memorylog.simulationPeriodic();

    //    if (DriverStation.isDisabled()) m_fieldSim.setTrajectory(autoPlotter.getSelected());
  }

  @Override
  public void close() throws Exception {
    //    m_fieldSim.close();
    //    m_stateHandler.close();
    //    m_vision.close();
    //    m_led.close();
    //    m_wrist.close();
    //    m_swerveDrive.close();
    m_elevator.close();
    //    m_intake.close();
    m_controls.close();

    //    m_distanceSensor.close();
    m_logger.close();
  }
}
