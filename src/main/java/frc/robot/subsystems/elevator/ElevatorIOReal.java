// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.CAN;
import frc.robot.constants.ELEVATOR;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {
  // Initializing both motors
  protected final TalonFX[] elevatorMotors = {
    new TalonFX(CAN.elevatorMotorLeft), new TalonFX(CAN.elevatorMotorRight)
  };

  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  public ElevatorIOReal() {
    for (TalonFX motor : elevatorMotors) {
      motor.getConfigurator().apply(new TalonFXConfiguration());
      // TODO: Update to Phoenix6
      //      motor.setNeutralMode(m_neutralMode);
      //      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      //      motor.setSelectedSensorPosition(0.0);

      // Config PID
      //      motor.selectProfileSlot(ELEVATOR.kSlotIdx, ELEVATOR.kPIDLoopIdx);
      //      motor.config_kP(ELEVATOR.kSlotIdx, ELEVATOR.kP, ELEVATOR.kTimeoutMs);
      //      motor.config_kI(ELEVATOR.kSlotIdx, ELEVATOR.kI, ELEVATOR.kTimeoutMs);
      //      motor.config_kD(ELEVATOR.kSlotIdx, ELEVATOR.kD, ELEVATOR.kTimeoutMs);
      // motor.config_IntegralZone(ELEVATOR.kSlotIdx, 0);

      // Setting hard limits as to how fast the elevator can move forward and backward
      //      motor.configPeakOutputForward(ELEVATOR.kMaxForwardOutput, ELEVATOR.kTimeoutMs);
      // TODO: Review after new elevator is integrated
      //      motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50,
      // 0.1));
    }
    //    elevatorMotors[0].configPeakOutputReverse(ELEVATOR.kMaxReverseOutput,
    // ELEVATOR.kTimeoutMs);

    // Setting the right motor to output the same as the left motor
    //    elevatorMotors[0].setInverted(ELEVATOR.mainMotorInversionType);
    // elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());
    // elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);
    //    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    //    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.percentOutput = elevatorMotors[0].get();
    inputs.velocityMetersPerSec =
        elevatorMotors[0].getVelocity().getValue() * ELEVATOR.encoderCountsToMeters * 10;
    inputs.heightEncoderCounts = elevatorMotors[0].getPosition().getValue();
    inputs.heightMeters = inputs.heightEncoderCounts * ELEVATOR.encoderCountsToMeters;
    inputs.outputVoltage = elevatorMotors[0].getSupplyVoltage().getValue();
    inputs.outputCurrent = elevatorMotors[0].getStatorCurrent().getValue();
    inputs.simEncoderSign = elevatorMotors[0].getInverted() ? -1 : 1;
    inputs.neutralMode = m_neutralMode.name();
  }

  private void initElevatorMotorFollower() {
    // if (DriverStation.isDisabled() && m_elevatorInitialized) {
    //   elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());
    //   elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);

    //   if (elevatorMotors[1].getControlMode() == ControlMode.Follower) m_elevatorInitialized =
    // true;
    // }
  }

  // Setting the raw output of the motors
  //  public void setPercentOutput(double output) {
  //    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  //  }

  // Sets the calculated trapezoid state of the motor
  public void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    // TODO: Find out why feedforward is no longer needed?
    //    elevatorMotors[0].set(
    //        TalonFXControlMode.Position,
    //        state.position / ELEVATOR.encoderCountsToMeters,
    //        DemandType.ArbitraryFeedForward,
    //        //        calculateFeedforward(state)
    //        0);
    //    elevatorMotors[1].set(ControlMode.PercentOutput,
    // elevatorMotors[0].getMotorOutputPercent());
  }

  // Sets the perceived position of the motors
  // Usually used to zero the motors if the robot is started in a non-stowed position
  public void setSensorPosition(double meters) {
    //    elevatorMotors[0].setSelectedSensorPosition(meters / ELEVATOR.encoderCountsToMeters);
  }

  public void setNeutralMode(NeutralModeValue mode) {
    //    m_neutralMode = mode;
    //    elevatorMotors[0].setNeutralMode(mode);
    //    elevatorMotors[1].setNeutralMode(mode);
  }

  public void setPIDvalues(double f, double p, double i, double d, double iZone) {
    //    elevatorMotors[0].config_kF(ELEVATOR.kSlotIdx, f);
    //    elevatorMotors[0].config_kP(ELEVATOR.kSlotIdx, p);
    //    elevatorMotors[0].config_kI(ELEVATOR.kSlotIdx, i);
    //    elevatorMotors[0].config_kD(ELEVATOR.kSlotIdx, d);
    //    elevatorMotors[0].config_IntegralZone(ELEVATOR.kSlotIdx, iZone);
  }
}
