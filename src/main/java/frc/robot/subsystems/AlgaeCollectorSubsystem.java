// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class AlgaeCollectorSubsystem extends SubsystemBase {
  private final SparkMax m_grabberMotor1 = new SparkMax(SubsystemConstants.kGrabber1Channel, MotorType.kBrushless);
  // private final SparkMax m_grabberMotor2 = new SparkMax(SubsystemConstants.kGrabber2Channel, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopControllerRight = m_grabberMotor1.getClosedLoopController();
  private final RelativeEncoder rightEncoder = m_grabberMotor1.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  /** Creates a new ExampleSubsystem. */
  public AlgaeCollectorSubsystem() {
    rightEncoder.setPosition(0);
    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(0.08)
      .i(0)
      .d(0.0)
      .outputRange(-1, 1);
      // Set PID values for velocity control in slot 1
      // .p(0.0001, ClosedLoopSlot.kSlot1)
      // .i(0, ClosedLoopSlot.kSlot1)
      // .d(0, ClosedLoopSlot.kSlot1)
      // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .maxVelocity(1000)
      .maxAcceleration(25000)
      .allowedClosedLoopError(1);

    motorConfig.smartCurrentLimit(40);
    m_grabberMotor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // m_grabberMotor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command collect() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          closedLoopControllerRight.setReference(-100.0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
          // m_grabberMotor1.set(1.0);
          // m_grabberMotor2.set(-1.0);
          System.out.println("collect");
        });
  }

  public Command release() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          closedLoopControllerRight.setReference(100.0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
          // m_grabberMotor1.set(-1.0);
          // m_grabberMotor2.set(1.0);
          System.out.println("release");
        });
  }

  public Command stopMotor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // m_grabberMotor1.set(0);
          // m_grabberMotor2.set(0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
