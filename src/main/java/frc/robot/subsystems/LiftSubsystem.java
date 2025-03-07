// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.SubsystemConstants;


public class LiftSubsystem extends SubsystemBase {
  private final SparkMax m_rightMotor = new SparkMax(SubsystemConstants.kliftRightChannel, MotorType.kBrushless);
  private final SparkMax m_leftMotor = new SparkMax(SubsystemConstants.kliftLeftChannel, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopControllerRight = m_rightMotor.getClosedLoopController();
  private final SparkClosedLoopController closedLoopControllerLeft = m_leftMotor.getClosedLoopController();
  private final RelativeEncoder rightEncoder = m_rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = m_leftMotor.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final SparkMaxConfig motorConfigfollower = new SparkMaxConfig();

  private final double syncThresh = 5.0;
  private final double hardStopPosition = 130.0;
  private boolean isEnabled = true;
  private boolean isAtPosition = true;
  private double desiredPosition = 0.0;
  private double positionThresh = 10.0;
  
  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      motorConfigfollower.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);


    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(0.04)
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
      .maxVelocity(5000)
      .maxAcceleration(25000)
      .allowedClosedLoopError(1);

    motorConfig.smartCurrentLimit(80);

    m_rightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);


  }

  public Command goToPosition0() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(0.0);
          System.out.println("X was pressed");
        });
  }

  public Command goToPosition1() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(20.0);
          System.out.println("X was pressed");
        });
  }

  public Command goToPosition2() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(30.0);
          System.out.println("X was pressed");
        });
  }

  public Command goToPosition3() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(100.0);
          System.out.println("X was pressed");
        });
  }

  public Command goToInitPosition() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(20.0);
          System.out.println("going to init position");
        });
  }

  public Command encoderReset() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setPosition(0.0);
          rightEncoder.setPosition(0);
          leftEncoder.setPosition(0);
          System.out.println("Y was pressed");
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // prevent the lift from going past a software hardstop
    if (Math.abs(rightEncoder.getPosition()) > hardStopPosition) {
      m_rightMotor.set(0.0);
      m_leftMotor.set(0.0);
      setPosition(Math.abs(rightEncoder.getPosition()));
      System.out.printf("lift subsystem went past the software hardstop of %f!", hardStopPosition);
    }

    // check if we've reached the desired position
    if (Math.abs(Math.abs(rightEncoder.getPosition()) - desiredPosition) < positionThresh) {
      isAtPosition = true;
    }

    // stop all the motors if they are more than syncThresh out of sync
    if (Math.abs(Math.abs(rightEncoder.getPosition()) - Math.abs(leftEncoder.getPosition())) > syncThresh) {
      m_rightMotor.setVoltage(0.0);
      m_leftMotor.setVoltage(0.0);

      rightEncoder.setPosition(0);
      leftEncoder.setPosition(0);

      setPosition(0.0);

      rightEncoder.setPosition(0);
      leftEncoder.setPosition(0);

      m_rightMotor.setVoltage(0.0);
      m_leftMotor.setVoltage(0.0);
      isEnabled = false;
      System.out.println("lift motors out of sync! lift motors disabled");
    }
    if (rightEncoder.getPosition() > 10.0 && rightEncoder.getPosition() < 70.0) {
      System.out.printf("right %f %f left %f %f\n", rightEncoder.getPosition(), rightEncoder.getVelocity(), leftEncoder.getPosition(), leftEncoder.getVelocity());
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void setPosition(double rotations) {
    desiredPosition = rotations;
    if (isEnabled) {
      closedLoopControllerRight.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      closedLoopControllerLeft.setReference(-1.0*rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      isAtPosition = false;
    }
    else {
      System.out.println("Attempting to move lift but lift motors disabled!");
    }
  }

  public boolean isAtPosition() {
    return isAtPosition;
  }
}
