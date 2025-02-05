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


public class LiftSubsystem extends SubsystemBase {
  private final SparkMax m_testMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopController = m_testMotor.getClosedLoopController();
  private final RelativeEncoder encoder = m_testMotor.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();


  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    encoder.setPosition(0);
    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default to slot 0.
      .p(1.0)
      .i(0)
      .d(2.0)
      .outputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_testMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);


  }

  public Command goToPosition() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          closedLoopController.setReference(10.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
          System.out.println("X was pressed");
        });
  }

  public Command encoderReset() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
          encoder.setPosition(0);
          closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
          encoder.setPosition(0);
          System.out.println("Y was pressed");
        });
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
