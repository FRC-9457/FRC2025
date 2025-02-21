// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class AlgaeCollectorSubsystem extends SubsystemBase {
  private final PWMSparkMax m_grabberMotor1 = new PWMSparkMax(SubsystemConstants.kGrabber1Channel);
  private final PWMSparkMax m_grabberMotor2 = new PWMSparkMax(SubsystemConstants.kGrabber2Channel);
  /** Creates a new ExampleSubsystem. */
  public AlgaeCollectorSubsystem() {}

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
          m_grabberMotor1.set(1.0);
          m_grabberMotor2.set(-1.0);
          System.out.println("collect");
        });
  }

  public Command release() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_grabberMotor1.set(-1.0);
          m_grabberMotor2.set(1.0);
          System.out.println("release");
        });
  }

  public Command stopMotor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_grabberMotor1.set(0);
          m_grabberMotor2.set(0);
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
