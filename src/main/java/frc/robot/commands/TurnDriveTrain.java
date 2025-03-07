// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public class TurnDriveTrain extends Command {
  private final double kMaxVelocity = 90.0;
  private final double kMaxAcceleration = 900.0;
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kDt = 0.02;

//Declares varible names and creates timer object
  private final DriveTrainSubsystem m_robotDrive;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private double relAngle;
  private double goalAngle;

  public TurnDriveTrain(double relAngle, DriveTrainSubsystem driveTrainSubsystem) {
    this.relAngle = relAngle;
    m_robotDrive = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

//When the command is first called, reset and start the timer 
  @Override
  public void initialize() {
    // convert relative angle in degrees to goal in number of rotations
    Rotation2d newRot = new Rotation2d(relAngle*Math.PI/180.0);
    newRot = newRot.rotateBy(newRot);
    goalAngle = newRot.getRotations();
    m_controller.setGoal(newRot.getRotations());
  }

//Each loop will update the val variable and it also sets the motors on the robot to drive backwards
  @Override
  public void execute() {
    m_robotDrive.drive(
              0.0,
              0.0,
              m_controller.calculate(m_robotDrive.getRotations()),
              false);
  }

//When command ends, set motor values to zero - Fun fact, I had this set to -0.5 rather than zero which explains why the robot would always drive forward towards the end of autonomous
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(
              0.0,
              0.0,
              0.0,
              false);
  }

//Command will end after 4 seconds
  @Override
  public boolean isFinished() {
    return Math.abs(m_robotDrive.getRotations() - goalAngle) < 1.0/360.0;
  }
}