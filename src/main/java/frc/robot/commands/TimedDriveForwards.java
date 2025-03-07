// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class TimedDriveForwards extends Command {
//Declares varible names and creates timer object
  private Timer driveTimer = new Timer();
  private final DriveTrainSubsystem m_robotDrive;

  public TimedDriveForwards(DriveTrainSubsystem driveTrainSubsystem) {
//Requires these subsystems
    m_robotDrive = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

//When the command is first called, reset and start the timer 
  @Override
  public void initialize() {
    driveTimer.reset();
    driveTimer.start();
  }

//Each loop will update the val variable and it also sets the motors on the robot to drive backwards
  @Override
  public void execute() {
    m_robotDrive.drive(
              0.3,
              0.0,
              0.0,
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
    return driveTimer.get() > 5.0;
  }
}