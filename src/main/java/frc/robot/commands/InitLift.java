// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class InitLift extends Command {
//Declares varible names and creates timer object
  private Timer driveTimer = new Timer();
  private final LiftSubsystem m_robotLift;

  public InitLift(LiftSubsystem liftSubsystem) {
//Requires these subsystems
    m_robotLift = liftSubsystem;
    addRequirements(liftSubsystem);
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
    m_robotLift.goToInitPosition();
  }

//When command ends, set motor values to zero - Fun fact, I had this set to -0.5 rather than zero which explains why the robot would always drive forward towards the end of autonomous
  @Override
  public void end(boolean interrupted) {
  }

//Command will end after 4 seconds
  @Override
  public boolean isFinished() {
    return m_robotLift.isAtPosition();
  }
}