// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.TimedDriveForwards;
import frc.robot.commands.InitLift;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final DriveTrainSubsystem m_robotDrive = new DriveTrainSubsystem();
  // private final HangerSubsystem m_robotHanger = new HangerSubsystem();
  private final AlgaeCollectorSubsystem m_algaSubsystem = new AlgaeCollectorSubsystem();
  private final LiftSubsystem m_robotLift = new LiftSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
              -m_driverController.getLeftY(),
              m_driverController.getLeftX(),
              m_driverController.getRightX(),
              false),
               m_robotDrive));


    m_robotDrive.setMaxOutput(0.3);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  /*private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }*/

  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.3)));

        new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.3)));

        // new JoystickButton(m_driverController, Button.kA.value)
        // .onTrue(m_robotHanger.startMotor())
        // .onFalse(m_robotHanger.stopMotor());

        // new JoystickButton(m_driverController, Button.kB.value)
        // .onTrue(m_robotHanger.reverseMotor())
        // .onFalse(m_robotHanger.stopMotor());

        new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_robotLift.goToPosition0());

        new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_robotLift.goToPosition1());

        new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_robotLift.goToPosition2());

        new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(m_robotLift.goToPosition3());

        new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(m_robotLift.encoderReset());

        new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new RunCommand(
          () ->
              m_robotDrive.drive(
                  0.0,
                  0.05*m_vision.gettx(),
                  -0.0*m_vision.gettx(),
                  false),
          m_robotDrive))
          .onFalse(new RunCommand(
            () ->
                m_robotDrive.drive(
              -m_driverController.getLeftY(),
              m_driverController.getLeftX(),
              m_driverController.getRightX(),
              false),
               m_robotDrive));



        // new JoystickButton(m_driverController, Button.kStart.value)
        // .onTrue(runOnce(
        //   () -> {m_robotDrive.zeroHeading();}));


        // new JoystickButton(m_driverController, Button.kX.value)
        // .onTrue(m_algaSubsystem.collect())
        // .onFalse(m_algaSubsystem.stopMotor());

        // new JoystickButton(m_driverController, Button.kY.value)
        // .onTrue(m_algaSubsystem.release())
        // .onFalse(m_algaSubsystem.stopMotor());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new TimedDriveForwards(1.0, 0.3, m_robotDrive),
      new InitLift(m_robotLift)
    );
  }
}
