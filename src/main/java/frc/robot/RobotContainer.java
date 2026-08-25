// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;

import frc.robot.subsystems.Input;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

    private final Joystick joy;
    private final GenericHID bBoard;   
    private final CommandXboxController xboxController;
    private final Input input;
    private final SwerveDriveSubsystem drive;

    

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SendableChooser<Command> autonChooser) {
    bBoard = new GenericHID(OperatorConstants.BUTTON_BOARD_PORT);
    joy = new Joystick(OperatorConstants.JOYSTICK_PORT);
    xboxController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER_PORT);
    input = new Input(joy, xboxController.getHID());
    drive = new SwerveDriveSubsystem();

    autonChooser.setDefaultOption("Nothing", null);

    SmartDashboard.putData(autonChooser);

    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {

    drive.setDefaultCommand(new DriveCommand(drive, input, true));

    // D-pad up/down changes the Xbox controller's drive speed percentage.
    xboxController.povUp().onTrue(new InstantCommand(input::incrementTranslationControllerSpeed));
    xboxController.povDown().onTrue(new InstantCommand(input::decrementTranslationControllerSpeed));
    xboxController.povRight().onTrue(new InstantCommand(input::incrementTwistControllerSpeed));
    xboxController.povLeft().onTrue(new InstantCommand(input::decrementTwistControllerSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {

    return autonChooser.getSelected(); 
  }
}
