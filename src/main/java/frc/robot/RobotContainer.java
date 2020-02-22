/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem robotDrive = new SwerveSubsystem();
  private final HangSubsystem hang = new HangSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final AutoDriveCommand autoCommand = new AutoDriveCommand(robotDrive, shooter, intake);
  private final DriveControls controls = new DriveControls();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //robotDrive.setDefaultCommand(new TeleOpDriveCommand(robotDrive));
    
    robotDrive.setDefaultCommand(
      new RunCommand(() -> robotDrive.drive(
          modifyInput(controls.getForward()),
          modifyInput(controls.getStrafe()),
          modifyInput(controls.getYaw())), robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controls.zeroAzimuthPosition.whenPressed(new InstantCommand(() -> robotDrive.zero()));
    controls.setRobotOriented.whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)));
    controls.setFieldOriented.whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(true)));
    controls.robotOrientedForward.whenPressed(new InstantCommand(() -> robotDrive.drive(0.2, 0, 0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }
  
  public double modifyInput(double value) {
    // Deadband
    if (Math.abs(value) < 0.1) {
      return 0;
    }

    // If slow drive mode is enabled
    if (controls.slowDriveMode()) {
      return value / 2;
    } else {
      return value;
    }
  }

  public SwerveSubsystem getSwerve() {
    return robotDrive;
  }
}
