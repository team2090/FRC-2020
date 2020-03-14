/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoDriveCommandLeft;
import frc.robot.commands.AutoDriveCommandMid;
import frc.robot.commands.AutoDriveCommandRight;
import frc.robot.subsystems.HangSubsystem;
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
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final AutoDriveCommandLeft autoLeftCommand = new AutoDriveCommandLeft(robotDrive, shooter);
  private final AutoDriveCommandMid autoMidCommand = new AutoDriveCommandMid(robotDrive, shooter);
  private final AutoDriveCommandRight autoRightCommand = new AutoDriveCommandRight(robotDrive, shooter);
  private final DriveControls controls = new DriveControls();
  private double speedMod = 1.0;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    robotDrive.setDefaultCommand(
      new RunCommand(() -> robotDrive.drive(
          modifyInput(controls.getForward()),
          modifyInput(controls.getStrafe()),
          modifyInput(controls.getYaw())), robotDrive));
    
    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.stop(), shooter)
    );
    hang.setDefaultCommand(
      new RunCommand(() -> hang.holdPosition(), hang)
    );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controls.zeroField.whenPressed(new InstantCommand(() -> robotDrive.initDrive()));
    controls.setRobotOriented.whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)));
    controls.setFieldOriented.whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(true)));

    controls.normalDriveMode.whenPressed(new InstantCommand(() -> speedMod = 1.0));
    controls.slowDriveMode.whenPressed(new InstantCommand(() -> speedMod = 0.5));
    controls.precisionDriveMode.whenPressed(new InstantCommand(() -> speedMod = 0.25));
    
    controls.robotOrientedForward
      .whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)))
      .whileHeld(new RunCommand(() -> robotDrive.drive(0.2, 0, 0)))
      .whenReleased(new InstantCommand(() -> robotDrive.setFieldOriented(true)));
    controls.robotOrientedBackwards
      .whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)))
      .whileHeld(new RunCommand(() -> robotDrive.drive(-0.2, 0, 0)))
      .whenReleased(new InstantCommand(() -> robotDrive.setFieldOriented(true)));
    controls.robotOrientedRight
      .whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)))
      .whileHeld(new RunCommand(() -> robotDrive.drive(0, 0.2, 0)))
      .whenReleased(new InstantCommand(() -> robotDrive.setFieldOriented(true)));
    controls.robotOrientedLeft
      .whenPressed(new InstantCommand(() -> robotDrive.setFieldOriented(false)))
      .whileHeld(new RunCommand(() -> robotDrive.drive(0, -0.2, 0)))
      .whenReleased(new InstantCommand(() -> robotDrive.setFieldOriented(true)));
    controls.limelightAim.whileHeld(new RunCommand(() -> robotDrive.updateLimelightTracking(
      modifyInput(controls.getForward()),
			modifyInput(controls.getStrafe())))).whenReleased(new InstantCommand(() -> robotDrive.setCamMode(false)));
			
    controls.intake.whileHeld(new RunCommand(() -> shooter.runIntake()));
    controls.ballStorage.whileHeld(new RunCommand(() -> shooter.runBallStorage()));
    controls.backwardsBallStorage.whileHeld(new RunCommand(() -> shooter.runBallStorageBackwards()));
    controls.liftIntake.whileHeld(new RunCommand(() -> shooter.intakeUp()));
    controls.reverseIntake.whileHeld(new RunCommand(() -> shooter.runIntakeBackwards()));
    controls.shootBallLow
      .whileHeld(new RunCommand(() -> shooter.launchBall(0)));
    controls.shootBallMid
      .whileHeld(new RunCommand(() -> shooter.launchBall(1)));
    controls.shootBallHigh
      .whileHeld(new RunCommand(() -> shooter.launchBall(2)));
    
    controls.hangUp.whileHeld(new RunCommand(() -> hang.lift()));
    controls.hangDown.whileHeld(new RunCommand(() -> hang.down()));
  }
  
  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */
  public Command getAutonomousCommandLeft() {
    return autoLeftCommand;
  }
  
  public Command getAutonomousCommandMid() {
    return autoMidCommand;
  }

  public Command getAutonomousCommandRight() {
    return autoRightCommand;
  }

  public double modifyInput(double value) {
    // Deadband
    if (Math.abs(value) < 0.1) {
      return 0;
    }
		// Modify the inputed speed based on which speed mode is currently active
    return value * speedMod;
  }

  public SwerveSubsystem getSwerve() {
    return robotDrive;
  }
}
