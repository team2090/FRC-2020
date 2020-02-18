/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class StateMachineCommand extends CommandBase {
  private final SwerveSubsystem robotDrive;
  private final ShooterSubsystem robotShooter;
  private final IntakeSubsystem robotIntake;
  private final HangSubsystem robotHang;
  /**
   * Creates a new StateMachineCommand.
   */
  public StateMachineCommand(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake, HangSubsystem hang) {
    robotDrive = swerve;
    robotShooter = shooter;
    robotIntake = intake;
    robotHang = hang;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive, robotShooter, robotIntake, robotHang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
