/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SubsystemStateMachine extends CommandBase {
  private final SwerveSubsystem drive;
  private final HangSubsystem hang;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private boolean driveRunning = false;
  /**
   * Creates a new SubsystemStateMachine.
   */
  public SubsystemStateMachine(SwerveSubsystem driveSubsystem, HangSubsystem hangSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, hangSubsystem, intakeSubsystem, shooterSubsystem);

    drive = driveSubsystem;
    hang = hangSubsystem;
    shooter = shooterSubsystem;
    intake = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String stateInput = "";
    // Check if subsystem is suppose to run
    if (Robot.controls.getForward() != 0 || Robot.controls.getStrafe() != 0 || Robot.controls.getYaw() != 0) {
      stateInput += "1";
    } else {
      stateInput += "0";
    }

    // Manage behavior based on running status
    switch(stateInput) {
      default:
        SmartDashboard.putString("State Machine", "Current output sufficient");
    }

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
