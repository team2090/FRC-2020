/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    // shooterMotor1 = new CANSparkMax(shooter1, MotorType.kBrushless);
    // shooterMotor2 = new CANSparkMax(shooter2, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
