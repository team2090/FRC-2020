/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private CANPIDController shooterPIDController;
  private CANEncoder shooterEncoder;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    // shooterMotor1 = new CANSparkMax(shooter1, MotorType.kBrushless);
    // shooterMotor2 = new CANSparkMax(shooter2, MotorType.kBrushless);

    // shooterMotor1.restoreFactoryDefaults();
    // shooterMotor2.restoreFactoryDefaults();

    // shooterPIDController = shooterMotor1.getPIDController();

    // // Encoder object created to display position values
    // shooterEncoder = shooterMotor1.getEncoder();

    // // set PID coefficients
    // shooterPIDController.setP(shooterkP);
    // shooterPIDController.setI(shooterkI);
    // shooterPIDController.setD(shooterkD);
    // shooterPIDController.setIZone(shooterkIz);
    // shooterPIDController.setFF(shooterkFF);
    // shooterPIDController.setOutputRange(shooterkMinOutput, shooterkMaxOutput);

    // shooterMotor2.follow(shooterMotor1);


  }

  public void launch(double setpoint) {
    shooterPIDController.setReference(setpoint * shooterMaxRPM, ControlType.kVelocity);
  }
}
