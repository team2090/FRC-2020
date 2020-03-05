/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private CANPIDController shooterPIDController;
  private CANEncoder shooterEncoder;
  private TalonSRX intakeMotor;
  private TalonSRX ballStorage;
  private double[] targetVelocities = {0.5, 0.8, 1.0};
  private DoubleSolenoid ballHolder;
  private DoubleSolenoid intakeRelease;

  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(shooter1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooter2, MotorType.kBrushless);
    intakeMotor = new TalonSRX(intakeMotorId);
    ballStorage = new TalonSRX(storageMotorId);
    ballHolder = new DoubleSolenoid(ballHolderForwardChannel, ballHolderReverseChannel);
    intakeRelease = new DoubleSolenoid(intakeReleaseForwardChannel, intakeReleaseReverseChannel);
    
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(true);

    shooterPIDController = shooterMotor1.getPIDController();
    shooterEncoder = shooterMotor1.getEncoder(); // Encoder object created to display velocity values

    // set PID coefficients
    shooterPIDController.setP(shooterkP);
    shooterPIDController.setI(shooterkI);
    shooterPIDController.setD(shooterkD);
    shooterPIDController.setIZone(shooterkIz);
    shooterPIDController.setFF(shooterkFF);
    shooterPIDController.setOutputRange(shooterkMinOutput, shooterkMaxOutput);

    shooterMotor2.follow(shooterMotor1);
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    ballHolder.set(DoubleSolenoid.Value.kForward);
  }

  public void launchBall(int setPosition) {
    shooterMotor1.set(0.8);
    //shooterPIDController.setReference(targetVelocities[setPosition] * shooterMaxRPM, ControlType.kVelocity);
    // if (Math.abs(shooterEncoder.getVelocity() - (targetVelocities[setPosition] * shooterMaxRPM)) < 100) {
    //   ballHolder.set(DoubleSolenoid.Value.kReverse);
    //   ballStorage.set(ControlMode.PercentOutput, 0.5);
    // }
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
  }

  public void runIntake() {
    intakeRelease.set(DoubleSolenoid.Value.kReverse);
    intakeMotor.set(ControlMode.PercentOutput, 0.2);
    SmartDashboard.putBoolean("Intake", true);
  }
  public void runBallStorage() {
     // TODO: ADD DETECTION HERE
     ballStorage.set(ControlMode.PercentOutput, 0.6);
  }


  public void stop() {
    shooterMotor1.set(0);
    //SmartDashboard.putNumber("SPEED!", shooterEncoder.getVelocity());
    ballStorage.set(ControlMode.PercentOutput, 0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    ballHolder.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putBoolean("Intake", false);
  }
}
