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
  private CANPIDController ballStoragePIDController;
  private CANEncoder shooterEncoder;
  private TalonSRX intakeMotor;
  private CANSparkMax ballStorage;
  private double[] targetVelocities = {0.5, 0.8, 1.0};
  private DoubleSolenoid ballShifter;
  private DoubleSolenoid intakeRelease;

  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(shooter1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooter2, MotorType.kBrushless);
    intakeMotor = new TalonSRX(intakeMotorId);
    ballStorage = new CANSparkMax(storageMotorId, MotorType.kBrushless);
    ballShifter = new DoubleSolenoid(ballShifterForwardChannel, ballShifterReverseChannel);
    intakeRelease = new DoubleSolenoid(intakeReleaseForwardChannel, intakeReleaseReverseChannel);
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();
    ballStorage.restoreFactoryDefaults();

    shooterMotor1.setInverted(true);

    ballStoragePIDController = ballStorage.getPIDController();
    shooterPIDController = shooterMotor1.getPIDController();
    shooterEncoder = shooterMotor1.getEncoder(); // Encoder object created to display velocity values
    
    // set PID coefficients
    shooterPIDController.setP(0.01);
    shooterPIDController.setI(0);
    shooterPIDController.setD(0);
    shooterPIDController.setIZone(shooterkIz);
    shooterPIDController.setFF(shooterkFF);
    shooterPIDController.setOutputRange(shooterkMinOutput, shooterkMaxOutput);
    shooterMotor2.follow(shooterMotor1, true);

    ballStoragePIDController.setP(ballStoragekP);
    ballStoragePIDController.setI(ballStoragekI);
    ballStoragePIDController.setD(ballStoragekD);
    ballStoragePIDController.setIZone(ballStoragekIz);
    ballStoragePIDController.setFF(ballStoragekFF);
    ballStoragePIDController.setOutputRange(ballStoragekMinOutput, ballStoragekMaxOutput);

    intakeRelease.set(DoubleSolenoid.Value.kForward);
    ballShifter.set(DoubleSolenoid.Value.kForward);
  }

  public void launchBall(int setPosition) {
    //shooterMotor1.set(0.62);
    shooterPIDController.setReference(targetVelocities[setPosition] * shooterMaxRPM, ControlType.kVelocity);
    // if (Math.abs(shooterEncoder.getVelocity() - (targetVelocities[setPosition] * shooterMaxRPM)) < 100) {
    //   ballShifter.set(DoubleSolenoid.Value.kReverse);
    //   ballStorage.set(ControlMode.PercentOutput, 0.5);
    // }
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Output", shooterMotor1.getAppliedOutput());
  }

  public void runIntake() {
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(ControlMode.PercentOutput, 0.55);
  }

  public void runIntakeBackwards() {
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(ControlMode.PercentOutput, -0.55);
  }

  public void runBallStorage() {
     ballStorage.set(0.6);
  }

  public void runBallStorageBackwards() {
    ballStorage.set(-0.6);
 }

 public void intakeDown() {
  intakeRelease.set(DoubleSolenoid.Value.kReverse);
 }

  public void stop() {
    shooterPIDController.setReference(targetVelocities[0] * shooterMaxRPM, ControlType.kVelocity);
    ballStorage.set(0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    ballShifter.set(DoubleSolenoid.Value.kForward);
  
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Output", shooterMotor1.getAppliedOutput());
  }
}
