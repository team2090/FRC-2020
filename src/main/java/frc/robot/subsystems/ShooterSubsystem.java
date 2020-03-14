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

/**
 * The Shooter subsystem uses thre SparkMax motors as the shooter and ball storage motors
 * A TalonSRX is used as the intake motors
 */
public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private CANPIDController shooterPIDController;
  private CANPIDController ballStoragePIDController;
  private CANEncoder shooterEncoder;
  private TalonSRX intakeMotor;
  private CANSparkMax ballStorage;
  private DoubleSolenoid intakeRelease;

  /**
   * This constructs the Shooter Subsystem with the navx and given constants 
   * including the ratio of the robot length to width. 
   */
  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(shooter1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooter2, MotorType.kBrushless);
    intakeMotor = new TalonSRX(intakeMotorId);
    ballStorage = new CANSparkMax(storageMotorId, MotorType.kBrushless);
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
   
    // Set PID coefficients
    shooterPIDController.setP(shooterkP);
    shooterPIDController.setI(shooterkI);
    shooterPIDController.setD(shooterkD);
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
  }

  /**
   * Launch the balls by setting the shooter to a given target velocity
   * @param setPosition the array position of desired target velocity
   */
  public void launchBall(int setPosition) {
    shooterPIDController.setReference(targetVelocities[setPosition], ControlType.kVelocity);
    SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Output", shooterMotor1.getAppliedOutput());
  }

  /**
   * Lower and run the intake
   */
  public void runIntake() {
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(ControlMode.PercentOutput, 0.35);
  }

  /**
   * Lower and run the intake backwards
   */
  public void runIntakeBackwards() {
    intakeRelease.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(ControlMode.PercentOutput, -0.35);
  }

  /**
   * Runs the ball storage 
   */
  public void runBallStorage() {
     ballStorage.set(0.8);
  }

  /**
   * Runs the ball storage backwards
   */
  public void runBallStorageBackwards() {
    ballStorage.set(-0.8);
 }

  /**
   * Lift the intake up
   */
  public void intakeUp() {
    intakeRelease.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Default behavior of the shooter subsystem
   * Stops all motors 
   */
  public void stop() {
    shooterMotor1.set(0);
    ballStorage.set(0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
