/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class Wheel {
    private final double offsetAngle;
    private final CANSparkMax driveMotor;
    private final CANSparkMax azimuthMotor;
    //private final azimuthPIDSubsystem azimuthPIDController;
    private final CANPIDController drivePIDController;
    private final CANEncoder driveEncoder;
    private final CANPIDController azimuthSparkPID;
    private final AnalogInput azimuthEncoder;

    public Wheel(int azimuthMotorId, int driveMotorId, int encoderChannel, int offset) {
        this.azimuthMotor = new CANSparkMax(azimuthMotorId, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.azimuthSparkPID = azimuthMotor.getPIDController();
        this.azimuthEncoder = new AnalogInput(encoderChannel);
        this.drivePIDController = driveMotor.getPIDController();
        this.driveEncoder = driveMotor.getEncoder();
        this.offsetAngle = offset;
        //this.azimuthPIDController = new azimuthPIDSubsystem(Constants.azimuthkP, Constants.azimuthkI, Constants.azimuthkD, azimuthMotor, encoderChannel);
    }

    public void initWheel() {
        azimuthMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        drivePIDController.setP(Constants.drivekP);
        drivePIDController.setI(Constants.drivekI);
        drivePIDController.setD(Constants.drivekD);
        drivePIDController.setIZone(Constants.drivekIz);
        drivePIDController.setFF(Constants.drivekFF);
        drivePIDController.setOutputRange(Constants.drivekMinOutput, Constants.drivekMaxOutput);

        drivePIDController.setSmartMotionMaxVelocity(Constants.maxVel, 0);
        //drivePIDController.setSmartMotionMinOutputVelocity(Constants.minVel, 0);
        drivePIDController.setSmartMotionMaxAccel(Constants.maxAcc, 0);
        //drivePIDController.setSmartMotionAllowedClosedLoopError(Constants.allowedErr, 0);

        // set PID coefficients
        azimuthSparkPID.setP(Constants.azimuthkP);
        azimuthSparkPID.setI(Constants.azimuthkI);
        azimuthSparkPID.setD(Constants.azimuthkD);
        azimuthSparkPID.setIZone(Constants.azimuthkIz);
        azimuthSparkPID.setFF(Constants.azimuthkFF);
        azimuthSparkPID.setOutputRange(Constants.azimuthkMinOutput, Constants.azimuthkMaxOutput);
        azimuthSparkPID.setSmartMotionMaxVelocity(Constants.maxVel, 0);
        //azimuthSparkPID.setSmartMotionMinOutputVelocity(Constants.minVel, 0);
        azimuthSparkPID.setSmartMotionMaxAccel(Constants.maxAcc, 0);
        azimuthEncoder.setOversampleBits(4);
        //azimuthPIDController.enable();
    }

    public void setTargetAngle(double angle) {
        double currentPosition = ((azimuthEncoder.getValue() * 360.0 / 4049.0));
        currentPosition =  currentPosition > 180.0 ? currentPosition - 360.0 : currentPosition;
        currentPosition = currentPosition == 360 ? 0 : currentPosition; // Making 360 deg and 0 deg equal
        angle *= 360; // flip azimuth, hardware configuration dependent

        double azimuthError = Math.IEEEremainder(currentPosition - angle, 360.0);
        azimuthSparkPID.setReference(azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition() + (offsetAngle / 360.0 * 18), ControlType.kSmartMotion);
    }

    public void setTargetDistance(double setpoint) {
        drivePIDController.setReference(setpoint, ControlType.kVelocity);
        SmartDashboard.putNumber("Drive target velocity", setpoint);
        SmartDashboard.putNumber("Drive encoder velocity", driveEncoder.getPosition());
    } 

    public void setDriveOutput(double output) {
        drivePIDController.setReference(output * Constants.maxVel, ControlType.kVelocity);
        SmartDashboard.putNumber("Drive target velocity", output * Constants.maxVel);
        SmartDashboard.putNumber("Drive encoder velocity", driveEncoder.getVelocity());
    }
    
    public void set(double targetAngle, double drive) {
        
        double currentPosition = ((azimuthEncoder.getValue() * 360.0 / 4049.0));
        currentPosition =  currentPosition > 180.0 ? currentPosition - 360.0 : currentPosition;
        currentPosition = currentPosition == 360 ? 0 : currentPosition; // Making 360 deg and 0 deg equal
        targetAngle *= 360; // flip azimuth, hardware configuration dependent

        double azimuthError = Math.IEEEremainder(currentPosition - targetAngle, 360.0);

        // minimize azimuth rotation, reversing drive if necessary
        if (Math.abs(azimuthError) > 0.25 * 360) {
            azimuthError -= Math.copySign(0.5 * 360, azimuthError);
            drive = -drive;
             SmartDashboard.putBoolean("Direction", false);
        }   
       
        SmartDashboard.putNumber("Current Encoder Position (Encoder Units)", azimuthEncoder.getValue());
        SmartDashboard.putNumber("Target Change", azimuthError);
        SmartDashboard.putNumber("Current Spark Position", azimuthMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Target Spark Position (Encoder units)", (azimuthError / 360.0 * 18) + azimuthMotor.getEncoder().getPosition());

        if (drive != 0) {
            azimuthSparkPID.setReference(azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition() + (offsetAngle / 360.0 * 18), ControlType.kSmartMotion);
        }
        
        setDriveOutput(drive);
    
    }

    public void zero() {
        setTargetAngle(offsetAngle);
        setDriveOutput(0);
    }

    public void stop() {
        azimuthEncoder.close();
        azimuthMotor.set(0);
        driveMotor.set(0);
    }
}