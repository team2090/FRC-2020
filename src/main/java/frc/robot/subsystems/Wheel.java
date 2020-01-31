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
 * Controls a swerve drive wheel azimuth and drive motors.
 *
 * <p>The swerve-drive inverse kinematics algorithm will always calculate individual wheel angles in degrees 
 * with the zero being the straight ahead position based on the measurements of the absolute encoder. 
 * Wheel speed is calculated as 0 to 1 in the direction of the wheel angle.
 *
 * <p>This class will calculate how to implement this angle and drive direction optimally for the
 * azimuth and drive motors. In some cases it makes sense to reverse wheel direction to avoid
 * rotating the wheel azimuth 180 degrees.
 *
 * <p>Hardware assumed by this class includes a MA3 Encoder as an analog input and no
 * limits on wheel azimuth rotation. Azimuth Motors have an ID in the range 1-4 with corresponding
 * drive Falcon IDs in the range 11-14.
 */
public class Wheel {
    private final double offsetAngle;
    private final CANSparkMax driveMotor;
    private final CANSparkMax azimuthMotor;
    private final CANPIDController azimuthPIDController;
    private final AnalogInput azimuthEncoder;
    private final CANPIDController drivePIDController;
    private final CANEncoder driveEncoder;
    
    /**
     * This constructs a wheel with supplied azimuth spark, drive falcon, and MA3 encoder.
     *
     * @param azimuthMotorId the azimuth SparkMax
     * @param driveMotorId the drive Falcon 500
     * @param encoderChannel the MA3 Analog encoder port
     * @param offset the offset to get correct zero position (in deg)
     */
    public Wheel(int azimuthMotorId, int driveMotorId, int encoderChannel, double offset) {
        this.azimuthMotor = new CANSparkMax(azimuthMotorId, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.azimuthPIDController = azimuthMotor.getPIDController();
        this.azimuthEncoder = new AnalogInput(encoderChannel);
        this.drivePIDController = driveMotor.getPIDController();
        this.driveEncoder = driveMotor.getEncoder();
        this.offsetAngle = offset;
    }

    /**
     * Configures the drive and azimuth motors with supplied PID constants.
     */
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
        drivePIDController.setSmartMotionMinOutputVelocity(Constants.minVel, 0);
        drivePIDController.setSmartMotionMaxAccel(Constants.maxAcc, 0);
        //drivePIDController.setSmartMotionAllowedClosedLoopError(Constants.allowedErr, 0);

        // set PID coefficients
        azimuthPIDController.setP(Constants.azimuthkP);
        azimuthPIDController.setI(Constants.azimuthkI);
        azimuthPIDController.setD(Constants.azimuthkD);
        azimuthPIDController.setIZone(Constants.azimuthkIz);
        azimuthPIDController.setFF(Constants.azimuthkFF);
        azimuthPIDController.setOutputRange(Constants.azimuthkMinOutput, Constants.azimuthkMaxOutput);
        azimuthPIDController.setSmartMotionMaxVelocity(Constants.maxVel, 0);
        //azimuthPIDController.setSmartMotionMinOutputVelocity(Constants.minVel, 0);
        azimuthPIDController.setSmartMotionMaxAccel(Constants.maxAcc, 0);
        azimuthEncoder.setOversampleBits(4);
    }

    /**
     * Set the azimuth to the absolute angular position.
     * 
     * @param angle position in degrees
     */
    public void setTargetAngle(double angle) {
        double currentPosition = ((azimuthEncoder.getValue() * 360.0 / 4049.0));
        currentPosition =  currentPosition > 180.0 ? currentPosition - 360.0 : currentPosition;
        currentPosition = currentPosition == 360 ? 0 : currentPosition; // Making 360 deg and 0 deg equal
        angle *= 360; // flip azimuth, hardware configuration dependent

        double azimuthError = Math.IEEEremainder(currentPosition - angle, 360.0);
        azimuthPIDController.setReference(azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition() + (offsetAngle / 360.0 * 18), ControlType.kSmartMotion);
    }

    /**
     * Set the drive to move to a given setpoint.
     * 
     * @param setpoint the position in meters
     */
    public void setTargetDistance(double setpoint) {
        drivePIDController.setReference(setpoint, ControlType.kVelocity);
        SmartDashboard.putNumber("Drive target velocity", setpoint);
        SmartDashboard.putNumber("Drive encoder velocity", driveEncoder.getPosition());
    } 

    /**
     * Set the drive to move a given velocity.
     * 
     * @param output target velocity 0 to 1.0
     */
    public void setDriveOutput(double output) {
        drivePIDController.setReference(output * Constants.maxVel, ControlType.kVelocity);
        SmartDashboard.putNumber("Drive target velocity", output * Constants.maxVel);
        SmartDashboard.putNumber("Drive encoder velocity", driveEncoder.getVelocity());
    }
    
    /**
     * This method calculates the optimal driveTalon settings and applies them.
     *
     * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the wheel's zeroed position
     * @param drive 0 to 1.0 in the direction of the wheel azimuth
     */
    public void set(double targetAngle, double drive) {
        /* The current position is found using the MA3 Absolute Encoder Value. The encoder has units of 4049 
         * and is fully one-to-one with the azimuth motor's rotation.
         * 
         * If the current position is greater than 180, the negative position is used
         * E.g. If current position = 300 deg, then the current position of -60 deg should be used
        */

        double currentPosition = ((azimuthEncoder.getValue() * 360.0 / 4049.0));
        currentPosition =  currentPosition > 180.0 ? currentPosition - 360.0 : currentPosition;
        currentPosition = currentPosition == 360 ? 0 : currentPosition; 
        targetAngle *= 360; 

        // The difference between the target angle and the current position 
        double azimuthError = Math.IEEEremainder(currentPosition - targetAngle, 360.0);

        // Minimize azimuth rotation by reversing drive if necessary
        if (Math.abs(azimuthError) > 0.25 * 360) {
            azimuthError -= Math.copySign(0.5 * 360, azimuthError);
            drive = -drive;
             SmartDashboard.putBoolean("Direction", false);
        }   
       
        SmartDashboard.putNumber("Current Encoder Position (Encoder Units)", azimuthEncoder.getValue());
        SmartDashboard.putNumber("Target Change", azimuthError);
        SmartDashboard.putNumber("Current Spark Position", azimuthMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Target Spark Position (Encoder units)", (azimuthError / 360.0 * 18) + azimuthMotor.getEncoder().getPosition());

        // Prevent the azimuth from resetting position to zero
        if (drive != 0) {
            // The azimuth motor has units of 18 for one full rotation. The position is set using the azimuth error + current position. 
            azimuthPIDController.setReference(azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition() + (offsetAngle / 360.0 * 18), ControlType.kSmartMotion);
        }
        setDriveOutput(drive);
    }

    /**
     * Set the drive output to 0 and azimuth to the zeroed position
     */
    public void zero() {
        setTargetAngle(offsetAngle);
        setDriveOutput(0);
    }

    /**
     * Close the analog input from the encoder and set all motor output to 0.
     */

    public void stop() {
        azimuthEncoder.close();
        azimuthMotor.set(0);
        driveMotor.set(0);
    }
}