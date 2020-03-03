/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import static frc.robot.Constants.SwerveConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
    private final TalonFX driveMotor;
    private final CANSparkMax azimuthMotor;
    private final CANPIDController azimuthPIDController;
    private final AnalogInput azimuthEncoder;
    
    /**
     * This constructs a wheel with supplied azimuth spark, drive falcon, and MA3 encoder.
     *
     * @param azimuthMotorId the azimuth SparkMax
     * @param driveMotorId the drive Falcon 500
     * @param encoderChannel the MA3 Analog encoder port
     * @param offset the offset to get correct zero position (in deg)
     */
    public Wheel(int azimuthMotorId, int driveMotorId, int encoderChannel, double offset) {
        azimuthMotor = new CANSparkMax(azimuthMotorId, MotorType.kBrushless);
        azimuthPIDController = azimuthMotor.getPIDController();
        azimuthEncoder = new AnalogInput(encoderChannel);
        driveMotor = new TalonFX(driveMotorId);
        offsetAngle = offset;
    }

    /**
     * Configures the drive and azimuth motors with supplied PID 
     */
    public void initWheel() {
        azimuthMotor.restoreFactoryDefaults();
        TalonFXConfiguration configs = new TalonFXConfiguration();
        /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        configs.slot0.kP = drivekP;
        configs.slot0.kI = drivekI;
        configs.slot0.kD = drivekD;
        /* config all the settings */
        driveMotor.configAllSettings(configs);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

		/*
		 * Choose which direction motor should spin during positive
		 * motor-output/sensor-velocity. Note setInverted also takes classic true/false
		 * as an input.
		 */
		driveMotor.setInverted(TalonFXInvertType.CounterClockwise);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */

		/* Brake or coast during neutral */
		driveMotor.setNeutralMode(NeutralMode.Brake);
    
        // set azimuth PID coefficients
        azimuthPIDController.setP(azimuthkP);
        azimuthPIDController.setI(azimuthkI);
        azimuthPIDController.setD(azimuthkD);
        azimuthPIDController.setIZone(azimuthkIz);
        azimuthPIDController.setFF(azimuthkFF);
        azimuthPIDController.setOutputRange(azimuthkMinOutput, azimuthkMaxOutput);
        azimuthPIDController.setSmartMotionMaxVelocity(azimuthMaxVel, 0);
        azimuthPIDController.setSmartMotionMinOutputVelocity(azimuthMinVel, 0);
        azimuthPIDController.setSmartMotionMaxAccel(azimuthMaxAcc, 0);
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
        driveMotor.set(ControlMode.MotionMagic, setpoint * driveTicks);
        SmartDashboard.putNumber("driveSetpoint", setpoint);
        SmartDashboard.putNumber("drivePosition", driveMotor.getSelectedSensorPosition());
    } 

    /**
     * Set the drive to move a given velocity.
     * 
     * @param output target velocity 0 to 1.0
     */
    public void setDriveOutput(double output) {
        driveMotor.set(ControlMode.PercentOutput, output);
        SmartDashboard.putNumber("driveOutput", output);
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
        currentPosition += offsetAngle;
        currentPosition =  currentPosition < 0 ? currentPosition + 360.0 : currentPosition;
        currentPosition =  currentPosition > 180.0 ? currentPosition - 360.0 : currentPosition;
        currentPosition = currentPosition == 360 ? 0 : currentPosition; 
        targetAngle *= 360; 

        // The difference between the target angle and the current position 
        double azimuthError = Math.IEEEremainder(currentPosition - targetAngle, 360.0);
        // boolean overThres = false;

        // SmartDashboard.putBoolean("OverThres", overThres);

        // Minimize azimuth rotation by reversing drive if necessary
        // if (Math.abs(azimuthError) > 0.75 * 360) {
        //     azimuthError -= Math.copySign(360, azimuthError);
        //     drive = -drive;
        //     if (!overThres){
        //         overThres = true;
        //     }
        // } else 
        if (Math.abs(azimuthError) > 0.25 * 360) {
            azimuthError -= Math.copySign(0.5 * 360, azimuthError);
            drive = -drive;
             SmartDashboard.putBoolean("direction" + azimuthMotor.getDeviceId(), false);
        }   
        
        // SmartDashboard.putNumber("Encoder current position" + azimuthMotor.getDeviceId(), azimuthEncoder.getValue());
        // SmartDashboard.putNumber("currentPosition" + azimuthMotor.getDeviceId(), currentPosition);
        // SmartDashboard.putNumber("targetAngle" + azimuthMotor.getDeviceId(), targetAngle);
        // SmartDashboard.putNumber("AzimuthError" + azimuthMotor.getDeviceId(), azimuthError);
        // SmartDashboard.putNumber("Drive", drive);
        // SmartDashboard.putNumber("Current Spark Position", azimuthMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Target Spark Position (Encoder units)", azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("module" + azimuthMotor.getDeviceId(), currentPosition);
        // Prevent the azimuth from resetting position to zero
        if (drive != 0) {
            // The azimuth motor has units of 18 for one full rotation. The position is set using the azimuth error + current position. 
            azimuthPIDController.setReference(azimuthError / 360.0 * 18 + azimuthMotor.getEncoder().getPosition(), ControlType.kSmartMotion);
        }
        setDriveOutput(drive);
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    /**
     * Set the drive output to 0 and azimuth to the zeroed position
     */
    public void zero() {
        setTargetAngle(0);
        setDriveOutput(0);
    }

    /**
     * Close the analog input from the encoder and set all motor output to 0.
     */

    public void stop() {
        azimuthEncoder.close();
        azimuthMotor.set(0);
        driveMotor.set(ControlMode.PercentOutput, 0);
    }
}