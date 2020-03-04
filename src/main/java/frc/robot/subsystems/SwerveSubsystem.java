/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.SwerveConstants.*;

import java.util.ArrayList;

import static frc.robot.Constants.LimelightConstants.*;

/**
 * FRC Team 2090's Swerve Drive Code.
 * 
 * <p>Swerve logic and math is based on Team 2767's <a 
 * href="https://github.com/strykeforce/thirdcoast/tree/master/src/main/java/org/strykeforce/thirdcoast/swerve">
 * Third Coast Java Libraries </a>
 * 
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 */
public class SwerveSubsystem extends SubsystemBase {
  private static final Wheel[] wheels = new Wheel[4];
  private final double[] ws = new double[4]; 
  private final double[] wa = new double[4]; 
  private boolean isFieldOriented;
  public final AHRS gyro;
  private final double kLengthComponent;
  private final double kWidthComponent;
  private final ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
  private Orchestra orchestra;
  //private final Servo visionServo;

  /**
   * This constructs the Swerve Subsystem with the navx and given constants 
   * including the ratio of the robot length to width. The field oriented driving
   * mode is set based on whether the gyro is properly connected.
   */
  public SwerveSubsystem() {
    gyro = new AHRS();
    //visionServo = new Servo(visionServoPort);
    double radius = Math.hypot(robotLength, robotWidth);
    kLengthComponent = robotLength / radius;
    kWidthComponent = robotWidth / radius;
    setFieldOriented(true);
    generateWheels();
  }

  /**
   * Instantiates the wheels with the correct ports for the drive motor, azimuth motor, encoder, and angle offset.
   * Wheels are an array numbered 0-3 from front to back, with even numbers on the left side when facing forward.
   * 
   * Initialize the wheels and set them to the absolute zero based on the each Wheel's given offset.
   */
  public void generateWheels() {
    wheels[0] = new Wheel(FRONT_LEFT_ANGLE_MOTOR, FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET);
    wheels[1] = new Wheel(FRONT_RIGHT_ANGLE_MOTOR, FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET);
    wheels[2] = new Wheel(BACK_LEFT_ANGLE_MOTOR, BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET);
    wheels[3] = new Wheel(BACK_RIGHT_ANGLE_MOTOR, BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET);
    
    for (Wheel wheel : wheels) {
      wheel.initWheel();
      wheel.zero();
      instruments.add(wheel.getDriveMotor());
    }
  }

  /**
   * Returns the array of Wheels
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * 
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double yaw) {
    isFieldOriented = SmartDashboard.getBoolean("driveMode", true);
    /* If the robot is field oriented, the inputed values are modified to 
     * be with respect to the zero of the field on the navx.
    */
    if (isFieldOriented) {
      double angle = gyro.getAngle();
      SmartDashboard.putNumber("Gyro", gyro.getAngle());

      angle += gyro.getRate();
      angle = Math.IEEEremainder(angle, 360.0);

      angle = Math.toRadians(angle);
      final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = temp;
    }

    final double a = strafe - yaw * kLengthComponent;
    final double b = strafe + yaw * kLengthComponent;
    final double c = forward - yaw * kWidthComponent;
    final double d = forward + yaw * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d);
    ws[1] = Math.hypot(b, c);
    ws[2] = Math.hypot(a, d);
    ws[3] = Math.hypot(a, c);

    // wheel yaw
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    //normalize wheel speed
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }
    //set wheels
    for (int i = 0; i < 4; i++) {
      wheels[i].set(wa[i], ws[i]);
    }
   
  }

  /**
   * 
   * Set the drive with respect to the field
   *
   * @param enable true to set robot to field oriented driving, false to set to robot oriented
   */
  public void setFieldOriented(boolean enable) {
    isFieldOriented = enable;
    SmartDashboard.putBoolean("driveMode", enable);
  }

  /**
   * Stop all Swerve Modules
   */
  public void stop() {
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
    SmartDashboard.putString("State", "Stopped");
  }

  /**
   * Zero all Swerve Modules (azimuth position and drive output are set to 0)
   */
  public void zero() {
    for (Wheel wheel : wheels) {
      wheel.zero();
    }
  }

  public void robotOrientedDrive(double forward, double strafe, double yaw) {
    setFieldOriented(false);
    drive(forward, strafe, yaw);
    setFieldOriented(true);
  }

  public void updateLimelightTracking(double forwardInput, double strafeInput) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double headingError = -table.getEntry("tx").getDouble(0);

    SmartDashboard.putNumber("tx", headingError);
  
    double yawInput = 0;
    yawInput = Math.abs(headingError) < maxHeadingError ? 0 : (headingError * headingConstant);
    yawInput = Math.abs(yawInput) > maxYawOutput ? Math.copySign(maxYawOutput, yawInput): yawInput;

    drive(forwardInput, strafeInput, yawInput);
  }

  // public void adjustVisionPosition(double offset) {
  //   // offset += visionServo.get();
  //   // offset = offset > visionUpperPosition ? 0.8 : offset < visionLowerPosition ? 0.2 : offset;
  //   // visionServo.set(offset);
  // }

  /**
   * Change current output based on the whether or not other subsystems are running
   */
  public void modifyOutput() {
    
  }

  public void playMusic() {
    orchestra = new Orchestra(instruments);

    String[] songs = new String[] {
      "song1.chrp",
      "song2.chrp",
      "song3.chrp",
      "song4.chrp",
      "song5.chrp",
      "song6.chrp",
      "song7.chrp",
      "song8.chrp",
      "song9.chrp", /* the remaining songs play better with three or more FXs */
      "song10.chrp",
      "song11.chrp",
    };

    if (orchestra.isPlaying()) {
      orchestra.loadMusic(songs[(int) SmartDashboard.getNumber("Song", 0)]); 
      orchestra.play();
    }
  }
}
