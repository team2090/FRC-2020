/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class HangConstants {

  }

  public static final class IntakeConstants {
    
  }

  public static final class ShooterConstants {
    public static final int shooter1 = 6;
    public static final int shooter2 = 7;
  }

  public static final class SwerveConstants {
    // Ratio
    public static final double robotLength = 1;
    public static final double robotWidth = 1;

    // Front Left
    public static final int FRONT_LEFT_ANGLE_MOTOR = 4;
    public static final int FRONT_LEFT_DRIVE_MOTOR = 14;
    public static final double FRONT_LEFT_OFFSET = 122;
    public static final int FRONT_LEFT_ENCODER = 3;

    // Front Right
    public static final int FRONT_RIGHT_ANGLE_MOTOR = 3;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
    public static final double FRONT_RIGHT_OFFSET = -151;
    public static final int FRONT_RIGHT_ENCODER = 2;

    // Back Right
    public static final int BACK_RIGHT_ANGLE_MOTOR = 2;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 12;
    public static final double BACK_RIGHT_OFFSET = -136;
    public static final int BACK_RIGHT_ENCODER = 1;

    // Back Left
    public static final int BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR = 11;
    public static final double BACK_LEFT_OFFSET = 55;
    public static final int BACK_LEFT_ENCODER = 0;

    // Azimuth PID coefficients
    public static final double azimuthkP = 5e-5;
    public static final double azimuthkI = 1e-6;
    public static final double azimuthkD = 0;
    public static final double azimuthkIz = 0;
    public static final double azimuthkFF = 0;
    public static final double azimuthkMaxOutput = 1;
    public static final double azimuthkMinOutput = -1;

    // Drive PID coefficients
    public static final double drivekP = 0.25;
    public static final double drivekI = 0.001;
    public static final double drivekD = 20;
    public static final double drivekIz = 300;
    public static final double drivekFF = 1.0;
    public static final double drivekMaxOutput = 1;
    public static final double drivekMinOutput = -1;

    // Smart Motion Coefficients
    public static final double azimuthMaxVel = 2500; // rpm
    public static final double azimuthMaxAcc = 1500;
    public static final double azimuthMinVel = 0;
    
    public static final int driveTicks = 2048;
  }

  public static final class LimelightConstants {
    public static final double headingConstant = 0.03; // how hard to turn toward the target
    public static final double driveConstant = 0.26; // how hard to drive fwd toward the target
    public static final double maxDistanceError = 5.0;
    public static final double maxHeadingError = 1.0;
    public static final double maxForwardOutput = 0.5;
    public static final double maxYawOutput = 0.2;

  }
}
