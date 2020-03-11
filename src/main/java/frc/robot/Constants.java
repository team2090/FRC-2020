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
    public static final int armForwardChannel = 6;
    public static final int armReverseChannel = 7;
    public static final int lock1port = 1;
    public static final int lock2port = 2;
  }

  public static final class ShooterConstants {
    public static final int shooter1 = 20;
    public static final int shooter2 = 21;

    public static final int intakeMotorId = 8;
    public static final int storageMotorId = 9;

    public static final int ballShifterForwardChannel = 0;
    public static final int ballShifterReverseChannel = 1;
    public static final int intakeReleaseForwardChannel = 2;
    public static final int intakeReleaseReverseChannel = 3;

    // PID coefficients
    public static final double shooterkP = 0.7e-2;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0.1;
    public static final double shooterkIz = 0;
    public static final double shooterkFF = 0;
    public static final double shooterkMaxOutput = 1;
    public static final double shooterkMinOutput = -1;
    public static final double shooterMaxRPM = 4000;

    // PID coefficients
    public static final double ballStoragekP = 5e-5;
    public static final double ballStoragekI = 1e-6;
    public static final double ballStoragekD = 0;
    public static final double ballStoragekIz = 0;
    public static final double ballStoragekFF = 0;
    public static final double ballStoragekMaxOutput = 1;
    public static final double ballStoragekMinOutput = -1;
  }

  public static final class SwerveConstants {
    // Ratio
    public static final double robotLength = 1;
    public static final double robotWidth = 1;

    // Front Left
    public static final int FRONT_LEFT_ANGLE_MOTOR = 4;
    public static final int FRONT_LEFT_DRIVE_MOTOR = 14;
    public static final double FRONT_LEFT_OFFSET = 195;
    public static final int FRONT_LEFT_ENCODER = 3;

    // Front Right
    public static final int FRONT_RIGHT_ANGLE_MOTOR = 3;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
    public static final double FRONT_RIGHT_OFFSET = -31;
    public static final int FRONT_RIGHT_ENCODER = 2;

    // Back Right
    public static final int BACK_RIGHT_ANGLE_MOTOR = 2;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 12;
    public static final double BACK_RIGHT_OFFSET = 66;
    public static final int BACK_RIGHT_ENCODER = 1;

    // Back Left
    public static final int BACK_LEFT_ANGLE_MOTOR = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR = 11;
    public static final double BACK_LEFT_OFFSET = 124;
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
    public static final double drivekP = 0.2;
    public static final double drivekI = 0.0;
    public static final double drivekD = 0.0;
    public static final double drivekIz = 0;
    public static final double drivekFF = 0.2;
    public static final double drivekMaxOutput = 1;
    public static final double drivekMinOutput = -1;

    // Smart Motion Coefficients
    // I found max at 5720
    public static final double azimuthMaxVel = 5700; // rpm
    public static final double azimuthMaxAcc = 2500;
    public static final double azimuthMinVel = 0;
    
    public static final int driveTicks = 2048;

    public static final int visionServoPort = 0;

    public static final double visionUpperPosition = 0.8;
    public static final double visionLowerPosition = 0.2;
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
