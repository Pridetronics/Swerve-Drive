// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically imCANID this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerCANID = 0;

  }

  public static class IOConstants {
    public static final double kDeadband = 0.1;
    public static final int kDriveJoystickID = 0;

    public static final int kDriveJoystickXAxis = 1;
    public static final int kDriveJoystickYAxis = 0;
    public static final int kDriveJoystickTurningAxis = 4;
    public static final int kDriveFieldOrientedDriveBtnID = 4;
    public static final int kZeroHeadingBtnID = 2;
    public static final double kZeroHeadingDebounceTime = 2;
  }

  public static class WheelConstants {
  
    public static final double kSwerveWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    public static final double kDistancePerWheelRotation = kSwerveWheelDiameterMeters*Math.PI;
    public static final double k360DegreesToRadians = 2*Math.PI;

    public static final double kPTurning = 0.8;

    
    //Distance between the right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    //DIstance between the front and back wheels
    public static final double kWheelBaseLength = Units.inchesToMeters(22.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseLength/2, -kTrackWidth/2), //Front Left
      new Translation2d(kWheelBaseLength/2, kTrackWidth/2), //Front Right
      new Translation2d(-kWheelBaseLength/2, -kTrackWidth/2), //Back Left
      new Translation2d(-kWheelBaseLength/2, kTrackWidth/2) //Beck Right
    );

  }

  public static class DriveConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final double kTeleMaxDriveAccelerationUnitsPerSecond = 3;
    public static final double kTeleMaxTurningAccelerationUnitsPerSecond = 3;

    public static final double kTeleMaxDriveSpeedMetersPerSecond = 2.25;
    public static final double kTeleMaxTurningSpeedRadiansPerSecond = 270 * (Math.PI/180);

    public static final int kFrontLeftDriveMotorCANID = 2;
    public static final int kFrontLeftTurningMotorCANID = 1;
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderCANID = 9;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderOffsetDeg = 0;


    public static final int kFrontRightDriveMotorCANID = 4;
    public static final int kFrontRightTurningMotorCANID = 3;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final int kFrontRightDriveAbsoluteEncoderCANID = 10;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderOffsetDeg = 0;


    public static final int kBackRightDriveMotorCANID = 6;
    public static final int kBackRightTurningMotorCANID = 5;
    public static final boolean kBackRightTurningEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderCANID = 11;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderOffsetDeg = 0;

    public static final int kBackLeftDriveMotorCANID = 8;
    public static final int kBackLeftTurningMotorCANID = 7;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderCANID = 12;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderOffsetDeg = 0; 
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecond = 3;

    public static final double kMaxTurningSpeedRadiansPerSecond = 30 * (Math.PI / 180);
    public static final double kMaxTurningAccelerationRadiansPerSecond = 180 * (Math.PI / 180);

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
      kMaxTurningSpeedRadiansPerSecond,
      kMaxTurningAccelerationRadiansPerSecond);
  }

  public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecond
  ).setKinematics(WheelConstants.kDriveKinematics);

  public static class SwerveModuleConstants {
      public final int kDriveMotorCANID;
      public final int kTurningMotorCANID;
      public final int kTurningEncoderID;
      public final double kAbsoluteEncoderOffsetDegrees;
      public final boolean kAbsoluteEncoderReversed;
      public final boolean kDriveEncoderReversed;
      public final boolean kTurningEncoderReversed;

      SwerveModuleConstants(
              int driveMotorCANID, 
              int turningMotorCANID, 
              int CTRETurningEncoderID,
              int absoluteEncoderOffsetDegrees,
              boolean absoluteEncoderReversed,
              boolean driveEncoderReversed,
              boolean turningEncoderReversed
          ) {

          kDriveMotorCANID = driveMotorCANID;
          kTurningMotorCANID = turningMotorCANID;
          kTurningEncoderID = CTRETurningEncoderID;
          kAbsoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;// * (Math.PI/180);
          kAbsoluteEncoderReversed = absoluteEncoderReversed;
          kDriveEncoderReversed = driveEncoderReversed;
          kTurningEncoderReversed = turningEncoderReversed;
          
      }
  }


  public static class SwerveModuleClasses {

    public SwerveModuleConstants backLeft = new SwerveModuleConstants(
      DriveConstants.kBackLeftDriveMotorCANID, 
      DriveConstants.kBackLeftTurningMotorCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackLeftDriveEncoderReversed, 
      DriveConstants.kBackLeftTurningEncoderReversed
    );

    public SwerveModuleConstants backRight = new SwerveModuleConstants(
      DriveConstants.kBackRightDriveMotorCANID, 
      DriveConstants.kBackRightTurningMotorCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackRightDriveEncoderReversed, 
      DriveConstants.kBackRightTurningEncoderReversed
    );
    public SwerveModuleConstants frontLeft = new SwerveModuleConstants(
      DriveConstants.kFrontLeftDriveMotorCANID, 
      DriveConstants.kFrontLeftTurningMotorCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontLeftDriveEncoderReversed, 
      DriveConstants.kFrontLeftTurningEncoderReversed
    );
    public SwerveModuleConstants frontRight = new SwerveModuleConstants(
      DriveConstants.kFrontRightDriveMotorCANID, 
      DriveConstants.kFrontRightTurningMotorCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontRightDriveEncoderReversed, 
      DriveConstants.kFrontRightTurningEncoderReversed
    );
  }
}
