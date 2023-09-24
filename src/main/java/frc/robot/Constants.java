// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import frc.robot.SwerveModuleConstants;

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

  public static class WheelConstants {
  
    public static final double kSwerveWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1 / 5.8462;
    public static final double kTurningMotorGearRatio = 1 / 18.0;

    public static final double kDistancePerWheelRotation = kSwerveWheelDiameterMeters*Math.PI;
    public static final double k360DegreesToRadians = 2*Math.PI;

    public static final double kPTurning = 0.5;
  }

  public static class DriveConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    public static final int kFrontLeftDriveMotorCANID = 8;
    public static final int kFrontLeftTurningMotorCANID = 7;
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final int kFrontLeftDriveAbsoluteEncoderCANID = 0;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderOffsetDeg = -15;

    public static final int kBackLeftDriveMotorCANID = 2;
    public static final int kBackLeftTurningMotorCANID = 1;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderCANID = 2;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderOffsetDeg = -71;


    public static final int kFrontRightDriveMotorCANID = 6;
    public static final int kFrontRightTurningMotorCANID = 5;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderCANID = 1;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderOffsetDeg = -104;


    public static final int kBackRightDriveMotorCANID = 4;
    public static final int kBackRightTurningMotorCANID = 3;
    public static final boolean kBackRightTurningEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderCANID = 3;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderOffsetDeg = -270;
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
