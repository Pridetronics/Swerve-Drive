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

  //Constants for features related to user controller input
  public static class IOConstants {
    //Deadband for the joysticks (for driving inputs so the robot doesnt move when not touching controller)
    public static final double kDeadband = 0.1;
    //Identifier for the controller
    public static final int kDriveJoystickID = 0;

    //Axis for right/left movement
    public static final int kDriveJoystickXAxis = 1;
    //Axis for forward/backward movement
    public static final int kDriveJoystickYAxis = 0;
    //Axis for turning
    public static final int kDriveJoystickTurningAxis = 4;
    //Button ID for robot oriented drive (when holding)
    public static final int kDriveFieldOrientedDriveBtnID = 4;
    //Button ID for reseting the orientation of the robot to the forward direction of the robot
    public static final int kZeroHeadingBtnID = 2;
    //Time it takes before you can press the zero heading button again (seconds)
    public static final double kZeroHeadingDebounceTime = 2;
  }

  //Constants for data related to the wheels of each module (not the module itself)
  public static class WheelConstants {

    //Diameter of the wheel
    public static final double kSwerveWheelDiameterMeters = Units.inchesToMeters(4);

    //Gear ratio of the propultion motor (number of wheel rotations per motor rotation)
    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    //Gear ratio of the turning motor (number of wheel rotations per motor rotation)
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    //The distance traveled, in meters, per rotation of the wheel
    public static final double kDistancePerWheelRotation = kSwerveWheelDiameterMeters*Math.PI;
    //Just because im lazy
    public static final double k360DegreesToRadians = 2*Math.PI;

    //Power value in the PIDController that controls the wheel direction
    public static final double kPTurning = 0.8;

    
    //Distance between the right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    //DIstance between the front and back wheels
    public static final double kWheelBaseLength = Units.inchesToMeters(22.5);

    //Kinematics system that solves for each wheel's direction based on the given target direction ahd turn velocity
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseLength/2, -kTrackWidth/2), //Front Left
      new Translation2d(kWheelBaseLength/2, kTrackWidth/2), //Front Right
      new Translation2d(-kWheelBaseLength/2, -kTrackWidth/2), //Back Left
      new Translation2d(-kWheelBaseLength/2, kTrackWidth/2) //Beck Right
    );

  }

  //Constants for the movement of the robot
  public static class DriveConstants {
    //The literal max speed each wheel is allowed to go
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    /*
    Imagine 0 means no speed and 1 means 100% speed speed, numbers are in the form of a decimal percent
      -A value of 1 means the 0 to max time is 1 second
      -A value of 3 means the 0 to max time is 0.333333 seconds
      -A value of 0.5 means the 0 to max time is 2 seconds
    You get the idea
   */
    public static final double kTeleMaxDriveAccelerationUnitsPerSecond = 3;
    public static final double kTeleMaxTurningAccelerationUnitsPerSecond = 3;

    //Max speed of the robot itself
    public static final double kTeleMaxDriveSpeedMetersPerSecond = 2.25;
    //Max turning speed of the robot specified in degrees but converted to radians (with the "(Math.PI/180)")
    public static final double kTeleMaxTurningSpeedRadiansPerSecond = 270 * (Math.PI/180);

    //ID of the Can Spark Max the propels the swerve module wheel
    public static final int kFrontLeftDriveMotorCANID = 2;
    //ID of the Can Spark Max that turns the swerve module wheel
    public static final int kFrontLeftTurningMotorCANID = 1;
    //Whether the turning direction of the wheel is reversed
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    //Whether the propultion direction of the wheel is reversed
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    //ID of the CTRE CANCoder for getting the absolute position of the wheel 
      //(the encoder is the silly little device on top of the module that is wedged bwtween the two motors)
    public static final int kFrontLeftDriveAbsoluteEncoderCANID = 9;
    //Whether the CANCoder direction is reversed
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    /* Offset of the absolute encoder relative to the forward direction of the wheel
          My advice for setting this value to prevent a headache experience:
            1. Face all wheels in the forward direction of the robot, being as close to perfect as you can
            2. Open the Phoenix Tuner X, connect laptop to RoboRio via USB cable (Not by ethenet cable)
            3. Go through each encoder on the device list, press refresh to get the data for it, and get the current rotation of the encoder in degrees
              - The data you see when pressing refresh will include ahe absolute position data (NOT "POSITION"; ITS CALLED "ABSOLUTE POSITION"), you will use the Absolute position (without sensor/magnet offset)
              - The number will be a number between 0 and 1 (NOT A NEGATIVE NUMBER)
              - Multiply that number by 360 to convert it to degrees
            4. Take the number and simply just place it below! it will subtract that number from the encoder to make the wheel always face forward on the robot

       Your welcome! -Guy who made this program   
    */
    public static final int kFrontLeftDriveAbsoluteEncoderOffsetDeg = 143;

    //Everything above applies for the below modules
    
    public static final int kFrontRightDriveMotorCANID = 4;
    public static final int kFrontRightTurningMotorCANID = 3;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderCANID = 10;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderOffsetDeg = 172;


    public static final int kBackRightDriveMotorCANID = 6;
    public static final int kBackRightTurningMotorCANID = 5;
    public static final boolean kBackRightTurningEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderCANID = 11;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderOffsetDeg = 212;

    public static final int kBackLeftDriveMotorCANID = 8;
    public static final int kBackLeftTurningMotorCANID = 7;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderCANID = 12;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderOffsetDeg = 165; 
  }

  //Constants related to the autonomous period
  public static class AutoConstants {
    //Max speed during autonomous
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    //Acceleration during autonomous (note its in meters, not units)
    public static final double kMaxAccelerationMetersPerSecond = 3;

    //Max turning speed during autonomous
    public static final double kMaxTurningSpeedRadiansPerSecond = 30 * (Math.PI / 180);
    //Acceleration during autonomous (note its in radians, not units)
    public static final double kMaxTurningAccelerationRadiansPerSecond = 180 * (Math.PI / 180);

    //Power Controllers for the robot to keep it on course
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    //This is for the Profiled PID Controller that controls the robot direction
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
      kMaxTurningSpeedRadiansPerSecond,
      kMaxTurningAccelerationRadiansPerSecond);
  }
  //Data for autonomous trajectories
  public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecond
  ).setKinematics(WheelConstants.kDriveKinematics);

  //Class that can store swerve module data for the swerve module class
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

  //Class that stores swerve module constants fot the swerve module class
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
