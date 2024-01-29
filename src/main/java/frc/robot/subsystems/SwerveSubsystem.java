// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleClasses;
import frc.robot.Constants.WheelConstants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  //gets the constants for each module
  private static final SwerveModuleClasses SwerveModeuleSettings = new SwerveModuleClasses();
  //Creates an object for each module
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveModeuleSettings.frontLeft
  );

  private final SwerveModule frontRight = new SwerveModule(
    SwerveModeuleSettings.frontRight
  );

  private final SwerveModule backLeft = new SwerveModule(
    SwerveModeuleSettings.backLeft
  );

  private final SwerveModule backRight = new SwerveModule(
    SwerveModeuleSettings.backRight
  );

  //Gets the NavX gyro to tell what direction the robot is facing
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  //Uses the positions of each module to perdict where the robot is on the field (Mainly for autonomous)
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(WheelConstants.kDriveKinematics,
  new Rotation2d(0), new SwerveModulePosition[] {
    frontLeft.getSwervePosition(),
    frontRight.getSwervePosition(),
    backLeft.getSwervePosition(),
    backRight.getSwervePosition()
  });
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //Waits one second to let the gyro calibrate and then resets the forward direction
    //Non-yielding code
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch(Exception e) {}
    }).start();
  }

  //Resets the forward direction to the current forward direction of the robot
  public void zeroHeading() {
    gyro.reset();
  }

  //Sets the robot heading to be what it currently is on the field, useful if the robot is not aligned with the forward direction of the field at the start of the match
  public void setHeading(double currentRobotHeadingDegrees) {
    zeroHeading();
    gyro.setAngleAdjustment(currentRobotHeadingDegrees);
  }

  //Returns the heading of the robot on the field in degrees
  public double getHeading() {
    return gyro.getAngle() % 360;
  }

  //Gets the rotation of the robot for use by WPILIB systems
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  //Gets the position + rotation of the robot for use by WPILIB systems
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  //Resets the odometry to the robots current position and orientation (does NOT reset the gyro, just the odometer)
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwervePosition(),
      frontRight.getSwervePosition(),
      backLeft.getSwervePosition(),
      backRight.getSwervePosition()
    }, pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Tells the odometer the current position to compare with the last position and thus update its predicted position
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwervePosition(),
      frontRight.getSwervePosition(),
      backLeft.getSwervePosition(),
      backRight.getSwervePosition()
    });
    SmartDashboard.putString("Robot Location", getPose().toString());
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  //Stops all the modules
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  //Sets all the module's velocities and directions with an array of states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
