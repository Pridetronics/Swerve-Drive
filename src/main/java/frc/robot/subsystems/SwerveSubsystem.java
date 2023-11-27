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

  private static final SwerveModuleClasses SwerveModeuleSettings = new SwerveModuleClasses();

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

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(WheelConstants.kDriveKinematics,
  new Rotation2d(0), new SwerveModulePosition[] {
    frontLeft.getSwervePosition(),
    frontRight.getSwervePosition(),
    backLeft.getSwervePosition(),
    backRight.getSwervePosition()
  });

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch(Exception e) {}
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getAngle() % 360;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

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

    // SmartDashboard.putNumber("Module [9] Absolute", frontLeft.getAbsoluteEncoderRad() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [10] Absolute", frontRight.getAbsoluteEncoderRad() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [11] Absolute", backRight.getAbsoluteEncoderRad() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [12] Absolute", backLeft.getAbsoluteEncoderRad() *(180/Math.PI) );

    // SmartDashboard.putNumber("Module [9] Relative", frontLeft.getTurningPosition() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [10] Relative", frontRight.getTurningPosition() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [11] Relative", backRight.getTurningPosition() *(180/Math.PI) );
    // SmartDashboard.putNumber("Module [12] Relative", backLeft.getTurningPosition() *(180/Math.PI) );

    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwervePosition(),
      frontRight.getSwervePosition(),
      backLeft.getSwervePosition(),
      backRight.getSwervePosition()
    });
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[1]);
    frontRight.setDesiredState(desiredStates[0]);
    backLeft.setDesiredState(desiredStates[3]);
    backRight.setDesiredState(desiredStates[2]);
  }
}
