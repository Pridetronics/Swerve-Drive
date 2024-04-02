// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleClasses;
import frc.robot.Constants.WheelConstants;
import frc.robot.utils.ShuffleboardRateLimiter;

public class SwerveSubsystem extends SubsystemBase {
  //Creates an object for each module
  private final SwerveModuleInterface frontLeft;

  private final SwerveModuleInterface frontRight;

  private final SwerveModuleInterface backLeft;

  private final SwerveModuleInterface backRight;

  //Gets the NavX gyro to tell what direction the robot is facing
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private double simulatedGyroAngle = 0;
  public double simulatedRobotAngle = 0;
  private Timer simulatedGyroTimer;

  //Uses the positions of each module to perdict where the robot is on the field (Mainly for autonomous)
  private final SwerveDrivePoseEstimator odometer;

  private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Drive");
  private final GenericEntry robotRotationEntry = swerveTab.add("Robot Heading", 0.0)
    .withWidget(BuiltInWidgets.kGyro)
    .getEntry();
  private final GenericEntry robotPositionEntry = swerveTab.add("Robot Location", "N/A")
    .getEntry();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //gets the constants for each module
    final SwerveModuleClasses SwerveModuleSettings = new SwerveModuleClasses();
    if (RobotBase.isReal()) {
      frontLeft = new SwerveModule(SwerveModuleSettings.frontLeft);
      frontRight = new SwerveModule(SwerveModuleSettings.frontRight);
      backLeft = new SwerveModule(SwerveModuleSettings.backLeft);
      backRight = new SwerveModule(SwerveModuleSettings.backRight);
    } else {
      frontLeft = new SimulatedSwerveModule(SwerveModuleSettings.frontLeft);
      frontRight = new SimulatedSwerveModule(SwerveModuleSettings.frontRight);
      backLeft = new SimulatedSwerveModule(SwerveModuleSettings.backLeft);
      backRight = new SimulatedSwerveModule(SwerveModuleSettings.backRight);
    }    
    this.odometer = new SwerveDrivePoseEstimator(
      WheelConstants.kDriveKinematics,
      new Rotation2d(0), 
      new SwerveModulePosition[] {
        frontLeft.getSwervePosition(),
        frontRight.getSwervePosition(),
        backLeft.getSwervePosition(),
        backRight.getSwervePosition()
      }, 
      new Pose2d(),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(2,2, 2)
    );
    //Waits one second to let the gyro calibrate and then resets the forward direction
    //Non-yielding code
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch(Exception e) {}
    }).start();

    if (RobotBase.isSimulation()) {
      simulatedGyroTimer = new Timer();
      simulatedGyroTimer.start();
    }
  }

  //Resets the forward direction to the current forward direction of the robot
  public void zeroHeading() {
    if (RobotBase.isSimulation()) {
      simulatedGyroAngle = 0;
    }

    gyro.reset();
    
    resetOdometry(
      new Pose2d(
        getPose().getTranslation(), 
        new Rotation2d()
      )
    );
  }

  //Sets the robot heading to be what it currently is on the field, useful if the robot is not aligned with the forward direction of the field at the start of the match
  // public void setHeading(double currentRobotHeadingDegrees) {
  //   gyro.setAngleAdjustment(currentRobotHeadingDegrees);
  //   zeroHeading();
  // }

  //Returns the heading of the robot on the field in degrees
  public double getHeading() {
    if (RobotBase.isSimulation()) {
      return simulatedGyroAngle % 360;
    }
    return (-gyro.getAngle()) % 360;
  }

  //Used in climber subsystem
  public double getGyroRoll() {
    return gyro.getRoll();
  }

  //Gets the rotation of the robot for use by WPILIB systems
  public Rotation2d getRotation2d() {
    return odometer.getEstimatedPosition().getRotation();
  }

  private Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
    
  }

  //Gets the position + rotation of the robot for use by WPILIB systems
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose2d pose, double time) {
    odometer.addVisionMeasurement(pose, time);
  }

  //Resets the odometry to the robots current position and orientation (does NOT reset the gyro, just the odometer)
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getGyroRotation2d(), new SwerveModulePosition[] {
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
    odometer.update(getGyroRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwervePosition(),
      frontRight.getSwervePosition(),
      backLeft.getSwervePosition(),
      backRight.getSwervePosition()
    });
    ShuffleboardRateLimiter.queueDataForShuffleboard(robotPositionEntry, getPose().getTranslation().toString());
    ShuffleboardRateLimiter.queueDataForShuffleboard(robotRotationEntry, -getPose().getRotation().getDegrees());

    SmartDashboard.putString("Copy paste pose2d", 
      String.format("%.2f, %.2f, Rotation2d.fromDegrees(%d)",
        getPose().getTranslation().getX(),
        getPose().getTranslation().getY(),
        (int) Math.rint(getPose().getRotation().getDegrees())
      )
    );
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

    if (RobotBase.isSimulation()) {
      ChassisSpeeds originalSpeeds = WheelConstants.kDriveKinematics.toChassisSpeeds(
        desiredStates[0],
        desiredStates[1],
        desiredStates[2],
        desiredStates[3]
      );
      simulatedGyroAngle += Units.radiansToDegrees(originalSpeeds.omegaRadiansPerSecond*DriveConstants.kTeleMaxTurningSpeedRadiansPerSecond*simulatedGyroTimer.get());
      simulatedRobotAngle += Units.radiansToDegrees(originalSpeeds.omegaRadiansPerSecond*DriveConstants.kTeleMaxTurningSpeedRadiansPerSecond*simulatedGyroTimer.get());
      simulatedGyroTimer.reset();
    }
  }
}
