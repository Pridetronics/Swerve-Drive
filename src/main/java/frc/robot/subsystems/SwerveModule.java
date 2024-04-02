// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.utils.ShuffleboardRateLimiter;


public class SwerveModule extends SubsystemBase implements SwerveModuleInterface {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final SparkPIDController turningPidController;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;

  private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Drive");
  private final GenericEntry moduleStateEntry;
  private final GenericEntry modulePositionEntry;

  public SwerveModule(SwerveModuleConstants swerveModuleConstants) {
    //Sets motor controllers
    driveMotor = new CANSparkMax(swerveModuleConstants.kDriveMotorCANID, MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor = new CANSparkMax(swerveModuleConstants.kTurningMotorCANID, MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kBrake);

    //Sets motor forward directions
    driveMotor.setInverted(swerveModuleConstants.kDriveEncoderReversed);
    turningMotor.setInverted(swerveModuleConstants.kTurningEncoderReversed);

    //Sets absolute encoder
    this.absoluteEncoderReversed = swerveModuleConstants.kAbsoluteEncoderReversed;
    absoluteEncoder = new CANcoder(swerveModuleConstants.kTurningEncoderID);
    configAbsoluteEncoder(swerveModuleConstants.kAbsoluteEncoderOffsetDegrees);

    //Sets relative encoders from the NEO Motors
    driveEncoder = driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    turningEncoder = turningMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    //Sets the turning PID Controller
    turningPidController = turningMotor.getPIDController();
    turningPidController.setP(WheelConstants.kPTurning);
    turningPidController.setI(0);
    turningPidController.setD(0);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
    turningPidController.setPositionPIDWrappingMinInput(-Math.PI);

    //Conversion factors to convert from motor rotations to distacne in meters
    driveEncoder.setPositionConversionFactor(WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio);
    //converts to meters per second
    driveEncoder.setVelocityConversionFactor((WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio) / 60);
    
    //Conversion factors to convert from motor rotations to rotations in radians
    turningEncoder.setPositionConversionFactor(WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio);
    //converts to radians per second
    turningEncoder.setVelocityConversionFactor((WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio) / 60);

    resetEncoders();

    moduleStateEntry = swerveTab.add("Module [" + absoluteEncoder.getDeviceID() + "] State", "N/A").getEntry();
    modulePositionEntry = swerveTab.add("Module [" + absoluteEncoder.getDeviceID() + "] Position", "N/A").getEntry();

  }

  //Config for CANCoder because CTRE likes doing this i guess
  private void configAbsoluteEncoder(double absoluteOffsetDegrees) {
    CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = -absoluteOffsetDegrees/360;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);

  }

  //This for the odometer specifically
  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition( getDrivePosition(), new Rotation2d( getTurningPosition() ));
  }

  //Returns wheel position in meters
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }
  //returns wheel direction in radians
  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }
  //returns wheel velocity in meters per second
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }
  //returns wheel direction velocity in radians per second
  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }
  //Gets and returns the position on the absolute encoder, used to center the wheel with the robot
  private double getAbsoluteEncoderRad() {
    //Uses Status signals since CTRE thinks they are cool
    StatusSignal<Double> absoluteAngleSignal = absoluteEncoder.getAbsolutePosition();
    double absoluteAngle = absoluteAngleSignal.getValue()*360;
    swerveTab.add("Absolute Angle [" + absoluteEncoder.getDeviceID() + "]", absoluteAngle);
    absoluteAngle = absoluteEncoderReversed ? 360-absoluteAngle : absoluteAngle;

    double angle = Units.degreesToRadians(absoluteAngle);

    return angle;
  }

  //Resets the encoders, sets drive to zero and sets the turning to its current position relative to the robot's forward direction
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  //Returns the state of the module with velocity and direction
  public SwerveModuleState getState() {
    return new SwerveModuleState( getDriveVelocity(), new Rotation2d( getTurningPosition() ));
  }

  //Sets the module to a drive velocity and directional position
  public void setDesiredState(SwerveModuleState state) {    
    ShuffleboardRateLimiter.queueDataForShuffleboard(moduleStateEntry, getState().toString());
    ShuffleboardRateLimiter.queueDataForShuffleboard(modulePositionEntry, getSwervePosition().toString());

    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }
    //Prevents the wheel from taking more than a 90 degree turn
    state = SwerveModuleState.optimize(state, getState().angle);
    //Sets the wheel speed as a percent of the physical max speed, might change later with a velocity PID controller
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //driveMotor.set(0);

    //Sets the target angle for the turning motor
    turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
    //turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), state.angle.getRadians()));
  }

  //Stops the motor from moving and turning
  public void stop() {
    driveMotor.set(0);
    //Sets the turning motor to stop where its currently at
    turningPidController.setReference(getTurningPosition(), ControlType.kPosition);
    //turningMotor.set(0);
  }
}
 
