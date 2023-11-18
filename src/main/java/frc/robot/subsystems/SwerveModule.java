// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.WheelConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final SparkMaxPIDController turningPidController;

  private final CANCoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(SwerveModuleConstants swerveModuleConstants) {

    driveMotor = new CANSparkMax(swerveModuleConstants.kDriveMotorCANID, MotorType.kBrushless);
    
    turningMotor = new CANSparkMax(swerveModuleConstants.kTurningMotorCANID, MotorType.kBrushless);

    driveMotor.setInverted(swerveModuleConstants.kDriveEncoderReversed);
    turningMotor.setInverted(swerveModuleConstants.kTurningEncoderReversed);

    this.absoluteEncoderOffsetRad = swerveModuleConstants.kAbsoluteEncoderOffsetRadians;
    this.absoluteEncoderReversed = swerveModuleConstants.kAbsoluteEncoderReversed;
    absoluteEncoder = new CANCoder(swerveModuleConstants.kTurningEncoderID);
    configAbsoluteEncoder();

    driveEncoder = driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    turningEncoder = turningMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    turningPidController = turningMotor.getPIDController();
    turningPidController.setP(WheelConstants.kPTurning);
    turningPidController.setI(0);
    turningPidController.setD(0);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
    turningPidController.setPositionPIDWrappingMinInput(-Math.PI);

    driveEncoder.setPositionConversionFactor(WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio);
    driveEncoder.setVelocityConversionFactor(WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio);

    turningEncoder.setPositionConversionFactor(WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio);
    turningEncoder.setVelocityConversionFactor(WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio);

    resetEncoders();
  }

  private void configAbsoluteEncoder() {
    CANCoderConfiguration absoluteEncoderConfig = new CANCoderConfiguration();
    
    absoluteEncoder.configFactoryDefault();

    absoluteEncoderConfig.sensorDirection = true;
    absoluteEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    absoluteEncoder.configAllSettings(absoluteEncoderConfig);

  }

  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d( getTurningPosition() ));
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition() % (2*Math.PI);
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double absoluteAngle = absoluteEncoder.getAbsolutePosition();

    double angle = Units.degreesToRadians(absoluteAngle);

    angle -= absoluteEncoderOffsetRad;

    return angle * (absoluteEncoderReversed ? -1 : 1);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState( getDriveVelocity(), new Rotation2d( getTurningPosition() ));
  }

  public void setDesiredState(SwerveModuleState state) {

    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);

    SmartDashboard.putString("Swerve Module[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }

  public void stop() {
    driveMotor.set(0);
    turningPidController.setReference(turningEncoder.getPosition(), ControlType.kPosition);
  }
}
 