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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SparkPIDController turningPidController;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;

  public SwerveModule(SwerveModuleConstants swerveModuleConstants) {

    driveMotor = new CANSparkMax(swerveModuleConstants.kDriveMotorCANID, MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor = new CANSparkMax(swerveModuleConstants.kTurningMotorCANID, MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setInverted(swerveModuleConstants.kDriveEncoderReversed);
    turningMotor.setInverted(swerveModuleConstants.kTurningEncoderReversed);

    this.absoluteEncoderReversed = swerveModuleConstants.kAbsoluteEncoderReversed;
    absoluteEncoder = new CANcoder(swerveModuleConstants.kTurningEncoderID);
    configAbsoluteEncoder(swerveModuleConstants.kAbsoluteEncoderOffsetDegrees);

    driveEncoder = driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    turningEncoder = turningMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    turningPidController = turningMotor.getPIDController();
    turningPidController.setP(WheelConstants.kPTurning);
    turningPidController.setI(0);
    turningPidController.setD(0);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
    turningPidController.setPositionPIDWrappingMinInput(-Math.PI);
    driveEncoder.setPositionConversionFactor(WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio);
    driveEncoder.setVelocityConversionFactor((WheelConstants.kDistancePerWheelRotation*WheelConstants.kDriveMotorGearRatio) / 60);

    turningEncoder.setPositionConversionFactor(WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio);
    turningEncoder.setVelocityConversionFactor((WheelConstants.k360DegreesToRadians*WheelConstants.kTurningMotorGearRatio) / 60);

    resetEncoders();
  }

  private void configAbsoluteEncoder(double absoluteOffsetDegrees) {
    CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = -absoluteOffsetDegrees/360;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);

  }

  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition( getDrivePosition(), new Rotation2d( getTurningPosition() ));
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    StatusSignal<Double> absoluteAngleSignal = absoluteEncoder.getAbsolutePosition();
    double absoluteAngle = absoluteAngleSignal.getValue()*360;
    SmartDashboard.putNumber("AbsoluteAngle [" + absoluteEncoder.getDeviceID() + "]", absoluteAngle);
    absoluteAngle = absoluteEncoderReversed ? 360-absoluteAngle : absoluteAngle;

    double angle = Units.degreesToRadians(absoluteAngle);

    return angle;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState( getDriveVelocity(), new Rotation2d( getTurningPosition() ));
  }

  public void setDesiredState(SwerveModuleState state) {    
    SmartDashboard.putString("Module [" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //driveMotor.set(0);

    turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
    //turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), state.angle.getRadians()));
  }

  public void stop() {
    driveMotor.set(0);
    turningPidController.setReference(turningEncoder.getPosition(), ControlType.kPosition);
    //turningMotor.set(0);
  }
}
 
