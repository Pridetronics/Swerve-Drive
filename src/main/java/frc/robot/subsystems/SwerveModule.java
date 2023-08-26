// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final SparkMaxPIDController turningPidController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(
      int driveMotorId, 
      int turnignMotorId, 
      boolean driveMotorReversed, 
      boolean turningMotorReversed, 
      int absoluteEncoderId, 
      double absoluteEncoderOffset, 
      boolean absoluteEncoderReversed
    ) {
  
    
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    turningPidController = turningMotor.getPIDController();
    turningPidController.setP(WheelConstants.kPTurning);
    turningPidController.setI(0);
    turningPidController.setD(0);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
    turningPidController.setPositionPIDWrappingMinInput(-Math.PI);

    driveEncoder.setPositionConversionFactor(WheelConstants.kDistancePerWheelRotation);
    driveEncoder.setVelocityConversionFactor(WheelConstants.kDistancePerWheelRotation);

    turningEncoder.setPositionConversionFactor(WheelConstants.k360DegreesToRadians);
    turningEncoder.setVelocityConversionFactor(WheelConstants.k360DegreesToRadians);

    resetEncoders();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return driveEncoder.getVelocity();
  }

  public double getDriveVelocity() {
    return turningEncoder.getPosition();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();

    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState( getDriveVelocity(), new Rotation2d( getTurningPosition() ) );
  }

  public void setDesiredState(SwerveModuleState state) {

    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
    SmartDashboard.putString("Swerve Module[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop() {
    driveMotor.set(0);
    turningPidController.setReference(turningEncoder.getPosition(), ControlType.kPosition);
  }
}
 