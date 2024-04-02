// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;


public class SimulatedSwerveModule extends SubsystemBase implements SwerveModuleInterface {
  /** Creates a new ExampleSubsystem. */

  //For simulation
  private double simulatedWheelSpeedMetersPerSecond = 0;
  private double simulatedWheelPositionMeters = 0;
  private double simulatedWheelAngleRadians = 0;
  private double simulatedWheelAngleVelocityRadiansPerSecond = 0;
  private Timer simulatedTimeUpdate;
  private int encoderDeviceID;

  public SimulatedSwerveModule(SwerveModuleConstants swerveModuleConstants) {
    resetEncoders();
    simulatedTimeUpdate = new Timer();
    simulatedTimeUpdate.start();
    encoderDeviceID = swerveModuleConstants.kTurningEncoderID-8;
  }

  //This for the odometer specifically
  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition( getDrivePosition(), new Rotation2d( getTurningPosition() ));
  }

  //Returns wheel position in meters
  public double getDrivePosition() {
      return simulatedWheelPositionMeters;
  }
  //returns wheel direction in radians
  public double getTurningPosition() {
      return simulatedWheelAngleRadians;
  }
  //returns wheel velocity in meters per second
  public double getDriveVelocity() {
      return simulatedWheelSpeedMetersPerSecond;
  }
  //returns wheel direction velocity in radians per second
  public double getTurningVelocity() {
      return simulatedWheelAngleVelocityRadiansPerSecond;
  }

  //Resets the encoders, sets drive to zero and sets the turning to its current position relative to the robot's forward direction
  public void resetEncoders() {
    simulatedWheelPositionMeters = 0;
  }

  //Returns the state of the module with velocity and direction
  public SwerveModuleState getState() {
    return new SwerveModuleState( getDriveVelocity(), new Rotation2d( getTurningPosition() ));
  }

  //Sets the module to a drive velocity and directional position
  public void setDesiredState(SwerveModuleState state) {    
    SmartDashboard.putString("Module [" + encoderDeviceID + "] state", state.toString());

    simulatedTimeUpdate.start();

    simulatedWheelPositionMeters += simulatedWheelSpeedMetersPerSecond*simulatedTimeUpdate.get();

    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    simulatedWheelSpeedMetersPerSecond = state.speedMetersPerSecond;

    simulatedWheelAngleVelocityRadiansPerSecond = 
      (state.angle.getRadians() - simulatedWheelAngleVelocityRadiansPerSecond) / 
      simulatedTimeUpdate.get();

    if (Double.isNaN(simulatedWheelAngleVelocityRadiansPerSecond)) {
      simulatedWheelAngleVelocityRadiansPerSecond = 0;
    }
    simulatedWheelAngleRadians = state.angle.getRadians();

    simulatedTimeUpdate.reset();
  }

  //Stops the motor from moving and turning
  public void stop() {
    simulatedWheelAngleVelocityRadiansPerSecond = 0;
    simulatedWheelSpeedMetersPerSecond = 0;
    simulatedTimeUpdate.stop();
  }
}
 
