// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModuleInterface {
    public SwerveModulePosition getSwervePosition();
    public double getDrivePosition();
    public double getTurningPosition();
    public double getDriveVelocity();
    public double getTurningVelocity();
    public void resetEncoders();
    public SwerveModuleState getState();
    public void setDesiredState(SwerveModuleState state);
    public void stop();
}
