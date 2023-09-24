// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.SwerveModuleClasses;

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

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
