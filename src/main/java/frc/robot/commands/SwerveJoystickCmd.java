// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

private final SwerveSubsystem swerveSubsystem;
private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
private Supplier<Boolean> fieldOrientedFunction;
private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
private final Timer deltaTime = new Timer();

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private GenericEntry robotSpeedPercent = teleOpTab.addPersistent("Robot Speed Percent", DriveConstants.kTeleMaxDriveSpeedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

  this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxDriveAccelerationUnitsPerSecond);
  this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxDriveAccelerationUnitsPerSecond);
  this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxTurningAccelerationUnitsPerSecond);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Used for discret time updating
    deltaTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Uses the suppliers to get the current joystick positions
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    int teamBasedXAxisMult = 1;
    int teamBasedYAxisMult = 1;
    Optional<Alliance> currentTeam = DriverStation.getAlliance();
    if (currentTeam.isPresent() && currentTeam.get() == Alliance.Red) {
      teamBasedXAxisMult = -1;
      teamBasedYAxisMult = -1;
    }

    xSpeed *= teamBasedXAxisMult;
    ySpeed *= teamBasedYAxisMult;

    //Makes sure that the joysticks are not giving small values when not being touched
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    //Makes the joystick inputs change over time so the robot doesnt speed up too fast
    xSpeed = xLimiter.calculate(xSpeed) * robotSpeedPercent.getDouble(1)*DriveConstants.kTeleMaxDriveSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * robotSpeedPercent.getDouble(1)*DriveConstants.kTeleMaxDriveSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleMaxTurningSpeedRadiansPerSecond;

    SmartDashboard.putBoolean("Field Oriented Drive", fieldOrientedFunction.get());

    //Used for swerve kinematics
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    if (fieldOrientedFunction.get()) {
      //Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, swerveSubsystem.getRotation2d());

    }
    //Makes the chassis speeds use the change in time instead of blind guesses to prevent position skewing
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, deltaTime.get());
    //Resets the time to 0 to find the next deltaTime
    deltaTime.reset();
    //Solves for the wheel velocities and directions based on the joystick inputs
    SwerveModuleState[] moduleStates = WheelConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //Sets the state of each module
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops robot from moving when teleOp mode ends
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
