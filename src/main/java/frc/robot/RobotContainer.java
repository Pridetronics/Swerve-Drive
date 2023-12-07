// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.commands.SwerveAutoPaths;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final SendableChooser<Trajectory> autoCommandChooser = new SendableChooser<>();
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoCommandChooser.setDefaultOption("Do Nothing", null);
    // autoCommandChooser.addOption("test auto", SwerveAutoPaths.TestAutoPath());
    // autoCommandChooser.addOption("weird path", SwerveAutoPaths.WeirdPath());
    autoCommandChooser.addOption("Forward Right", SwerveAutoPaths.ForwardRight());

    SmartDashboard.putData("Autonomous Mode", autoCommandChooser);


    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickTurningAxis),
        () -> !driverJoystick.getRawButton(IOConstants.kDriveFieldOrientedDriveBtnID)
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(driverJoystick, IOConstants.kZeroHeadingBtnID)
    .onTrue(new ZeroRobotHeading(swerveSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Trajectory chosenTrajectory = autoCommandChooser.getSelected();
    if (chosenTrajectory == null) return null;

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveAutoCommand = new SwerveControllerCommand(
    chosenTrajectory, 
    swerveSubsystem::getPose, 
    WheelConstants.kDriveKinematics,
    xController,
    yController,
    thetaController,
    swerveSubsystem::setModuleStates,
    swerveSubsystem);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(chosenTrajectory.getInitialPose())),
      swerveAutoCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
