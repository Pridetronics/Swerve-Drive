// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public final class SwerveAutoPaths {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  //public static Trajectory TestAutoPath() {
  //   Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0, 0, new Rotation2d(0)), 
  //     List.of(
  //       new Translation2d(1, 0),
  //       new Translation2d(1, 1),
  //       new Translation2d(2, -2),
  //       new Translation2d(1, -3),
  //       new Translation2d(0, 0)
  //     ),
  //     new Pose2d(0, 0, new Rotation2d(1080)), 
  //     Constants.kTrajectoryConfig);

  //   return trajectory1;
  // }

  // public static Trajectory WeirdPath() {
  //   Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0, 0, new Rotation2d(0)), 
  //     List.of(
  //       new Translation2d(2, 0),
  //       new Translation2d(1, -2),
  //       new Translation2d(1, 2),
  //       new Translation2d(0, -2),
  //       new Translation2d(0, 0)
  //     ),
  //     new Pose2d(0, 0, new Rotation2d(1080)), 
  //     Constants.kTrajectoryConfig);

  //   return trajectory1;
  // }

  public static Trajectory ForwardRight() {
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(1, 0),
        new Translation2d(1, 1),
        new Translation2d(1, -1),
        new Translation2d(0, -1),
        new Translation2d(0, 1)
      ),
      new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
      Constants.kTrajectoryConfig);

    return trajectory1;
  }

  private SwerveAutoPaths() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
