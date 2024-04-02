// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class TrajectoryHelper {

    public static Translation2d toAllianceRelativePosition(Translation2d position) {
        Optional<Alliance> allianceTeam = DriverStation.getAlliance();
        if (allianceTeam.isPresent() && allianceTeam.get() == Alliance.Red) {
            return new Translation2d(DriveConstants.kFieldWidthMeters - position.getX(), position.getY());
        }
        return position;
    }

    public static Pose2d toAllianceRelativePosition(Pose2d position) {
        Optional<Alliance> allianceTeam = DriverStation.getAlliance();
        if (allianceTeam.isPresent() && allianceTeam.get() == Alliance.Red) {
            Rotation2d rotationOfPosition = position.getRotation();
            return new Pose2d(
                DriveConstants.kFieldWidthMeters - position.getX(), 
                position.getY(), 
                new Rotation2d(
                    Math.atan2(
                        rotationOfPosition.getSin(), 
                        -rotationOfPosition.getCos()
                    )
                )
            );
        }
        return position;
    }

    public static Trajectory createTrajectoryWithAllianceRelativePositioning(Pose2d init, List<Translation2d> points, Pose2d end) {
        List<Translation2d> newPointList = new ArrayList<Translation2d>();
        for (int i = 0; i < points.size(); i++) {
            
            newPointList.add( 
                toAllianceRelativePosition(
                    points.get(i)
                )
            );
        }
        return TrajectoryGenerator.generateTrajectory(
            toAllianceRelativePosition(init), 
            newPointList, 
            toAllianceRelativePosition(end), 
            Constants.kTrajectoryConfig
        );
    }

    private TrajectoryHelper() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
