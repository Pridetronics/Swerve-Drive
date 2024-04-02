// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.shuffleboardConstants;

/** Add your docs here. */
public class ShuffleboardRateLimiter {
    //Timer to track updating
    private static Timer rateLimitTimer = new Timer();
    //Stores a list of data to update
    private static HashMap<String, Runnable> shuffleboardUpdates = new HashMap<String, Runnable>();

    public static void periodic() {
        rateLimitTimer.start();
        //only updates the shuffleboard if the specified period has passed
        if (rateLimitTimer.advanceIfElapsed(shuffleboardConstants.kRateLimitTime)) {
            //Go through all the updates to be made in the hash map, and run a runnable method that is set to update the specified entry
            for (String index : shuffleboardUpdates.keySet()) {
                Runnable updateRunnable = shuffleboardUpdates.get(index);
                //Runs the runnable method
                updateRunnable.run();
            }
            //Resets the hash map to make sure only new data is posted on the next update
            shuffleboardUpdates.clear();
        }
    }

    public static void queueDataForShuffleboard(GenericEntry entry, Object value) {
        //Adds a new runnable method that updates the shuffleboard entry (with the location of the runnable in the hashmap being the name of the entry)
        shuffleboardUpdates.put(
            entry.getTopic().getName(), 
            () -> {
                entry.setValue(value);
            }
        );
    }
}
