// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Instant command only initializes, no periodic updating
public class ZeroRobotHeading extends InstantCommand {

  //Used for debounce
  private Timer timer = new Timer();

  private final SwerveSubsystem swerveSubsystem;
  public ZeroRobotHeading(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    //TODO ^^^^
    
    this.swerveSubsystem = swerveSubsystem;
    //Starts timer for later cooldown activation
    timer.start();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Runs when button is pressed

    //Checks debounce
    if (timer.hasElapsed(IOConstants.kZeroHeadingDebounceTime)) {
      //resets heading
      swerveSubsystem.zeroHeading();
      //resets cooldown
      timer.reset();
    } 
  }
}
