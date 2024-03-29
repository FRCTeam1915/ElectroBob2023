// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class AutonomousOne extends SequentialCommandGroup {
  //Creates a new AutonomousOne. 
  //Robot goes forward abrubtly dropping game piece, goes back to push into cell, run out of community
  public AutonomousOne(DriveSubsystem driveTrain, Shooter s) {
    addCommands(new driveDirection(driveTrain, 1, 1));
    addCommands(new driveDirection(driveTrain, 1.2, -1));
    addCommands(new driveDirection(driveTrain, 5, 1));
//    addCommands(new DriveForwardTimed(driveTrain), new AutoShoot(s));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
}


