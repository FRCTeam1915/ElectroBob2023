// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardTimed extends CommandBase {
  DriveSubsystem driveTrain;
  private boolean finish = false;
  Timer timer;
  double drivetime;
  
  
  /** Creates a new DriveForawrdTimed. */
  public DriveForwardTimed(DriveSubsystem dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    timer = new Timer();
    drivetime = 5.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Autonomous going in a straight line to leave community
    //drivetime = RobotContainer.chooserA.getSelected();

    //System.out.println("about to call drive forward for time");
    //System.out.println(drivetime);

    timer.reset();
    timer.start();

    //while(timer.get() < Constants.DRIVE_FORWARD_TIME)
    
    while(timer.get() < Constants.DRIVE_FORWARD_TIME)
    {
      //System.out.println("driving forward for");
      //System.out.println(drivetime);
      driveTrain.drive(Constants.AUTONOMOUS_SPEED, 0, 0, true, false);
      
    }
    driveTrain.drive(0, 0, 0, true, false);
    finish = true;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*     if(timer.get() < Constants.DRIVE_FORWARD_TIME)
    {
      //System.out.println("driving forward for");
      //System.out.println(drivetime);
      driveTrain.drive(Constants.AUTONOMOUS_SPEED, 0, 0, true, false);
      
    } else {
      driveTrain.drive(0, 0, 0, true, false);
      //finish = true;
    }
*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
