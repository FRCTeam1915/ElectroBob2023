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

public class driveDirection extends CommandBase {
  double thyme;
  double direction;
  DriveSubsystem driveTrain;
  private boolean finish = false;
  Timer timer;
  double drivetime;
  
  
  //Creates a new DriveForawrdTimed. 
  public driveDirection(DriveSubsystem dt, double dy, double dz) {
    thyme = dy;
    direction = dz; //1 forward, -1 Backward
    driveTrain = dt;
    addRequirements(driveTrain);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    timer.reset();
    timer.start();

    while(timer.get() < drivetime);
    //while(timer.get() < Constants.DRIVE_FORWARD_TIME)
    {

      //driveTrain.drive(Constants.AUTONOMOUS_SPEED, 0, 0, true, finish);
    }
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
