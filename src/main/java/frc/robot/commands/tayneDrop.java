// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TaynesIntake;

public class tayneDrop extends CommandBase {
  boolean inn;
  TaynesIntake tayke;
  Timer timer;
  /** Creates a new tayneTake. */
  public tayneDrop(TaynesIntake ttayke, boolean tinn) {
    inn = tinn;
    tayke = ttayke;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tayke);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //timer.reset();
    //timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //while(timer.get() < 2) {
    //new tayneTakeIn(tayke, false);
    //  if(inn = true) {
    //    TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, 0.1);
    //  } else {
        TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, -0.5);
    //  }
    //}
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
