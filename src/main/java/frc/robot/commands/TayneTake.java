// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TaynesIntake;

public class TayneTake extends CommandBase {
  boolean inn;
  TaynesIntake tayke;
  // Creates a new tayneTake.
  public TayneTake(TaynesIntake ttayke, boolean tinn) {
    inn = tinn;
    tayke = ttayke;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tayke);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, 0.5);
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
