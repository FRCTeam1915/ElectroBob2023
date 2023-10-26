// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TaynesIntake;

public class holdButton extends CommandBase {
  boolean inn;
  boolean holdButton;
  boolean tayneTake;
  TaynesIntake tayke;
  // Creates a new tayneTake.
  public holdButton(TaynesIntake ttayke, boolean tinn) {
    holdButton = tinn;
    tayke = ttayke;
    addRequirements(tayke);
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(tayneTake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (holdButton == true) {
      SmartDashboard.putBoolean("A button", holdButton);
      TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, -0.2);

    } else { 
      SmartDashboard.putBoolean("A button", holdButton);
      TaynesIntake.taynesIntake.set(ControlMode.PercentOutput, 0);
    }
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
