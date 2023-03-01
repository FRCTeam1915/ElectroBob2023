// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
//import frc.robot.Constants.Intake;
import frc.robot.subsystems.Intake;



  
public class fineGrain extends CommandBase {
  double Position_LOW;
  double Position_HIGH;
  double uppHigh;
  double uppLow;
  Intake intake;

public fineGrain(Intake i, double tuppHigh, double tuppLow){
  intake = i;
  uppLow = tuppLow;
  uppHigh = tuppHigh;
  
  /** Creates a new fineGrain. */

    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(intake);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Position_LOW = Intake.m_encoder_low.getPosition();
    //If uppLow is greater < 0, move up, elif > 0, move down, else stay still
    if(uppLow < 0) {
      Intake.m_pid_low.setReference(Position_LOW + Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
    }
    else if(uppLow > 0) {
      Intake.m_pid_low.setReference(Position_LOW - Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
    }

    Position_HIGH = Intake.m_encoder_high.getPosition();
    //If uppHigh is greater < 0, move up, elif > 0, move down, else stay still
    if(uppHigh < 0) {
      Intake.m_pid_high.setReference(Position_HIGH + Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
    }
    else if(uppHigh > 0) {
      Intake.m_pid_high.setReference(Position_HIGH - Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
