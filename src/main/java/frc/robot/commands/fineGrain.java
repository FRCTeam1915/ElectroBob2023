// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  boolean bbutton;
  double addSpeed;

public fineGrain(Intake i, double tuppHigh, double tuppLow, boolean tbbutton){
  intake = i;
  uppLow = tuppLow;
  uppHigh = tuppHigh;
  bbutton = tbbutton;
  
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
    if(bbutton == true){
      addSpeed = .5;
      SmartDashboard.putBoolean("B button", bbutton);
    }
    else{
      SmartDashboard.putBoolean("B button", bbutton);
      addSpeed = 0;
    }
    if(uppLow < 0) {
      Intake.m_pid_low.setP(0.1);
      Intake.m_pid_low.setI(0);
      Intake.m_pid_low.setD(0);
      Intake.m_pid_low.setReference(Position_LOW, CANSparkMax.ControlType.kPosition);
      Intake.m_pid_low.setReference(Position_LOW + Constants.Intake.fineGrainDistance + addSpeed, CANSparkMax.ControlType.kPosition);
      SmartDashboard.putBoolean("B button", bbutton);
      
    }
    else if(uppLow > 0) {
      Intake.m_pid_low.setP(.1);
      Intake.m_pid_low.setI(0);
      Intake.m_pid_low.setD(0);
      Intake.m_pid_low.setReference(Position_LOW, CANSparkMax.ControlType.kPosition);
      Intake.m_pid_low.setReference(Position_LOW - Constants.Intake.fineGrainDistance + addSpeed, CANSparkMax.ControlType.kPosition);
      SmartDashboard.putBoolean("B button", bbutton);
    }
    

    /*Position_HIGH = Intake.m_encoder_high.getPosition();
    //If uppHigh is greater < 0, move up, elif > 0, move down, else stay still
    if(uppHigh < 0) {
      Intake.m_pid_high.setP(.1);
      Intake.m_pid_high.setI(0);
      Intake.m_pid_high.setD(0);
      Intake.m_pid_high.setReference(Position_HIGH, CANSparkMax.ControlType.kPosition);
      Intake.m_pid_high.setReference(Position_HIGH + Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
     
    }
    else if(uppHigh > 0) {
      Intake.m_pid_high.setP(.1);
      Intake.m_pid_high.setI(0);
      Intake.m_pid_high.setD(0);
      Intake.m_pid_high.setReference(Position_HIGH, CANSparkMax.ControlType.kPosition);
      Intake.m_pid_high.setReference(Position_HIGH - Constants.Intake.fineGrainDistance, CANSparkMax.ControlType.kPosition);
      
    }
    */
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
