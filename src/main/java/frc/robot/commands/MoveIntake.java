// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase {
  Intake intake;
  double Position_LOW;
  double Position_HIGH;
  //Creates a new IntakeBall.
  public MoveIntake(Intake i, 
                    double tPosition_LOW,
                    double tPosition_HIGH) {
    intake = i;
    Position_LOW = tPosition_LOW;
    Position_HIGH = tPosition_HIGH;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.intakeBall(RobotContainer.driverJoystick, Constants.INTAKE_SPEED);
    Intake.m_pid_low.setReference(Position_LOW, CANSparkMax.ControlType.kPosition);
    Intake.m_pid_high.setReference(Position_HIGH, CANSparkMax.ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}

