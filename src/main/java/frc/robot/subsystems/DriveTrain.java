// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
/**
  CANSparkMax leftFront;
  CANSparkMax rightFront;
  CANSparkMax leftBack;
  CANSparkMax rightBack;
*/
  WPI_TalonSRX rightFront;
  WPI_TalonSRX rightBack;
  WPI_TalonSRX leftFront;
  WPI_TalonSRX leftBack;

  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive;
  private final AnalogInput rangeFinder;

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    /**
    leftFront = new Spark(Constants.LEFT_FRONT);
    leftFront.setInverted(false);
    rightFront = new Spark(Constants.RIGHT_FRONT);
    rightFront.setInverted(false);
    leftBack = new Spark(Constants.LEFT_BACK);
    leftBack.setInverted(false);
    rightBack = new Spark(Constants.RIGHT_BACK);
    rightBack.setInverted(false);
    

    leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushed);
    leftFront.setInverted(false);
    rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushed);
    rightFront.setInverted(false);
    leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
    leftBack.setInverted(false);
    rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
    rightBack.setInverted(false);
    */
    
    leftMotors = new MotorControllerGroup(leftFront,leftBack);
    rightMotors = new MotorControllerGroup(rightFront,rightBack);
    

    leftFront = new WPI_TalonSRX(5);    
    rightFront = new WPI_TalonSRX(8);    
    
    leftMotors = new MotorControllerGroup(leftFront);
    rightMotors = new MotorControllerGroup(rightFront);


    drive = new DifferentialDrive(leftMotors,rightMotors);
    rangeFinder = new AnalogInput(Constants.RANGE_FINDER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks(XboxController controller, double speed)
  {
  drive.arcadeDrive(controller.getLeftY()*speed, controller.getLeftX()*speed);
  }

  public void driveForward(double speed)
  {
    drive.tankDrive(speed,speed);
  }

  public boolean driveToDistance(double setPointdistance, double speed)
  {
    while(rangeFinder.getAverageVoltage() > setPointdistance)
    {
      driveForward(speed);
    }
    return true;
  }

  public void stop()
  {
    drive.stopMotor();
  }
}
