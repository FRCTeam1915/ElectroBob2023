// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// exmaples taken from 
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Encoder%20Feedback%20Device/src/main/java/frc/robot/Robot.java

package frc.robot.subsystems;

import javax.swing.RowFilter.ComparisonType;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  Spark intake;
  private CANSparkMax m_motor_low, m_motor_high;
  public static SparkMaxPIDController m_pid_low;
  public static SparkMaxPIDController m_pid_high;

  public static RelativeEncoder m_encoder_low;
  public static RelativeEncoder m_encoder_high;
  private double target_position_low, target_position_high;

  double Position_LOW, Position_HIGH;

  /** Creates a new Intake. */
  public Intake() {
    intake = new Spark(Constants.INTAKE);
    m_motor_low = new CANSparkMax(Constants.Intake.CAN_low, MotorType.kBrushless);
    m_motor_high = new CANSparkMax(Constants.Intake.CAN_high, MotorType.kBrushless);
    m_motor_low.setIdleMode(IdleMode.kBrake);
//    m_encoder_low = m_motor_low.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature,40960);
//    m_encoder_high = m_motor_high.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature,4096);
    m_encoder_low = m_motor_low.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    m_encoder_high = m_motor_high.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);

    m_encoder_low.getPosition();
    //SmartDashboard.putNumber("ProcessVariable", m_encoder_low.getPosition());
    //private CANEncoder encoder = new CANEncoder(motor);
    //encoder.getPosition();

    m_encoder_low.setPosition(0);
    m_encoder_high.setPosition(0);

    m_motor_low.restoreFactoryDefaults();
    m_motor_high.restoreFactoryDefaults();

    m_pid_low = m_motor_low.getPIDController();
    m_pid_high = m_motor_high.getPIDController();

    m_pid_low.setFeedbackDevice(m_encoder_low);
    m_pid_high.setFeedbackDevice(m_encoder_high);


    m_pid_low.setP(Constants.Intake.kPlow);
    m_pid_low.setI(Constants.Intake.kIlow);
    m_pid_low.setD(Constants.Intake.kDlow);

    m_pid_high.setP(Constants.Intake.kPhigh);
    m_pid_high.setI(Constants.Intake.kIhigh);
    m_pid_high.setD(Constants.Intake.kDhigh);

    //target_position_low = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("m_motor_low Position", m_encoder_low.getPosition());
    //SmartDashboard.putNumber("m_motor_low Target Position", target_position_low);
    SmartDashboard.putNumber("m_motor_high Position", m_encoder_high.getPosition());
    //SmartDashboard.putNumber("m_motor_high Target Position", target_position_high);
  }

//  public void set_position_A() {
//    m_pid_low.setReference(Constants.Intake.Position_A_LOW, CANSparkMax.ControlType.kPosition);
//  }


  // Move both the lower and upper arm to the desired position
  public void setArmPosition(double lower,double upper) {
    Position_LOW = Intake.m_encoder_low.getPosition();
    Position_HIGH = Intake.m_encoder_high.getPosition();

//    double tempLow = Position_LOW + (lower * Constants.Intake.fineGrainDistance);
//    double tempHigh = Position_HIGH + (upper * Constants.Intake.fineGrainDistance);
    double tempLow = Position_LOW + (lower * 3.5);
    double tempHigh = Position_HIGH + (upper * 3.5);

    System.out.println("Setting Arm Position low = " + tempLow);
    System.out.println("Setting Arm Position high = " + tempHigh);
    

    Intake.m_pid_low.setReference(tempLow, CANSparkMax.ControlType.kPosition);
    Intake.m_pid_high.setReference(tempHigh, CANSparkMax.ControlType.kPosition);

    //Intake.m_pid_low.setReference(-10.0, CANSparkMax.ControlType.kPosition);
  }

  public void intakeBall(XboxController controller, double speed)
  {
    //m_motor_low.set(controller.getRightTriggerAxis()*speed);
    //if (controller.getRightBumperReleased())
    //{
    //  target_position += 30;
    //}
      
    //m_pid_low.setReference(target_position_low, CANSparkMax.ControlType.kPosition);
    
    //SmartDashboard.putNumber("m_motor_low Position", m_encoder_low.getPosition());
    //SmartDashboard.putNumber("m_motor_low Target Position", target_position_low);
    //SmartDashboard.putNumber("m_motor_high Position", m_encoder_high.getPosition());
    //SmartDashboard.putNumber("m_motor_high Target Position", target_position_high);
  }

  public void stop()
  {
    m_motor_low.stopMotor();
  }
}