// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TaynesIntake extends SubsystemBase {
  //public static WPI_TalonSRX taynesIntake;
  public static TalonSRX taynesIntake;
  /** Creates a new TaynesIntake. */
  public TaynesIntake() {
    //WPI_TalonSRX taynesIntake = new WPI_TalonSRX(Constants.taynesIntake);
    //taynesIntake = new WPI_TalonSRX(Constants.taynesIntake);
    taynesIntake = new TalonSRX(Constants.taynesIntake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void stop() {
    //taynesIntake.stopMotor();
    //taynesIntake.set(ControlMode.PercentOutput, 0);
  }
}
