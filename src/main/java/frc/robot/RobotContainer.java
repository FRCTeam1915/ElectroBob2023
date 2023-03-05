// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonomousOne;
import frc.robot.commands.AutonomousTwo;
//import frc.robot.commands.AutoShoot;
//import frc.robot.commands.AutonomousOne;
//import frc.robot.commands.AutonomousTwo;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.fineGrain;
import frc.robot.commands.tayneDrop;
import frc.robot.commands.tayneTake;
import frc.robot.commands.xForm;
//import frc.robot.commands.tayneTakeIn;
//import frc.robot.commands.tayneTakeOut;
//import frc.robot.commands.IntakeBall;
//import frc.robot.commands.ShootBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TaynesIntake;
//import frc.robot.subsystems.Shooter;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveToDistance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private boolean m_reset = false;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_IntakeController =
      new CommandXboxController(OperatorConstants.kIntakeControllerPort);

  private final DriveSubsystem driveTrain = new DriveSubsystem();
 // XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


//  private final DriveTrain driveTrain; //differential drive
  //private final DriveWithJoysticks driveWithJoystick;
  private final DriveForwardTimed driveForwardTimed;
  private final DriveToDistance driveToDistance;
  public static XboxController driverJoystick;

  //private final Shooter shooter;
  //private final ShootBall shootBall;
  //private final AutoShoot autoShoot;

  private final Intake intake;
  private final TaynesIntake tayke;
  private final IntakeBall intakeBall;

  //private final AutonomousOne autonomousOne;
  //private final AutonomousTwo autonomousTwo;

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /**
    driveTrain = new DriveTrain();
    driveWithJoystick = new DriveWithJoysticks(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick);
    */
    driveTrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveTrain.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            driveTrain));

    

    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    driveToDistance = new DriveToDistance(driveTrain);
    driveToDistance.addRequirements(driveTrain);

    driverJoystick = new XboxController(Constants.JOYSTICK_NUMBER); //

    /**
    shooter = new Shooter();
    shootBall = new ShootBall(shooter);
    shootBall.addRequirements(shooter);

    autoShoot = new AutoShoot(shooter);
    autoShoot.addRequirements(shooter);
    */

    intake = new Intake();
    tayke = new TaynesIntake();
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);
    intake.setDefaultCommand(intakeBall);

/**     //Autonomous Mode
    autonomousOne = new AutonomousOne(driveTrain, shooter);
    autonomousTwo =  new AutonomousTwo(driveTrain, shooter);

    chooser.addOption("Autonomous Two", autonomousTwo);
    chooser.setDefaultOption("Autonomous One", autonomousOne);
    SmartDashboard.putData("Autonomous", chooser);
*/
/*
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(Constants.CAMERA_RES_X, Constants.CAMERA_RES_Y);
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(Constants.CAMERA_RES_X, Constants.CAMERA_RES_Y);
*/
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary 
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* 
    Trigger xButton = m_IntakeController.x();
    xButton.onTrue(new MoveIntake(intake,
                                  Constants.Intake.A.Position_LOW,
                                  Constants.Intake.A.Position_HIGH,
                                  Constants.Intake.A.kPlow,
                                  Constants.Intake.A.kPhigh,
                                  Constants.Intake.A.kIlow,
                                  Constants.Intake.A.kIhigh,
                                  Constants.Intake.A.kDlow,
                                  Constants.Intake.A.kDhigh));

    Trigger yButton = m_IntakeController.y();
    yButton.onTrue(new MoveIntake(intake,
                                  Constants.Intake.B.Position_LOW,
                                  Constants.Intake.B.Position_HIGH,
                                  Constants.Intake.B.kPlow,
                                  Constants.Intake.B.kPhigh,
                                  Constants.Intake.B.kIlow,
                                  Constants.Intake.B.kIhigh,
                                  Constants.Intake.B.kDlow,
                                  Constants.Intake.B.kDhigh));
                              
    Trigger bButton = m_IntakeController.b();
    bButton.onTrue(new MoveIntake(intake, 
                                  Constants.Intake.C.Position_LOW,
                                  Constants.Intake.C.Position_High,
                                  Constants.Intake.C.kPlow,
                                  Constants.Intake.C.kPhigh,
                                  Constants.Intake.C.kIlow,
                                  Constants.Intake.C.kIhigh,
                                  Constants.Intake.C.kDlow,
                                  Constants.Intake.C.kDhigh));
*/
    //Trigger rightBumper = m_driverController.rightBumper();
    //rightBumper.onTrue(new ShootBall(shooter).repeatedly());

    Trigger leftBumper = m_IntakeController.leftBumper();  //leftBumper
    leftBumper.whileTrue(new fineGrain(intake, 1, 0));

    Trigger rightBumper = m_IntakeController.rightBumper(); //rightBumper button
    rightBumper.whileTrue(new fineGrain(intake, -1, 0));

    Trigger povUp = m_IntakeController.povUp(); //changed button povUp
    povUp.whileTrue(new fineGrain(intake, 0, 1));

    Trigger povDown = m_IntakeController.povDown(); //povDown
    povDown.whileTrue(new fineGrain(intake, 0, -1));

    Trigger bbutton = m_driverController.b();
    bbutton.whileTrue(new tayneTake(tayke, true).repeatedly());
    
    Trigger ybutton = m_driverController.y();
    ybutton.whileTrue(new tayneDrop(tayke, true).repeatedly());
    
    Trigger aButton = m_driverController.a();
    aButton.onTrue(new DriveToDistance(driveTrain).repeatedly());

    Trigger xbutton = m_driverController.x();
    xbutton.whileTrue(new xForm(driveTrain));

    // JoystickButton aButton = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    // aButton.whenPressed(new DriveToDistance(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command  getAutonomousCommand() {
    // An example command will be run in autonomous
    //return chooser.getSelected();

    return new AutonomousOne(driveTrain, null);

    /*
    return new RunCommand(
            () -> driveTrain.drive(
                5,
                0,
                0,
                true),
            driveTrain)
            .andThen(new WaitCommand(1))
            .andThen(() -> driveTrain.drive(0,0,0,true));
    */
    /*
    //move to position A
    new MoveIntake(intake,
      Constants.Intake.A.Position_LOW,
      Constants.Intake.A.Position_HIGH,
      Constants.Intake.A.kPlow,
      Constants.Intake.A.kPhigh,
      Constants.Intake.A.kIlow,
      Constants.Intake.A.kIhigh,
      Constants.Intake.A.kDlow,
      Constants.Intake.A.kDhigh);

    //Place game piece
    new tayneDrop(tayke, false);
  
    //Leave community
   // new DriveForwardTimed(DriveSubsystem);

  */ 
  }
}
