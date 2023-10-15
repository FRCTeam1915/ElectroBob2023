// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.AutoShoot;
//import frc.robot.commands.AutonomousOne;
//import frc.robot.commands.AutonomousTwo;
import frc.robot.commands.Button;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.FineGrain;
import frc.robot.commands.GyroReset;
import frc.robot.commands.TayneDrop;
import frc.robot.commands.TayneShoot;
import frc.robot.commands.TayneTake;
import frc.robot.commands.xForm;
//import frc.robot.commands.tayneTakeIn;
//import frc.robot.commands.tayneTakeOut;
//import frc.robot.commands.IntakeBall;
//import frc.robot.commands.ShootBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TaynesIntake;
//import frc.robot.subsystems.Shooter;
import frc.robot.commands.driveDirection;
import frc.robot.commands.DriveToDistance;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final driveDirection driveForwardTimed;
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

  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  public static SendableChooser<Double> chooserA = new SendableChooser<>();

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
              Math.max(0.0, (Math.abs(m_driverController.getLeftY())-OIConstants.kDriveDeadband)/(1.0-OIConstants.kDriveDeadband)) * Math.signum(-m_driverController.getLeftY()),
              Math.max(0.0, (Math.abs(m_driverController.getLeftX())-OIConstants.kDriveDeadband)/(1.0-OIConstants.kDriveDeadband)) * Math.signum(-m_driverController.getLeftX()),
              Math.max(0.0, (Math.abs(m_driverController.getRightX())-OIConstants.kDriveDeadband)/(1.0-OIConstants.kDriveDeadband)) * Math.signum(-m_driverController.getRightX()),
              true, true),driveTrain));

    intake = new Intake();
    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.setArmPosition(
            m_IntakeController.getLeftY(), m_IntakeController.getRightY()), intake)
    );
    

    driveForwardTimed = new driveDirection(driveTrain, 0, 0);
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

    tayke = new TaynesIntake();
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);
    //intake.setDefaultCommand(intakeBall);

/*    //Autonomous Mode
    autonomousOne = new AutonomousOne(driveTrain, shooter);
    autonomousTwo =  new AutonomousTwo(driveTrain, shooter);

    
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

    
    SmartDashboard.putData("Autonomous Run Time", chooserA);
    chooserA.setDefaultOption("0 seconds", 0.0);
    chooserA.setDefaultOption("2 seconds", 2.0);
    chooserA.addOption("5 seconds", 5.0);
    chooserA.addOption("8 seconds", 8.0);
    chooserA.addOption("10 seconds", 10.0);
    //chooserB.setDefaultOption("Autonomous One", 8000.0);

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

    //Trigger leftBumper = m_IntakeController.leftBumper();  //leftBumper
    //leftBumper.whileTrue(new fineGrain(intake, 1, 0));

    //Trigger rightBumper = m_IntakeController.rightBumper(); //rightBumper button
    //rightBumper.whileTrue(new fineGrain(intake, -1, 0));

    

    Trigger povUp = m_IntakeController.povUp(); //changed button povUp
    povUp.whileTrue(new FineGrain(intake, 0, 1,false));

    Trigger povDown = m_IntakeController.povDown(); //povDown
    povDown.whileTrue(new FineGrain(intake, 0, -1,false));

    //Trigger bbutton = m_driverController.b();
    Trigger rightBumper = m_IntakeController.rightBumper();
    rightBumper.whileTrue(new TayneTake(tayke, true).repeatedly());

    Trigger bbutton = m_IntakeController.b();
    bbutton.onTrue(new Button(true).repeatedly());
    bbutton.onFalse(new Button(false).repeatedly());
    
    //Trigger ybutton = m_driverController.y();
    Trigger leftBumper = m_IntakeController.leftBumper();
    leftBumper.whileTrue(new TayneDrop(tayke, true).repeatedly());

    Trigger aButton = m_IntakeController.a();
    aButton.whileTrue(new TayneShoot(tayke, true).repeatedly());
    //aButton.onTrue(new DriveToDistance(driveTrain).repeatedly());

    Trigger xbutton = m_driverController.x();
    xbutton.whileTrue(new xForm(driveTrain));

    Trigger abutton = m_driverController.a();
    abutton.whileTrue(new GyroReset(driveTrain));
    

    // JoystickButton aButton = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    // aButton.whenPressed(new DriveToDistance(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    /*
          // Create config for trajectory
          TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            driveTrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);
        // Reset odometry to the starting pose of the trajectory.
        driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    

        // Run path following command, then stop at the end.
        swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false, false));
*/

    // An example command will be run in autonomous
    //return chooser.getSelected();

    
    //return chooserA.getSelected();

    //chooserA.addOption("Autonomous Two", 4.0);
    //chooserB.setDefaultOption("Autonomous One", 8000.0);

    //System.out.println("Current Dashboard value");
    //System.out.println(chooserA.getSelected());

    //return new AutonomousOne(driveTrain, null);

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
  */
    //Leave community
   return new DriveForwardTimed(driveTrain);

  


  }
}
