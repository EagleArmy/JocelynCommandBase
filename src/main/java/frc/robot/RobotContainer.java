// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.ForwardArmPositionCmd;
import frc.robot.commands.MaxVelocityCmd;
import frc.robot.commands.SendToAMPCmd;
import frc.robot.commands.SendToShooterCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.BackwardArmPositionCmd;
import frc.robot.commands.DriveForTimeCmd;

import frc.robot.subsystems.AMPSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MaxVelocitySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PositionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.GenericHID;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AMPSubsystem m_AMP = new AMPSubsystem();
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private final PositionSubsystem m_Position = new PositionSubsystem();
  private final MaxVelocitySubsystem m_maxvSubsystem = new MaxVelocitySubsystem();
  private ShuffleboardTab m_Tab;


  // The driver's controller
  XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort1);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    applyDeadband( m_driverController1.getLeftY(), OIConstants.deadBand ),    
                    applyDeadband( 0, 0 ),
                    //applyDeadband( m_driverController1.getLeftX(), OIConstants.deadBand ),
                    applyDeadband( -(m_driverController1.getRightX())/2, OIConstants.deadBand ),
                    false,
                    m_robotDrive.getDirectionStatus()),
            m_robotDrive));

            // m_robotDrive.drive(
            //         applyDeadband( m_driverController1.getLeftY(), OIConstants.deadBand ),
            //         applyDeadband( m_driverController1.getLeftX(), OIConstants.deadBand ),
            //         applyDeadband( -m_driverController1.getRightX(), OIConstants.deadBand ),
            // m_robotDrive))
            // );



        m_robotDrive.setMaxOutput(0.8);



        m_Tab = Shuffleboard.getTab("COMPETITION_FINAL");
        // Limelight set up.
        HttpCamera limelight = new HttpCamera("Limelight", "http://10.34.88.11:5800/");
        CameraServer.startAutomaticCapture(limelight);
        m_Tab.add("Limelight", limelight)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(7, 0)
            .withSize(10, 10);
            // .withSize(7, 5);


    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandExecute(
            command ->
                Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}. 
   */
  private void configureButtonBindings() {

    
    // Drive at half speed when dPad Down is held
        new POVButton(m_driverController1, 0)
            .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
            .whileFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
    
    // Hard Break when dPad Up is held
        new POVButton(m_driverController1, 180)
            .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0)))
            .whileFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

    // Changes robot directions on joystick values when Y button is pressed
        new JoystickButton(m_driverController1, Button.kY.value)
            .onTrue(new InstantCommand(() -> m_robotDrive.changeDirections()));



    // Moves AMP arm forward when right bumper is pressed
        new JoystickButton(m_driverController2, Button.kRightBumper.value)
            .onTrue(new ForwardArmPositionCmd(m_Position));

    // Moves AMP arm backward when left bumper is pressed
        new JoystickButton(m_driverController2, Button.kLeftBumper.value)
            .onTrue(new BackwardArmPositionCmd(m_Position));



    // Sends note to AMP when left trigger is held
    Trigger leftTrigger = new Trigger(() -> m_driverController2.getLeftTriggerAxis() > 0.5 );
    leftTrigger   
        .and( new Trigger(() -> m_AMP.isEmpty()))    
        .whileTrue(new InstantCommand(() -> m_AMP.sendtoamp()))
        .whileFalse(new InstantCommand(() -> m_AMP.stop()));

    // Sends note to Shooter when right trigger is held
    Trigger rightTrigger = new Trigger(() -> m_driverController2.getRightTriggerAxis() > 0.5 );
    rightTrigger
        .and( new Trigger(() -> m_Shooter.isEmpty()))
        .whileTrue(new InstantCommand(() -> m_Shooter.intakeandhold()))
        .whileFalse(new InstantCommand(() -> m_Shooter.stop()));



    // Drops note in AMP when X button is held
    new JoystickButton(m_driverController1, Button.kX.value)
        .whileTrue(new InstantCommand(() -> m_AMP.dropnote()))
        .whileFalse(new InstantCommand(() -> m_AMP.stop()));

    // Shoots note into Speaker when B button is held
    new JoystickButton(m_driverController1, Button.kB.value)
        .whileTrue(new ShootCmd(m_Shooter, m_maxvSubsystem));

    // Revs Shooter when X button is held
    new JoystickButton(m_driverController2, Button.kX.value)
        .whileTrue(new InstantCommand(() -> m_Shooter.rev()));

    // Returns arm back to resting position if belt skipped when A button is held
    new JoystickButton(m_driverController2, Button.kA.value)
        .whileTrue(new InstantCommand(() -> m_Position.returnskip()));

    // Drops (Shooter) note when B button is held
    new JoystickButton(m_driverController2, Button.kB.value)
        .whileTrue(new InstantCommand(() -> m_Shooter.spitout()));
    new JoystickButton(m_driverController2, Button.kB.value)
        .whileFalse(new InstantCommand(() -> m_Shooter.stop()));

    // Drops (AMP) note when B button is held
    new JoystickButton(m_driverController2, Button.kY.value)
        .whileTrue(new InstantCommand(() -> m_AMP.spitout()));
    new JoystickButton(m_driverController2, Button.kY.value)
        .whileFalse(new InstantCommand(() -> m_AMP.stop()));

    // Hard Break when dPad Up is held
    new POVButton(m_driverController2, 180)
        .whileTrue(new InstantCommand(() -> m_Shooter.stop()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(

    // Auto: Middle Start -> shoots into Speaker, stops motors, reverses out of starting zone 
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto()),
        // new WaitCommand(1),
        // new DriveForTimeCmd(m_robotDrive, 0.3, 1.5, 0, 0)

    // Auto: Diagonal Start -> shoots into Speaker, stops motors, reverses out of starting zone 
        new InstantCommand(() -> m_Shooter.rev()),
        new WaitCommand(1.4),
        new InstantCommand(() -> m_Shooter.shootAuto()),
        new WaitCommand(2), // Was 2 during the practice match
        new InstantCommand(() -> m_Shooter.stopAuto()),
        new WaitCommand(1)
        //new DriveForTimeCmd(m_robotDrive, 0.3, 4, 0, 0)

    
        
    // Auto: Irreveleant Location -> shoots into Speaker, stops motors, stays in starting zone
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto()),
        // new WaitCommand(1)

    // Auto: Middle Start -> shoots into Speaker, stops motors, reverses out of starting zone while picking up note, drives back, shoots into Speaker
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto()),
        // new WaitCommand(1),
        // new InstantCommand(() -> m_Shooter.intakeandholdAuto()),
        // new DriveForTimeCmd(m_robotDrive, 0.3, 1.5, 0, 0),
        // new WaitUntilCommand(m_Shooter::isFull),
        // new InstantCommand(() -> m_Shooter.stopAutoPickup()),
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(0.5),
        // new DriveForTimeCmd(m_robotDrive, -0.3, 1.6, 0, 0),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto())







        
        //new InstantCommand(() -> m_robotDrive.driveCartesian(0.3, 0, 0)).withTimeout(2)

        // new InstantCommand(() -> m_robotDrive.drive(0.3, 0, 0, false, false)),
        // new WaitCommand(1),
        // new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false))

        //(new InstantCommand(() -> m_robotDrive.drive(0.3, 0, 0, false, false))).withTimeout(2)
        //new DriveForTimeCmd(m_robotDrive, 0.3)

        //new InstantCommand(() -> m_Shooter.intakeandhold()),
        // new WaitCommand(1),
        // new InstantCommand(() -> m_Shooter.stopAuto()),
        // new WaitCommand(3),
        // new InstantCommand(() -> m_robotDrive.drive()),
        // new WaitCommand(1.3),
        // new InstantCommand(() -> m_robotDrive.stop())

        
        //new DriveForTimeCmd(m_robotDrive, 0.3, 1.3,0,0)


        // Autonomous: Goes to AMP, drops note, returns to original position, drives out of starting zone
        // new DriveForTimeCmd(m_robotDrive, 0.3, 1, 0, 0),
        // new DriveForTimeCmd(m_robotDrive, 0, 1.0099 , 0, 0.2),   //1.0099 Autonomuos, 1.1289 Practice
        // new DriveForTimeCmd(m_robotDrive, 0.3, 1.93, 0, 0),
        // new InstantCommand(() -> new ForwardArmPositionCmd(m_Position)),
        // new InstantCommand(() -> m_AMP.dropnote()).withTimeout(0.5),
        // new InstantCommand(() -> m_AMP.stop()),
        // new DriveForTimeCmd(m_robotDrive, -0.3, 1.93, 0, 0),
        // new DriveForTimeCmd(m_robotDrive, 0, 1.0099, 0, -0.2),
        // new DriveForTimeCmd(m_robotDrive, 0.3, 2, 0, 0),
        // new DriveForTimeCmd(m_robotDrive, 0, 0,0,0)
        
        // Auto: Middle Start -> shoots into Speaker, stops motors, reverses out of starting zone while picking up note, drives back, shoots into Speaker
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto()),
        // new WaitCommand(1),
        // new DriveForTimeCmd(m_robotDrive, 0.3, 2.3, 0, 0).alongWith(new InstantCommand(() -> m_Shooter.intakeandhold())),
        // new WaitCommand(0.7),
        // new DriveForTimeCmd(m_robotDrive, 0.3, 2.3, 0, 0).withTimeout(2),
        // new InstantCommand(() -> m_Shooter.rev()),
        // new WaitCommand(1.4),
        // new InstantCommand(() -> m_Shooter.shootAuto()),
        // new WaitCommand(2), // Was 2 during the practice match
        // new InstantCommand(() -> m_Shooter.stopAuto())
    );

    
    
    // Autonomous Testing: S-shaped Path
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    // MecanumControllerCommand mecanumControllerCommand =
    //     new MecanumControllerCommand(
    //         exampleTrajectory,
    //         m_robotDrive::getPose,
    //         DriveConstants.kFeedforward,
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         new ProfiledPIDController(
    //             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

    //         // Needed for normalizing wheel speeds
    //         AutoConstants.kMaxSpeedMetersPerSecond,

    //         // Velocity PID's
    //         new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
    //         new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
    //         new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
    //         new PIDController(DriveConstants.kPRearRightVel, 0, 0),
    //         m_robotDrive::getCurrentWheelSpeeds,
    //         m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
    //         m_robotDrive);

    // // Reset odometry to the initial pose of the trajectory, run path following
    // // command, then stop at the end.
    // return Commands.sequence(
    //     new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
    //     mecanumControllerCommand,
    //     new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }

  public static double applyDeadband( double joystickValue, double deadBand )
  {
    if ( Math.abs( joystickValue ) < deadBand )
    {
      return 0.0;
    }   
    else
    {
      return ( 1.0 / ( 1.0 - deadBand ) ) * ( joystickValue + ( -Math.signum( joystickValue ) * deadBand ) );
    }
  }
}
