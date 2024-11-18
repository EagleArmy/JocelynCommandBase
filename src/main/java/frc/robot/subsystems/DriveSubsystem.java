// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase 
{
  // Mecanum Drivetrain Motors
  private final TalonFX m_frontLeft = new TalonFX( DriveConstants.kFrontLeftMotorPort );
  private final TalonFX m_rearLeft = new TalonFX( DriveConstants.kRearLeftMotorPort );
  private final TalonFX m_frontRight = new TalonFX( DriveConstants.kFrontRightMotorPort );
  private final TalonFX m_rearRight = new TalonFX( DriveConstants.kRearRightMotorPort );

  private final MecanumDrive m_drive = new MecanumDrive( m_frontLeft, m_rearLeft, m_frontRight, m_rearRight );
  //private final DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight);

  // Gyro Sensor
  private final AHRS m_gyro = new AHRS( SPI.Port.kMXP );

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry( DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),new MecanumDriveWheelPositions() );
  
  public boolean reversedDirections = false;

  public DriveSubsystem() 
  {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Configure the drive motors - Inverting the right side motors, setting all motors to brake mode, setting ramp rate based on 2023 value    
    var frontLeftConfiguration = new TalonFXConfiguration();
    frontLeftConfiguration.MotorOutput.Inverted = DriveConstants.kLeftReversed;
    frontLeftConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    frontLeftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_frontLeft.getConfigurator().apply( frontLeftConfiguration );

    var rearLeftConfiguration = new TalonFXConfiguration();
    rearLeftConfiguration.MotorOutput.Inverted = DriveConstants.kLeftReversed;
    rearLeftConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    rearLeftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_rearLeft.getConfigurator().apply( rearLeftConfiguration );

    var frontRightConfiguration = new TalonFXConfiguration();
    frontRightConfiguration.MotorOutput.Inverted = DriveConstants.kRightReversed;
    frontRightConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    frontRightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_frontRight.getConfigurator().apply( frontRightConfiguration );

    var rearRightConfiguration = new TalonFXConfiguration();
    rearRightConfiguration.MotorOutput.Inverted = DriveConstants.kRightReversed;
    rearRightConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    rearRightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_rearRight.getConfigurator().apply( rearRightConfiguration );

    this.resetEncoders();

  }

  @Override
  public void periodic() 
  {
    // Update the odometry in the periodic block
    m_odometry.update( m_gyro.getRotation2d(), getCurrentWheelDistances() );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() 
  {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) 
  {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param reversedDirections
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean reversedDirections) 
  {
    if ( fieldRelative ) 
    {
      m_drive.driveCartesian( xSpeed, ySpeed, rot, m_gyro.getRotation2d() );
    } 
    else if ( reversedDirections )
    {
      m_drive.driveCartesian( -xSpeed, -ySpeed, rot );
    }
    else 
    {
      m_drive.driveCartesian( xSpeed, ySpeed, rot );
    }
  }

  // public void drive(double xSpeed, double ySpeed, double rRotation) 
  // {
  //   m_drive.tankDrive(xSpeed, ySpeed);
  // }
  

  public void changeDirections()
  {
    reversedDirections = !reversedDirections;
  }
  
  public boolean getDirectionStatus()
  {
    return reversedDirections;
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts( MecanumDriveMotorVoltages volts ) 
  {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    m_frontLeft.setPosition( 0 );
    m_rearLeft.setPosition( 0 );
    m_frontRight.setPosition( 0 );
    m_rearRight.setPosition( 0 );
  }

  public void feed()
  {
    m_rearLeft.setPosition( 0 );
    m_frontRight.setPosition( 0 );
    m_rearRight.setPosition( 0 );
  }


  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() 
  {
    double frontLeftRotationsPerSecond = (double) m_frontLeft.getVelocity().getValue() / DriveConstants.kGearRatio;
    double frontLeftVelocity = frontLeftRotationsPerSecond * Math.PI * DriveConstants.kWheelDiameterMeters;

    double frontRightRotationsPerSecond = (double) m_frontRight.getVelocity().getValue() / DriveConstants.kGearRatio;
    double frontRightVelocity = frontRightRotationsPerSecond * Math.PI * DriveConstants.kWheelDiameterMeters;

    double rearLeftRotationsPerSecond = (double) m_rearLeft.getVelocity().getValue() / DriveConstants.kGearRatio;
    double rearLeftVelocity = rearLeftRotationsPerSecond * Math.PI * DriveConstants.kWheelDiameterMeters;

    double rearRightRotationsPerSecond = (double) m_rearRight.getVelocity().getValue() / DriveConstants.kGearRatio;
    double rearRightVelocity = rearRightRotationsPerSecond * Math.PI * DriveConstants.kWheelDiameterMeters;

    return new MecanumDriveWheelSpeeds( frontLeftVelocity, frontRightVelocity, rearLeftVelocity, rearRightVelocity );
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() 
  {
    double frontLeftDistance = m_frontLeft.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * Math.PI * DriveConstants.kWheelDiameterMeters;
    double rearLeftDistance = m_rearLeft.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * Math.PI * DriveConstants.kWheelDiameterMeters;
    double frontRightDistance = m_frontRight.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * Math.PI * DriveConstants.kWheelDiameterMeters;
    double rearRightDistance = m_rearRight.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * Math.PI * DriveConstants.kWheelDiameterMeters;

    return new MecanumDriveWheelPositions( frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance );
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput( double maxOutput ) 
  {
    m_drive.setMaxOutput(maxOutput);
  }


  public void stop() {
    m_frontLeft.set(0);
    m_rearLeft.set(0);
    m_frontRight.set(0);
    m_rearRight.set(0);
  }

  public void driveCartesian( double ySpeed, double xSpeed, double zRotation )
  {
    m_drive.driveCartesian( ySpeed, xSpeed, zRotation );
  }
  // public void lineardrive( double ySpeed, double xSpeed )
  // {
  //   m_drive.tankDrive(ySpeed, xSpeed);
  // }
  

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() 
  {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() 
  {
    return -m_gyro.getRate();
  }

}
