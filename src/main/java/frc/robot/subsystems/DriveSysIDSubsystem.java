// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSysIDSubsystem extends SubsystemBase 
{
  // Mecanum Drivetrain Motors
  private final TalonFX m_frontLeft = new TalonFX( DriveConstants.kFrontLeftMotorPort );
  private final TalonFX m_rearLeft = new TalonFX( DriveConstants.kRearLeftMotorPort );
  private final TalonFX m_frontRight = new TalonFX( DriveConstants.kFrontRightMotorPort );
  private final TalonFX m_rearRight = new TalonFX( DriveConstants.kRearRightMotorPort );

  private final MecanumDrive m_drive = new MecanumDrive( m_frontLeft, m_rearLeft, m_frontRight, m_rearRight );

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_SysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            ( Measure<Voltage> volts ) -> {
                m_frontLeft.setVoltage( volts.in( Volts) );
                m_rearLeft.setVoltage( volts.in( Volts) );
                m_frontRight.setVoltage( volts.in( Volts) );
                m_rearRight.setVoltage( volts.in( Volts) );
            },
            log -> {
                log.motor( "drive-front-left" )
                    .voltage( m_appliedVoltage.mut_replace( m_frontLeft.get() * RobotController.getBatteryVoltage(), Volts ) )
                    .linearPosition( m_distance.mut_replace( this.getDistance( m_frontLeft ), Meters ) )
                    .linearVelocity( m_velocity.mut_replace( this.getRate( m_frontLeft ), MetersPerSecond ) );
                log.motor( "drive-rear-left" )
                    .voltage( m_appliedVoltage.mut_replace( m_rearLeft.get() * RobotController.getBatteryVoltage(), Volts ) )
                    .linearPosition( m_distance.mut_replace( this.getDistance( m_rearLeft ), Meters ) )
                    .linearVelocity( m_velocity.mut_replace( this.getRate( m_rearLeft ), MetersPerSecond ) );
                log.motor( "drive-front-right" )
                    .voltage( m_appliedVoltage.mut_replace( m_frontRight.get() * RobotController.getBatteryVoltage(), Volts ) )
                    .linearPosition( m_distance.mut_replace( this.getDistance( m_frontRight ), Meters ) )
                    .linearVelocity( m_velocity.mut_replace( this.getRate( m_frontRight ), MetersPerSecond ) );
                log.motor( "drive-rear-right" )
                    .voltage( m_appliedVoltage.mut_replace( m_rearRight.get() * RobotController.getBatteryVoltage(), Volts ) )
                    .linearPosition( m_distance.mut_replace( this.getDistance( m_rearRight ), Meters ) )
                    .linearVelocity( m_velocity.mut_replace( this.getRate( m_rearRight ), MetersPerSecond ) );                
            },
            this ) );

  public DriveSysIDSubsystem() 
  {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Configure the drive motors - Inverting the right side motors, setting all motors to brake mode, setting ramp rate based on 2023 value    
    var frontLeftConfiguration = new TalonFXConfiguration();
    frontLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontLeftConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    frontLeftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_frontLeft.getConfigurator().apply( frontLeftConfiguration );

    var rearLeftConfiguration = new TalonFXConfiguration();
    rearLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rearLeftConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    rearLeftConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_rearLeft.getConfigurator().apply( rearLeftConfiguration );

    var frontRightConfiguration = new TalonFXConfiguration();
    frontRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontRightConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    frontRightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_frontRight.getConfigurator().apply( frontRightConfiguration );

    var rearRightConfiguration = new TalonFXConfiguration();
    rearRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rearRightConfiguration.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    rearRightConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.kRampInSec;
    m_rearRight.getConfigurator().apply( rearRightConfiguration );

    this.resetEncoders();
  }

  public Command driveCommand( DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot )
  {
    return run( () -> m_drive.driveCartesian( xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble() ) ).withName( "driveCartesian" );
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {
    m_drive.driveCartesian( xSpeed, ySpeed, rot );
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

  public double getRate( TalonFX motor )
  {
    double rotationsPerSecond = (double) motor.getVelocity().getValue() / DriveConstants.kGearRatio;
    double velocity = rotationsPerSecond * Math.PI * DriveConstants.kWheelDiameterMeters;
    return velocity;
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

  public double getDistance( TalonFX motor )
  {
    double distance = motor.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * Math.PI * DriveConstants.kWheelDiameterMeters;
    return distance;
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

  public Command sysIdQuasistatic( SysIdRoutine.Direction direction )
  {
    return m_SysIdRoutine.quasistatic( direction );
  }

  public Command sysIdDynamic( SysIdRoutine.Direction direction )
  {
    return m_SysIdRoutine.dynamic( direction );
  }
}
