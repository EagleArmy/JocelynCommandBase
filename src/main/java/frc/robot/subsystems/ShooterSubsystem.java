// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.AMPConstants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX PickupMotor = new TalonFX(PickupConstants.PickupMotorID);
  private final TalonFX IndexerMotor = new TalonFX(IndexerConstants.IndexerMotorID);
  private final TalonFX shooterTop = new TalonFX( ShooterConstants.kShooterTopChannel );
  private final TalonFX shooterBottom = new TalonFX( ShooterConstants.kShooterBottomChannel );
  private final TalonFX PusherMotor = new TalonFX( ShooterConstants.PusherMotorID);
  private final TimeOfFlight sensor = new TimeOfFlight(13);
  private final TalonFX extramotor = new TalonFX( 13 );
  private boolean test = true;

  

  /** Create a new claw subsystem. */
  public ShooterSubsystem() {
    // Let's name everything on the LiveWindow
    addChild("PickupMotor", PickupMotor);
    addChild("IndexerMotor", IndexerMotor);
    addChild("shooterTop", shooterTop);
    addChild("shooterBottom", shooterBottom);
    addChild("PusherMotor", PusherMotor);
    addChild("sensor", sensor);


    var pickupConfiguration = new TalonFXConfiguration();
    pickupConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PickupMotor.getConfigurator().apply( pickupConfiguration );

    var indexerConfiguration = new TalonFXConfiguration();
    indexerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IndexerMotor.getConfigurator().apply( indexerConfiguration );

    var pusherConfiguration = new TalonFXConfiguration();
    pusherConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PusherMotor.getConfigurator().apply( pusherConfiguration );

    sensor.setRangingMode( TimeOfFlight.RangingMode.Short, 24);
  }

  public void log() { 
  }


  public void intakeandhold() {
    if (test) {
    PickupMotor.set(0.5);
    IndexerMotor.set(-0.5);
    extramotor.set(0.3);
    PusherMotor.set(-0.2);
    }
    else {
      stop();
    }
  }

  public void spitout() {
    PickupMotor.set(-0.5);
    IndexerMotor.set(0.5);
    extramotor.set(-0.3);
    PusherMotor.set(0.2);
  }

  public void stop() {
    PickupMotor.set(0);
    IndexerMotor.set(0);
    shooterTop.set(0);
    shooterBottom.set(0);
    extramotor.set(0);
    PusherMotor.set(0);
  }

  public void stopAuto() {
    PickupMotor.set(0);
    IndexerMotor.set(0);
    shooterTop.set(0);
    shooterBottom.set(0);
    PusherMotor.set(0);
  }

  public void stopAutoPickup() {
    PickupMotor.set(0);
    IndexerMotor.set(0);
    extramotor.set(0);
    PusherMotor.set(0);
  }

  public void intakeandholdAuto() {
    System.out.println("Value: " + getSensorRange());
    if (!(getSensorRange() < 120)) {
      intakeandhold();
    }
    else {
      stop();
    }
  }

  public void rev() {
    shooterTop.set(-0.7);
    shooterBottom.set(-0.7);
  }


  public void shoot() {
    PickupMotor.set(0.5);
    IndexerMotor.set(-0.5);
    PusherMotor.set(-0.3);
    shooterTop.set(-0.7);
    shooterBottom.set(-0.7);
    extramotor.set(0.3);
  }

  public void shootAuto() {
    PickupMotor.set(0.5);
    IndexerMotor.set(-0.5);
    PusherMotor.set(-0.3);
    //shooterTop.set(-0.7);
    //shooterBottom.set(-0.7);
    //extramotor.set(0.3);
  }

  public double getSensorRange() {
    return sensor.getRange();
  }

  public boolean isEmpty() 
  {
    return (sensor.getRange() > 120);
  }

  public boolean isFull() 
  {
    return (sensor.getRange() < 120);
  }

  public double getShooterSensorLimit() {
    return SensorConstants.ShooterSensorLimit;
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }
}