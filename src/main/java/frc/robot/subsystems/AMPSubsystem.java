// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AMPConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PickupConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class AMPSubsystem extends SubsystemBase {
  private final TalonFX PickupMotor = new TalonFX(PickupConstants.PickupMotorID);
  private final TalonFX AMPMotor = new TalonFX(AMPConstants.AMPMotorID);
  private final TalonFX IndexerMotor = new TalonFX(IndexerConstants.IndexerMotorID);
  private final TimeOfFlight sensor = new TimeOfFlight(11);
  private boolean firstsee = true;

  /** Create a new claw subsystem. */
  public AMPSubsystem() {
    // Let's name everything on the LiveWindow
    addChild("PickupMotor", PickupMotor);
    addChild("AMPMotor", AMPMotor);
    addChild("IndexerMotor", IndexerMotor);
    addChild("sensor", sensor);

    var pickupConfiguration = new TalonFXConfiguration();
    pickupConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PickupMotor.getConfigurator().apply( pickupConfiguration );

    var indexerConfiguration = new TalonFXConfiguration();
    indexerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IndexerMotor.getConfigurator().apply( indexerConfiguration );

    var ampConfiguration = new TalonFXConfiguration();
    ampConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    AMPMotor.getConfigurator().apply( ampConfiguration );

    sensor.setRangingMode( TimeOfFlight.RangingMode.Short, 24);
  }


  public void log() {
    // SmartDashboard.putData("Forward", PickupMotor);
    // SmartDashboard.putData("Backward", AMPMotor);
  }


  public void sendtoamp() {
    if (firstsee) {
    PickupMotor.set(0.5);
    IndexerMotor.set(0.5);
    AMPMotor.set(-0.3);
    }
    else {
      stop();
    }
  }

  public void spitout() {
    PickupMotor.set(-0.5);
    IndexerMotor.set(-0.5);
    AMPMotor.set(0.3);
  }

  public void stop() {
    PickupMotor.set(0);
    AMPMotor.set(0);
    IndexerMotor.set(0);
  }

  public void dropnote() {
    AMPMotor.set(-0.5);
  }

  public boolean isEmpty() {
    return (sensor.getRange() > 300);
  }

  public double getSensorRange() {
    return sensor.getRange();
  }

  public double getAMPSensorLimit() {
    return SensorConstants.AMPSensorLimit;
  }


  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }
}