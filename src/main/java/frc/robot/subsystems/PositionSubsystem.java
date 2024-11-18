// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.ControlMode;

import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class PositionSubsystem extends SubsystemBase {
  private TalonFX PositionMotor1;
  private double position;

  /** Create a new claw subsystem. */
  public PositionSubsystem() {

    PositionMotor1 = new TalonFX(PositionConstants.kPositionMotor1);
    PositionMotor1.setPosition( 0 );
    position = getEncoderPosition();
    
    // Let's name everything on the LiveWindow
    addChild("PositionMotor1", PositionMotor1);

    var PositionMotor1Configuration = new TalonFXConfiguration();
    PositionMotor1Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PositionMotor1Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PositionMotor1.getConfigurator().apply( PositionMotor1Configuration );
    
    var talonFXConfigs1 = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs1 = talonFXConfigs1.Slot0;
    slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs1.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs1.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs1.kI = 0; // no output for integrated error
    slot0Configs1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs1 = talonFXConfigs1.MotionMagic;
    motionMagicConfigs1.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigs1.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs1.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // var motionMagicConfigs1 = talonFXConfigs1.MotionMagic;
    // motionMagicConfigs1.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    // motionMagicConfigs1.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs1.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    PositionMotor1.getConfigurator().apply(talonFXConfigs1);

  }

  

  public void forward() {
    //create a Motion Magic request, voltage output
    MotionMagicVoltage m_request = new MotionMagicVoltage(PositionConstants.kforwardPosition);
    // set target position to 100 rotations
    PositionMotor1.setControl(m_request.withPosition(PositionConstants.kbackwardPosition));
    position = getEncoderPosition();
  }

    public void backward() {
    //create a Motion Magic request, voltage output
    MotionMagicVoltage m_request = new MotionMagicVoltage(PositionConstants.kbackwardPosition);
    // set target position to 100 rotations
    
    PositionMotor1.setControl(m_request.withPosition(PositionConstants.kforwardPosition));
    // PositionMotor1.setPosition( 0 );
    // var PositionMotor1Configuration = new TalonFXConfiguration();
    // PositionMotor1Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // PositionMotor1.getConfigurator().apply( PositionMotor1Configuration );
    position = getEncoderPosition();
  }

  public void returnskip() {
    PositionMotor1.set(-0.05);
    setEncoderPosition(0);
    PositionMotor1.setPosition(0);
  }

  public double getEncoderPosition() {
    var rotorPosSignal = PositionMotor1.getRotorPosition();
    var rotorPos = rotorPosSignal.getValue();
    return rotorPos;
 }

 public void setEncoderPosition( double newposition) {
  position = newposition;
 }
  

  public void log() {
    // SmartDashboard.putData("PositionMotor1", PositionMotor1);
  }
  
  /** Call log method every loop. */
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
    log();
  }

}



// //shoot when velocity reaches some value 
// // motor.getVelocity();