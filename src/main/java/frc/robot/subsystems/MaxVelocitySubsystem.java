// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.ControlMode;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class MaxVelocitySubsystem extends SubsystemBase {
  private TalonFX topShooter, bottomShooter;
 
  /** Create a new claw subsystem. */
  public MaxVelocitySubsystem() {

    topShooter = new TalonFX(ShooterConstants.kShooterTopChannel);
    // Let's name everything on the LiveWindow
    addChild("topShooter", topShooter);

    var topShooterConfiguration = new TalonFXConfiguration();
    topShooterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    topShooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topShooter.getConfigurator().apply( topShooterConfiguration );
    
    // in init function
    var talonFXConfigs1 = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs1 = talonFXConfigs1.Slot0;
    slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs1.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs1.kI = 0; // no output for integrated error
    slot0Configs1.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs1 = talonFXConfigs1.MotionMagic;
    motionMagicConfigs1.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs1.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    topShooter.getConfigurator().apply(talonFXConfigs1);




    bottomShooter = new TalonFX(ShooterConstants.kShooterBottomChannel);
    // Let's name everything on the LiveWindow
    addChild("bottomShooter", bottomShooter);

    var bottomShooterConfiguration = new TalonFXConfiguration();
    bottomShooterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bottomShooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomShooter.getConfigurator().apply( bottomShooterConfiguration );
    
    // in init function
    var talonFXConfigs2 = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs2 = talonFXConfigs2.Slot0;
    slot0Configs2.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs2.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs2.kI = 0; // no output for integrated error
    slot0Configs2.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs2 = talonFXConfigs2.MotionMagic;
    motionMagicConfigs2.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs2.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    bottomShooter.getConfigurator().apply(talonFXConfigs2);

  }

  public void testing() {
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    // if (m_joy.getAButton()) {
    // // while the joystick A button is held, use a slower acceleration
    // m_request.Acceleration = 100; // rot/s^2
    // } else {
    // // otherwise, fall back to the config
    // m_request.Acceleration = 0;
    // }
    // set target velocity to 80 rps
    topShooter.setControl(m_request.withVelocity(70));
    bottomShooter.setControl(m_request.withVelocity(-70));
  }

  public void stop() {
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    topShooter.setControl(m_request.withVelocity(0));
    bottomShooter.setControl(m_request.withVelocity(0));
  }

  public void log() {
    // SmartDashboard.putData("leader", topShooter);
    // SmartDashboard.putData("follower", bottomShooter);
  }
  
  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }

}





//shoot when velocity reaches some value 
// motor.getVelocity();