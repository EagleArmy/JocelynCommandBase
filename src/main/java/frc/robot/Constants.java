package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants 
{
  public static final class DriveConstants 
  {
    public static final int kFrontLeftMotorPort = 6;
    public static final int kRearLeftMotorPort = 8;
    public static final int kFrontRightMotorPort = 5;
    public static final int kRearRightMotorPort = 7;


    public static final InvertedValue kLeftReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRightReversed = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final double kRampInSec = 0.1875;
    
    public static final double kTrackWidth = 0.57;  // distance between center of left and right wheels in meters
    public static final double kWheelBase = 0.0002;  // distance between front left and rear left wheels in meters
    public static final double kGearRatio = 12.42857142;  // gear ratio of each drive train motor gear box    
    /*
    2024 VALUES
    public static final double kTrackWidth = 0.545193601;  // distance between center of left and right wheels in meters
    public static final double kWheelBase = .3048;  // distance between front left and rear left wheels in meters
    public static final double kGearRatio = 12.42857142;  // gear ratio of each drive train motor gear box
    */

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderResolution = 2048;  // Falcon 500 has an encoder resolution of 2048
    public static final double kWheelDiameterMeters = 0.2032;  // 8" mecanum wheel diameter in meters
    
    // *********************************************
    // *   MUST BE CHANGED BASED ON SYSID VALUES   *
    // *       CURRENTLY SET TO 2023 NUMBERS       *
    // *********************************************

    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.05829, 2.1614, 0.34687);
    public static final double kPFrontLeftVel = 2.4242;
    public static final double kPRearLeftVel = 0.28481;
    public static final double kPFrontRightVel = 0.18568;
    public static final double kPRearRightVel = 0.0018933;

    // public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.070251, 2.0648, 0.37416);
    // public static final double kPFrontLeftVel = 2.913;
    // public static final double kPRearLeftVel = 0.067362;
    // public static final double kPFrontRightVel = 0.3492;
    // public static final double kPRearRightVel = 0.14478;

    // public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.058218, 2.191, 0.38399);
    // public static final double kPFrontLeftVel = 2.9237;
    // public static final double kPRearLeftVel = 2.9237;
    // public static final double kPFrontRightVel = 2.9237;
    // public static final double kPRearRightVel = 2.9237;

  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort1 = 0;
    public static final double deadBand = 0.2;   //Brew: was 0.1
    public static final int kDriverControllerPort2 = 1;
  }

  public static final class AutoConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // *********************************************
    // *   MUST BE CHANGED BASED ON SYSID VALUES   *
    // *********************************************
    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0;

    // public static final double kPXController = 0.5;
    // public static final double kPYController = 0.5;
    // public static final double kPThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ShooterConstants 
  {
    public static final int ShooterMotorID = 2;
    public static final int kShooterTopChannel = 2;
    public static final int kShooterBottomChannel = 1;
    public static final int PusherMotorID = 11;
  }

  public static final class PositionConstants 
  {
    public static final int kPositionMotor1 = 9;
    public static final double kforwardPosition = 0.0;
    public static final double kbackwardPosition = 23.5;             // CHANGE!!!
  }

  public static final class PickupConstants 
  {
    public static final int PickupMotorID = 4;
  }

  public static final class IndexerConstants
  {
    public static final int IndexerMotorID = 3;
  }

  public static final class AMPConstants 
  {
    public static final int AMPMotorID = 10;
    public static final double kAMPSpeed = 0.6;
  }

  public static final class SensorConstants 
  {
    public static final int AMPSensorLimit = 5000;                    // CHANGE!!!
    public static final int ShooterSensorLimit = 65;                // CHANGE!!!
  }
}
