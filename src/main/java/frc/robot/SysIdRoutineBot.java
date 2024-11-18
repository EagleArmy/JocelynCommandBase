package frc.robot;

import frc.robot.Constants.OIConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveSysIDSubsystem;

public class SysIdRoutineBot
{
    private final DriveSysIDSubsystem m_drive = new DriveSysIDSubsystem();

    CommandXboxController m_driverController = new CommandXboxController( OIConstants.kDriverControllerPort1 );

    public SysIdRoutineBot() 
    {
        m_drive.setDefaultCommand(
            new RunCommand(
                () ->
                    m_drive.drive(
                        applyDeadband( -m_driverController.getLeftY(), OIConstants.deadBand ),
                        applyDeadband( -m_driverController.getLeftX(), OIConstants.deadBand ),
                        applyDeadband( m_driverController.getRightX(), OIConstants.deadBand ),
                        false),
                m_drive));
        
        configureBindings();
    }

    public void configureBindings()
    {
        m_driverController.a().whileTrue( m_drive.sysIdQuasistatic( SysIdRoutine.Direction.kForward) );
        m_driverController.b().whileTrue( m_drive.sysIdQuasistatic( SysIdRoutine.Direction.kReverse ) );
        m_driverController.x().whileTrue( m_drive.sysIdDynamic( SysIdRoutine.Direction.kForward ) );
        m_driverController.y().whileTrue( m_drive.sysIdDynamic( SysIdRoutine.Direction.kReverse ) );
    }

    public Command getAutonomousCommand()
    {
        return m_drive.run( () -> {} );
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


