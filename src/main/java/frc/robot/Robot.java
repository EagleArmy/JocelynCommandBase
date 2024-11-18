package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private RobotContainer m_robot;
  // private SysIdRoutineBot m_robot;

  @Override
  public void robotInit() 
  {
    m_robot = new RobotContainer();
    // m_robot = new SysIdRoutineBot();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robot.getAutonomousCommand();

    if ( m_autonomousCommand != null ) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  
}
