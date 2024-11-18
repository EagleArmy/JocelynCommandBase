package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MaxVelocitySubsystem;

/**
 * A simple command that grabs a hatch with the {@link MaxVelocitySubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class MaxVelocityCmd extends Command {
  // The subsystem the command runs on
  private final MaxVelocitySubsystem m_maxvSubsystem;

  public MaxVelocityCmd(MaxVelocitySubsystem subsystem) {
    m_maxvSubsystem = subsystem;
    addRequirements(m_maxvSubsystem);
  }

  
  @Override
  public void initialize() { }

  @Override
  public void execute() {
    m_maxvSubsystem.testing();
  }

  @Override
  public void end(boolean interrupted) {
      m_maxvSubsystem.stop();             // Stop the shooter motors when the command ends
  }

  @Override
  public boolean isFinished() {
    return false;
  }
        
}
