package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionSubsystem;

/**
 * A simple command that grabs a hatch with the {@link PositionSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ForwardArmPositionCmd extends Command {
  // The subsystem the command runs on
  private PositionSubsystem m_positionSubsystem;

  public ForwardArmPositionCmd(PositionSubsystem subsystem) {
    m_positionSubsystem = subsystem;
    addRequirements(m_positionSubsystem);
  }


  @Override
  public void initialize() { }
  

  @Override
  public void execute() {
    m_positionSubsystem.forward();
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
