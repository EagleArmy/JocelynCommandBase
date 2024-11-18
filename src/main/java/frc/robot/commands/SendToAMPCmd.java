package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPSubsystem;

/**
 * A simple command that grabs a hatch with the {@link AMPSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class SendToAMPCmd extends Command {
  // The subsystem the command runs on
  private final AMPSubsystem m_ampSubsystem;
  private boolean running = true;

  public SendToAMPCmd(AMPSubsystem subsystem) {
    m_ampSubsystem = subsystem;
    addRequirements(m_ampSubsystem);
  }


  @Override
  public void initialize() { }

  @Override
  public void execute() {
    if (!(m_ampSubsystem.getSensorRange() < 300)) {
      m_ampSubsystem.sendtoamp();
    }
    else {
      m_ampSubsystem.stop();
      System.out.println("AMP SENSOR TRIGGERED!!!!");
    }
  }

  @Override
  public void end(boolean interrupted) {
      m_ampSubsystem.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}