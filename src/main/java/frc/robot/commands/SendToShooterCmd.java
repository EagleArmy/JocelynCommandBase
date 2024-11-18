package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A simple command that grabs a hatch with the {@link ShooterSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class SendToShooterCmd extends Command {
  // The subsystem the command runs on
  private final ShooterSubsystem m_shooterSubsystem;

  public SendToShooterCmd(ShooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    addRequirements(m_shooterSubsystem);
  }


  @Override
  public void initialize() { }

  @Override
  public void execute() {
    if (!(m_shooterSubsystem.getSensorRange() < 120)) {
      m_shooterSubsystem.intakeandhold();
    }
    else {
      m_shooterSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
      m_shooterSubsystem.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}