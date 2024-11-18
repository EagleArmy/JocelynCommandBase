package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.MaxVelocitySubsystem;

/**
 * A simple command that grabs a hatch with the {@link ShooterSubsystem} and {@link MaxVelocitySubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ShootCmd extends Command {
  // The subsystem the command runs on
  private final ShooterSubsystem m_shooterSubsystem;
  private final MaxVelocitySubsystem m_maxvelocitySubsystem;

  public ShootCmd(ShooterSubsystem subsystem1, MaxVelocitySubsystem subsystem2) {
    m_shooterSubsystem = subsystem1;
    m_maxvelocitySubsystem = subsystem2;
    addRequirements(m_shooterSubsystem, m_maxvelocitySubsystem);
  }

  
  @Override
  public void initialize() { }

  @Override
  public void execute() {
    // m_maxvelocitySubsystem.testing();
    m_shooterSubsystem.shoot();
  }

  @Override
  public void end(boolean interrupted) {
      m_shooterSubsystem.stop(); // Stop the shooter motors when the command ends
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}