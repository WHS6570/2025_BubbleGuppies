package commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoGrab extends Command {
  // The subsystem the command runs on
  private final IntakeSubsystem m_intakeSubsystem;

  public AutoGrab(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(m_intakeSubsystem);
   
  }

  @Override
  public void initialize() {
    
    }

  @Override
  public void execute() {
    m_intakeSubsystem.spinny(0.25);
  }

  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.getdistance() <= 0.2 && m_intakeSubsystem.getdistance() != -1) {
        return true;
    } else {
        return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.spinny(0);
  }
}