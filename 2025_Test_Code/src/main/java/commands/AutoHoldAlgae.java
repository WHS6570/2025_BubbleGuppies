package commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoHoldAlgae extends Command {
  // The subsystem the command runs on
  private final IntakeSubsystem m_intakeSubsystem;

  public AutoHoldAlgae(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(m_intakeSubsystem);
   
  }

  @Override
  public void initialize() {
    
    }

  @Override
  public void execute() {
    m_intakeSubsystem.spinny((m_intakeSubsystem.getdistance()-0.06)*10);
  }

  @Override
  public boolean isFinished() {

        return false;
    
  }
}