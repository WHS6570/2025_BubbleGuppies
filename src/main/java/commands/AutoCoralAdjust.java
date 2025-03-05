package commands;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoCoralAdjust extends Command {
  // The subsystem the command runs on
  private final IntakeSubsystem m_intakeSubsystem;
  double m_startTime = 0;

  public AutoCoralAdjust(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(m_intakeSubsystem);
   
  }

  @Override
  public void initialize() {
    m_startTime = WPIUtilJNI.now() * 1e-6;
    }

  @Override
  public void execute() {
    m_intakeSubsystem.spinny(-0.1);
  }

  @Override
  public boolean isFinished() {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_startTime;
    if (elapsedTime>0.2) {return true;} else {return false;}
  }
}