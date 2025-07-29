package commands;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoShoot extends Command {
  // The subsystem the command runs on
  private final IntakeSubsystem m_intakeSubsystem;
  private final ArmSubsystem m_armSubsystem;
double m_startTime = 0;

  public AutoShoot(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_armSubsystem = armSubsystem;
    addRequirements(m_intakeSubsystem, m_armSubsystem);
   
  }

  @Override
  public void initialize() {
    m_startTime = WPIUtilJNI.now() * 1e-6;
    }

  @Override
  public void execute() {
    if (m_armSubsystem.goalcheck() == "C3") {
      m_intakeSubsystem.spinny(0.8);
    } else {
    m_intakeSubsystem.spinny(-1);
    }
  }

  @Override
  public boolean isFinished() {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_startTime;
    if (elapsedTime>0.5) {return true;} else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.spinny(0.0);
  }
}