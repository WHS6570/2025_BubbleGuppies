package commands;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoA1 extends Command {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;


  public AutoA1(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
   
  }

  @Override
  public void initialize() {
    m_armSubsystem.flex(ArmConstants.kAlgae1[0], ArmConstants.kAlgae1[1], "A1", true, true);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}