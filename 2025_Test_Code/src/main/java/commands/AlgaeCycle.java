package commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FFArmSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AlgaeCycle extends Command {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;
  int algaepos = 0;

  public AlgaeCycle(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
   
  }

  @Override
  public void initialize() {
    algaepos = m_armSubsystem.getalgaepos();
    algaepos++;
    algaepos = (algaepos >= 3) ? 1 : algaepos;
    switch(algaepos) {
      case 1:
        m_armSubsystem.flex(ArmConstants.kAlgae1[0], ArmConstants.kAlgae1[1], "A1", true, false);
        break;
      case 2:
        m_armSubsystem.flex(ArmConstants.kAlgae2[0], ArmConstants.kAlgae2[1], "A2", true, false);
        break;
    }
      SmartDashboard.putNumber("algae_cycle", algaepos);  
      m_armSubsystem.setalgaepos(algaepos);
    }


  @Override
  public boolean isFinished() {
    return true;
  }
}