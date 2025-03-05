package commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FFArmSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class SetTrim extends Command {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  int count = 0;
  int coralpos = 0;
  boolean up;
  double aadjust = 0;
  double yadjust = 0;

  public SetTrim(double achange, double ychange, ArmSubsystem asubsystem, DriveSubsystem dSubsystem) {
    m_armSubsystem = asubsystem;
    m_driveSubsystem = dSubsystem;

    addRequirements(m_armSubsystem, m_driveSubsystem);
    aadjust = achange;
    yadjust = ychange;
   
  }

  @Override
  public void initialize() {
    m_armSubsystem.setatrim(aadjust);
    m_driveSubsystem.setytrim(yadjust);

      }


  @Override
  public boolean isFinished() {
    return true;
  }
}