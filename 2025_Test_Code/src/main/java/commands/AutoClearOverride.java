package commands;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoClearOverride extends Command {
  // The subsystem the command runs on
  private final DriveSubsystem m_driveSubsystem;


  public AutoClearOverride(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
   
  }

  @Override
  public void initialize() {
    m_driveSubsystem.clearoverride();
    }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}