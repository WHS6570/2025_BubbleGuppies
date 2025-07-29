package commands;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoBackup extends Command {
  // The subsystem the command runs on
  private final DriveSubsystem m_driveSubsystem;
double m_startTime = 0;

  public AutoBackup(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
   
  }

  @Override
  public void initialize() {
    m_startTime = WPIUtilJNI.now() * 1e-6;
    }

  @Override
  public void execute() {
    m_driveSubsystem.drive(-0.5, 0, 0, false, "CS");
  }

  @Override
  public boolean isFinished() {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_startTime;
    if (elapsedTime>0.5) {return true;} else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0.0, 0, 0, false, "CS");
  }
}