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
public class CoralCycle extends Command {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;
  int count = 0;
  int coralpos = 0;
  boolean up;

  public CoralCycle(boolean upwards, ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
    up = upwards;
   
  }

  @Override
  public void initialize() {
    coralpos = m_armSubsystem.getcoralpos();
      if (up) {
        coralpos++;
        if (coralpos >= 4) {coralpos = 1;}
      } else {
        coralpos--;
         if (coralpos <= 0) {coralpos = 3;}
      }
      switch(coralpos) {
        case 0:
          m_armSubsystem.flex(ArmConstants.kStow[0], ArmConstants.kStow[1], "ST", false, true);
          break;
        case 1:
        m_armSubsystem.flex(ArmConstants.kReef1[0], ArmConstants.kReef1[1], "C1", false, true); 
          break;
        case 2:
        m_armSubsystem.flex(ArmConstants.kReef2[0], ArmConstants.kReef2[1], "C2", false, true);
          break;
        case 3:
        m_armSubsystem.flex(ArmConstants.kReef3[0], ArmConstants.kReef3[1], "C3", false, true);
          break;
        //case 4:
        //  flex(ArmConstants.kReef4[0], ArmConstants.kReef4[1]);
        //  break;
      }
      SmartDashboard.putNumber("coral_cycle", coralpos);  
      m_armSubsystem.setcoralpos(coralpos);
    }


  @Override
  public boolean isFinished() {
    return true;
  }
}