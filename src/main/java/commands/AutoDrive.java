package commands;
import javax.sound.sampled.LineEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FFArmSubsystem;
/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AutoDrive extends Command {
  // The subsystem the command runs on
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  int algaepos = 0;
  private String mode;
  private String goal;
  private double yoffset;
  private double zdist;
  PIDController rotpid = new PIDController(0.4, 0, 0);
  PIDController drivepid = new PIDController(0.8, 0, 0.1);
  PIDController strafepid = new PIDController(0.8, 0, 0.1);
  private String limelight = "limelight";
  public boolean rotlinedup = false;
  public boolean ylinedup = false;
  public boolean zlinedup = false;

  public AutoDrive(String modeset, ArmSubsystem asubsystem, DriveSubsystem dSubsystem) {
    m_armSubsystem = asubsystem;
    m_driveSubsystem = dSubsystem;
    mode = modeset;

    addRequirements(m_armSubsystem, m_driveSubsystem);
   
  }

  @Override
  public void initialize() {
    
    }

  @Override
  public void execute() {
    double xSpeedDelivered = 0;
    double ySpeedDelivered = 0;
    double rotDelivered = 0;
    goal = m_armSubsystem.goalcheck();
    if (goal=="CS" || goal=="AP") {
      limelight = "limelight";
      yoffset = 0.24; 
    } else if (goal=="C1" || goal=="C2" || goal=="C3") {
      limelight = "limelight-lesser";
      if (m_armSubsystem.offsetcheck()) {
        yoffset = 0.27;
      } else {
      yoffset = -0.04;
      }
    } else if (goal=="A1" || goal=="A2") {
      limelight = "limelight-lesser";
      yoffset = .17;
    }
      NetworkTable table = NetworkTableInstance.getDefault().getTable(limelight);
      double[] targpose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
      double degoffset = NetworkTableInstance.getDefault().getTable(limelight).getEntry("tx").getDouble(0);
    // Original: double rx = (targpose[4] < (targpose[2] * -6) && targpose[4] > (targpose[2] * 6)) ? 0 : targpose[4];
      double range = (Math.pow(Math.abs(targpose[2]), 2.5));
      double rx = (targpose[4] < range && targpose[4] > -range) ? 0 : targpose[4];
      //double[] targpose = {1,2,3,4,5,6};
     // targpose = new double[6];
      // targpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
      if (mode == "approach") {
        xSpeedDelivered = -drivepid.calculate(targpose[2], m_armSubsystem.goaldistance()) * DriveConstants.kMaxSpeedMetersPerSecond;
        xSpeedDelivered = MathUtil.clamp(xSpeedDelivered, -1, 1);
        if ((targpose[2] < m_armSubsystem.goaldistance()+ 0.04) && (targpose[2] > m_armSubsystem.goaldistance() - 0.04)) {
          zlinedup = true;
        } else {
          zlinedup = false;
        }
                
        if (rx <= -6 || rx >= -2) {
          rotDelivered = -rotpid.calculate(rx, -4);
          rotDelivered = MathUtil.clamp(rotDelivered, -0.45, 0.45);
          rotlinedup = false;
        } else {
          rotDelivered = 0;
          rotlinedup = true;
          }

        if (targpose[0] >= (yoffset + m_driveSubsystem.getytrim() + 0.03) || targpose[0] <= (yoffset + m_driveSubsystem.getytrim() - 0.03)) {
          ySpeedDelivered = strafepid.calculate(targpose[0], yoffset + m_driveSubsystem.getytrim()) * DriveConstants.kMaxSpeedMetersPerSecond;
          ySpeedDelivered = MathUtil.clamp(ySpeedDelivered, -0.5, 0.5);
          ylinedup = false;
        } else {
          ySpeedDelivered = 0;
          ylinedup = true;
          }
     } else if (mode == "square") {
        xSpeedDelivered = 0;
       // ySpeedDelivered = drivepid.calculate(targpose[4], 0) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = -strafepid.calculate(targpose[0], 0) * DriveConstants.kMaxSpeedMetersPerSecond;
       rotDelivered = rotpid.calculate(degoffset, 0);
       }
  SmartDashboard.putNumber("tz", targpose[2]);
  SmartDashboard.putNumber("tx", targpose[0]);
  SmartDashboard.putNumber("yaw", targpose[4]);
  SmartDashboard.putNumber("xspeedd", xSpeedDelivered);
  SmartDashboard.putNumber("yspeedd", ySpeedDelivered);
  SmartDashboard.putNumber("rotd", rotDelivered);
  SmartDashboard.putNumber("degoffset", degoffset);
  SmartDashboard.putNumber("rangeval", range);
  SmartDashboard.putNumber("TargDist", m_armSubsystem.goaldistance());
  SmartDashboard.putBoolean("RotLinedup", rotlinedup);
  SmartDashboard.putBoolean("ylinedup", ylinedup);
  SmartDashboard.putBoolean("zlinedup", zlinedup);
  // if (rotlinedup && ylinedup) {
  //   m_driveSubsystem.setcolor(2);
  // }
  m_driveSubsystem.autodrive(-xSpeedDelivered, -ySpeedDelivered, rotDelivered);
    
  }
  // @Override
  // public void end(boolean interrupted) {
  //   m_driveSubsystem.setcolor(0);
  // }  
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setcolor(2);
    m_driveSubsystem.autodrive(0, 0, 0);
  }
  @Override
  public boolean isFinished() {
    if (rotlinedup && ylinedup && zlinedup) {
      return true;
    } else {
      return false;
    }
  }
}