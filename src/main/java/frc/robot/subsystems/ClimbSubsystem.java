package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public PneumaticHub ph = new PneumaticHub(2);
  DoubleSolenoid m_soleboom = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
  Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);

  public ClimbSubsystem() {
    ph.enableCompressorDigital();

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 // public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
 //   return runOnce(
  //      () -> {
          /* one-time action goes here */
  //      });
 // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void startcomp() {
    // Query some boolean state, such as a digital sensor.
    ph.enableCompressorDigital();
  }
  public void pushnpull(boolean extend) {
    if (extend == true) {
        m_soleboom.set(Value.kForward);
    } else if (extend == false) {
        m_soleboom.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}