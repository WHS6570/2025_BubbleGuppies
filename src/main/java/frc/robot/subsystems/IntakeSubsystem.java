package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax m_left550 = new SparkMax(43, MotorType.kBrushless);
  SparkMax m_right550 = new SparkMax(44, MotorType.kBrushless);
  SparkMaxConfig leftconfig = new SparkMaxConfig();
  SparkMaxConfig rightconfig = new SparkMaxConfig();
  public CANrange distOnboard;

  public IntakeSubsystem() {
    leftconfig.smartCurrentLimit(30)
    .idleMode(IdleMode.kBrake);
    rightconfig.smartCurrentLimit(30)
    .idleMode(IdleMode.kBrake);
    m_left550.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_right550.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    distOnboard = new CANrange(31);
    //distOnboard.setAutomaticMode(true);
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
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public void spinny(double speed) {
    m_left550.set(-speed);
    m_right550.set(speed);
  }
  public void trough() {
    m_left550.set(0.0);
    m_right550.set(0.5);
  }
  public double getdistance() {
    return distOnboard.getDistance().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("dists", distOnboard.getDistance().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}