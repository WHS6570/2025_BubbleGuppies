package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    static Spark blinkin = new Spark(1);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double target = table.getEntry("tv").getDouble(2.0);
    IntakeSubsystem intake = new IntakeSubsystem();
    //Rev2mDistanceSensor distOnboard = intake.distOnboard;

    public LEDSubsystem() {}

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
  public void disco(double lightnum) {
    blinkin.set(lightnum);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (target == 1) {
        disco(0.77); //0.77 = green
    } else if (intake.distOnboard.getDistance().getValueAsDouble() < 5){ //May need to change number.
        disco(0.81); //0.81 = aqua
    } else {
        disco(0.41); //0.41 = blue & gold?
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}