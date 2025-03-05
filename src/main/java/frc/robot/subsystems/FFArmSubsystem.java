package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;

public class FFArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax shouldermotor = new SparkMax(41, MotorType.kBrushless);
  SparkMax wristmotor = new SparkMax(42, MotorType.kBrushless);
  private PIDController shoulderpid = new PIDController(0.02, 0, 0);
  private AbsoluteEncoder shoulderencoder;
  private SparkClosedLoopController wristpid;
  private AbsoluteEncoder wristencoder;
  private AbsoluteEncoderConfig armcoderconfig;
  private AbsoluteEncoderConfig wristcoderconfig;
  private SparkMaxConfig shoulderconfig;
  private SparkMaxConfig wristconfig;
  public int coralpos = 0;
  public int algaepos = 0;
  public double armgoal = 0;
  public double wristgoal = 0;
  public int count = 0;
  private ArmFeedforward armff = new ArmFeedforward(2.0315, 4.0802, 0.00001);

  private double feedforward;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutAngle m_angle = Degrees.mutable(0);

  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);

  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(  voltage -> {
      shouldermotor.setVoltage(voltage);
    },  log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shouldermotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(shoulderencoder.getPosition(), Degrees))
                    
                    .angularVelocity(
                        m_velocity.mut_replace(shoulderencoder.getVelocity(), DegreesPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public FFArmSubsystem() {
   // shoulderpid = shouldermotor.getClosedLoopController();
    shoulderencoder = shouldermotor.getAbsoluteEncoder();
    wristpid = wristmotor.getClosedLoopController();
    wristencoder = wristmotor.getAbsoluteEncoder();
    shoulderconfig = new SparkMaxConfig();
    wristconfig = new SparkMaxConfig();
    armcoderconfig = new AbsoluteEncoderConfig();
    wristcoderconfig = new AbsoluteEncoderConfig();
    //PIDController shoulderpid = new PIDController(0.02, 0, 0);
    //ArmFeedforward armff = new ArmFeedforward(2.0315, 4.0802, 0.00001);


    shoulderconfig.inverted(true);

    shoulderconfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(360)
        .velocityConversionFactor(360);

    shoulderconfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.02)
        .i(0.0)
        .d(0.0)
        .outputRange(-0.1, 0.3);
        shoulderconfig.softLimit
        .forwardSoftLimit(ArmConstants.kForwardLimit[0])
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ArmConstants.kReverseLimit[0])
        .reverseSoftLimitEnabled(true);
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    shouldermotor.configure(shoulderconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //  shouldermotor.configure
    wristconfig.absoluteEncoder
    .positionConversionFactor(360)
    .inverted(true)
    .velocityConversionFactor(360);

wristconfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.006)
    .i(0.0)
    .d(0.0)
    .outputRange(-0.2, 0.2);
    wristconfig.softLimit
        .forwardSoftLimit(ArmConstants.kForwardLimit[1])
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ArmConstants.kReverseLimit[1])
        .reverseSoftLimitEnabled(true);
/*
 * Apply the configuration to the SPARK MAX.
 *
 * kResetSafeParameters is used to get the SPARK MAX to a known state. This
 * is useful in case the SPARK MAX is replaced.
 *
 * kPersistParameters is used to ensure the configuration is not lost when
 * the SPARK MAX loses power. This is useful for power cycles that may occur
 * mid-operation.
 */
wristmotor.configure(wristconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
  public void flex(double shoulderangle, double wristangle) {
   // shoulderpid.setReference(shoulderangle,ControlType.kPosition);
   //wristpid.setReference(wristangle,ControlType.kPosition);
   armgoal = shoulderangle;
   wristgoal = wristangle;

  }

  public void coralCycle(boolean up) {
    count++;
    if (count == 2) {
      count = 0; 
      return;
    } else {
      if (up) {
        coralpos = (coralpos++ > 4) ? 1 : coralpos++;
      } else {
        coralpos = (coralpos-- < 1) ? 4 : coralpos--;
      }
      switch(coralpos) {
        case 0:
          flex(ArmConstants.kStow[0], ArmConstants.kStow[1]);
          break;
        case 1:
          flex(ArmConstants.kReef1[0], ArmConstants.kReef1[1]); 
          break;
        case 2:
          flex(ArmConstants.kReef2[0], ArmConstants.kReef2[1]);
          break;
        case 3:
          flex(ArmConstants.kReef3[0], ArmConstants.kReef3[1]);
          break;
        case 4:
          flex(ArmConstants.kReef4[0], ArmConstants.kReef4[1]);
          break;
      }
      SmartDashboard.putNumber("coral_cycle", coralpos);  
    }
    
  }

  public void algaeCycle() {
    algaepos = (algaepos++ > 1) ? 0 : algaepos++;
    switch(algaepos) {
      case 0:
        flex(ArmConstants.kAlgae1[0], ArmConstants.kAlgae1[1]);
        break;
      case 1:
        flex(ArmConstants.kAlgae2[0], ArmConstants.kAlgae2[1]);
        break;
    }
    SmartDashboard.putNumber("algae_cycle", algaepos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armangle", shouldermotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("wristangle", wristmotor.getAbsoluteEncoder().getPosition());
    //SmartDashboard.putNumber("armconv", );
    SmartDashboard.putNumber("armset", armgoal);
    SmartDashboard.putNumber("wristset", wristgoal);
   // shoulderpid.setReference(armgoal,ControlType.kPosition);
    // if (shouldermotor.getAbsoluteEncoder().getPosition() < 25) {
    //   wristpid.setReference(20,ControlType.kPosition);
    // } else {
    //   wristpid.setReference(wristgoal,ControlType.kPosition);
    // }
    if (wristmotor.getAbsoluteEncoder().getPosition() > 40 && armgoal <30) {
      //feedforward = armff.calculate(Math.toRadians(shoulderencoder.getPosition()), 0.8);
      shouldermotor.setVoltage(MathUtil.clamp((shoulderpid.calculate(shoulderencoder.getPosition(), 30) + 0), -7.0, 7.0));
        } else {
      //feedforward = armff.calculate(Math.toRadians(shoulderencoder.getPosition()), 0.8, 0.8);
      shouldermotor.setVoltage(MathUtil.clamp((armff.calculate(Math.toRadians(shoulderencoder.getPosition()-123),0.8) + shoulderpid.calculate(shoulderencoder.getPosition(), armgoal)), -7.0, 7.0));
    }
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}