package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax shouldermotor = new SparkMax(41, MotorType.kBrushless);
  SparkMax wristmotor = new SparkMax(42, MotorType.kBrushless);
  private SparkClosedLoopController shoulderpid;
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
  public static String goal = "CS";
  public boolean rloffset = false;
  public double atrim = 0;

  public ArmSubsystem() {
    shoulderpid = shouldermotor.getClosedLoopController();
    shoulderencoder = shouldermotor.getAbsoluteEncoder();
    wristpid = wristmotor.getClosedLoopController();
    wristencoder = wristmotor.getAbsoluteEncoder();
    shoulderconfig = new SparkMaxConfig();
    wristconfig = new SparkMaxConfig();
    armcoderconfig = new AbsoluteEncoderConfig();
    wristcoderconfig = new AbsoluteEncoderConfig();


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
    .outputRange(-0.1, 0.1);
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
  public void flex(double shoulderangle, double wristangle, String targ, boolean resetcoral, boolean resetalgae) {
   // shoulderpid.setReference(shoulderangle,ControlType.kPosition);
   //wristpid.setReference(wristangle,ControlType.kPosition);
   armgoal = shoulderangle;
   wristgoal = wristangle;
   goal = targ;
   if (resetcoral) {
    coralpos = 0;
   }
   if (resetalgae) {
    algaepos = 0;
   }

  }

  // public void coralCycle(boolean up) {
  //   count++;
  //   if (count == 2) {
  //     count = 0; 
  //     return;
  //   } else {
  //     if (up) {
  //       coralpos = (coralpos++ > 4) ? 1 : coralpos++;
  //     } else {
  //       coralpos = (coralpos-- < 1) ? 4 : coralpos--;
  //     }
  //     switch(coralpos) {
  //       case 0:
  //         flex(ArmConstants.kStow[0], ArmConstants.kStow[1]);
  //         break;
  //       case 1:
  //         flex(ArmConstants.kReef1[0], ArmConstants.kReef1[1]); 
  //         break;
  //       case 2:
  //         flex(ArmConstants.kReef2[0], ArmConstants.kReef2[1]);
  //         break;
  //       case 3:
  //         flex(ArmConstants.kReef3[0], ArmConstants.kReef3[1]);
  //         break;
  //       case 4:
  //         flex(ArmConstants.kReef4[0], ArmConstants.kReef4[1]);
  //         break;
  //     }
  //     SmartDashboard.putNumber("coral_cycle", coralpos);  
  //   }
    
  // }

  // public void algaeCycle() {
  //   algaepos = (algaepos++ > 1) ? 0 : algaepos++;
  //   switch(algaepos) {
  //     case 0:
  //       flex(ArmConstants.kAlgae1[0], ArmConstants.kAlgae1[1]);
  //       break;
  //     case 1:
  //       flex(ArmConstants.kAlgae2[0], ArmConstants.kAlgae2[1]);
  //       break;
  //   }
  //   SmartDashboard.putNumber("algae_cycle", algaepos);
  // }
  public String goalcheck() {
    return goal;
  }

  public double getatrim() {
    return atrim;
  }

  public boolean offsetcheck() {
    return rloffset;
  }

  public void setatrim(double change) {
    atrim = atrim + change;
  }

  public void offsetset(boolean leftright) {
    rloffset = leftright;
  }
  public int getcoralpos() {
    return coralpos;
  }

  public void setcoralpos(int change) {
    coralpos = change;
  }

  public double goaldistance() {
    switch (goal) {
      case "CS":
        return -0.96;
        
      case "C1":
        return -1;
        
      case "C2":
        return -0.85;
        
      case "C3":
        return -0.9;
        
      case "C4":
        return -1.05;
        
      case "A1":
        return -1.0;
        
      case "A2":
        return -1.00;
        
      case "AP":
        return -1.00;
      default:
        return -1.2;
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armangle", shouldermotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("wristangle", wristmotor.getAbsoluteEncoder().getPosition());
    //SmartDashboard.putNumber("armconv", );
    SmartDashboard.putNumber("armset", armgoal);
    SmartDashboard.putNumber("wristset", wristgoal);
    SmartDashboard.putNumber("atrim", atrim);

   // shoulderpid.setReference(armgoal,ControlType.kPosition);
   if (goal=="rest") {
    wristpid.setReference(200,ControlType.kPosition);
   } else {
      if (shouldermotor.getAbsoluteEncoder().getPosition() < 25) {
        wristpid.setReference(20,ControlType.kPosition);
      } else {
       wristpid.setReference(wristgoal,ControlType.kPosition);
      }}
    if (goal=="rest") {
      shouldermotor.setVoltage(0);
    } else {
      if (wristmotor.getAbsoluteEncoder().getPosition() > 40 && armgoal < 25) {
       shoulderpid.setReference(30,ControlType.kPosition);
      } else {
        shoulderpid.setReference(armgoal + atrim, ControlType.kPosition);
      }}
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}