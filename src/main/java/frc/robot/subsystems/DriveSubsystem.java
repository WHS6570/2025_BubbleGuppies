// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
 // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
PIDController rotpid = new PIDController(0.1, 0, 0);
PIDController drivepid = new PIDController(0.75, 0, 0.15);
PIDController strafepid = new PIDController(1, 0, 0.15);
private String limelight = "limelight";
public double ytrim = 0;
public int color = 0;
RobotConfig config;

static Spark blinkin = new Spark(0);
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // Usage reporting for MAXSwerve template
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            //this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

            (speeds) -> driveRobotRelative2(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.75, 0.0, 0.15), // Translation PID constants
                    new PIDConstants(0.75, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    
    // Configure AutoBuilder last
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        double llacquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double lllacquired = NetworkTableInstance.getDefault().getTable("limelight-lesser").getEntry("tv").getDouble(0);
        SmartDashboard.putNumber("heading",getHeading());
        SmartDashboard.putNumber("ytrim", ytrim);
        SmartDashboard.putNumber("color",color);
        SmartDashboard.putNumber("Acquired?", llacquired);
        if (color == 2) {
          blinkin.set(0.73);
        } else if (llacquired==1 || lllacquired==1) {
          blinkin.set(0.57);
        } else {
          blinkin.set(0.41);
        }
        //blinkin.set(0.57);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // Pose2d temppose = new Pose2d(-m_odometry.getPoseMeters().getX(), -m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation());
    // return temppose;
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  public void autodrive(double xSpeed, double ySpeed, double rot) {
    double xSpeedDelivered = -xSpeed;
    double ySpeedDelivered = -ySpeed;
    double rotDelivered = rot;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String mode) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
   // if (mode == "approach") {
      limelight = "limelight-lesser";
    //} else {
     // limelight = "limelight";
    //}
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

  //   if (mode == "approach") {
  //     xSpeedDelivered = -drivepid.calculate(targpose[2], -1.1) * DriveConstants.kMaxSpeedMetersPerSecond;
  //     xSpeedDelivered = MathUtil.clamp(xSpeedDelivered, -2.4, 2.4);
  //  // xSpeedDelivered = 0;
  //     if (targpose[0] >= 0.03 || targpose[0] <= -0.03) {
  //       ySpeedDelivered = -strafepid.calculate(targpose[0], 0) * DriveConstants.kMaxSpeedMetersPerSecond;
  //       ySpeedDelivered = MathUtil.clamp(ySpeedDelivered, -2.4, 2.4);
  //     } else {
  //       ySpeedDelivered = 0;
  //       }
  //   ySpeedDelivered = 0;
  //    rotDelivered = rotpid.calculate(rx, 0);
  //   //  rotDelivered = 0;
  //  } else if (mode == "square") {
  //     xSpeedDelivered = 0;
  //    // ySpeedDelivered = drivepid.calculate(targpose[4], 0) * DriveConstants.kMaxSpeedMetersPerSecond;
  //    ySpeedDelivered = -strafepid.calculate(targpose[0], 0) * DriveConstants.kMaxSpeedMetersPerSecond;
  //     rotDelivered = rotpid.calculate(degoffset, 0);
  //    }
SmartDashboard.putNumber("tz", targpose[2]);
SmartDashboard.putNumber("tx", targpose[0]);
SmartDashboard.putNumber("yaw", targpose[4]);
SmartDashboard.putNumber("xspeedd", xSpeedDelivered);
SmartDashboard.putNumber("yspeedd", ySpeedDelivered);
SmartDashboard.putNumber("rotd", rotDelivered);
SmartDashboard.putNumber("degoffset", degoffset);
SmartDashboard.putNumber("rangeval", range);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeedDelivered, -ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(-xSpeedDelivered, -ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double getytrim() {
    return ytrim;
  }

  public void setytrim(double change) {
    ytrim = ytrim + change;
  }

  public void setcolor(int newcolor){
    color = newcolor;
  }
  public void setlimelight(String ll){
    limelight = ll;
  }
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }
  public void driveRobotRelative(ChassisSpeeds speeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees((-m_gyro.getAngle())));
    //ChassisSpeeds targetSpeeds = speeds.unaryMinus();
    //-->targetSpeeds = ChassisSpeeds.discretize(ChassisSpeeds.fromRobotRelativeSpeeds(targetSpeeds, Rotation2d.fromDegrees(-m_gyro.getAngle())), 0.2);
    
    //Welling - So, this was a suggestion to double desaturate wheel speeds. The next 4 lines are new...
    //var tmpStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    //SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, AutoConstants.kMaxSpeedMetersPerSecond);
    // convert back to chassis speeds
    //speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(tmpStates);
    //Welling - Here is where we cut if this doesn't work.
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(-getHeading())), 0.2);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, AutoConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative2(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
 
  public void overridefb() {
    PPHolonomicDriveController.overrideXFeedback(() -> { return 0.0;});
    PPHolonomicDriveController.overrideYFeedback(() -> { return 0.0;});
  }

  public void clearoverride() {
    PPHolonomicDriveController.clearFeedbackOverrides();
  }
}
