// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import commands.AlgaeCycle;
import commands.AutoA1;
import commands.AutoA2;
import commands.AutoBackup;
import commands.AutoC1;
import commands.AutoC2;
import commands.AutoC3;
import commands.AutoClearOverride;
import commands.AutoCoralAdjust;
import commands.AutoDrive;
import commands.AutoFBOverride;
import commands.AutoStow;
import commands.AutoStowAlgae;
import commands.AutoGrab;
import commands.AutoGrabAlgae;
import commands.AutoHoldAlgae;
import commands.AutoLeft;
import commands.AutoRight;
import commands.AutoShoot;
import commands.AutoShootAlgae;
import commands.AutoSource;
import commands.CoralCycle;
import commands.ResetLights;
import commands.SetTrim;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FFArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import frc.robot.subsystems.LEDSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
    private final SendableChooser<Command> autoChooser;

  //private final LEDSubsystem m_robotLED = new LEDSubsystem();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_shooterController = new XboxController(OIConstants.kShooterControllerPort);  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
// Build an auto chooser. This will use Commands.none() as the default option.
    NamedCommands.registerCommand("AutoDrive", new AutoDrive("approach", m_robotArm, m_robotDrive));
    NamedCommands.registerCommand("AutoGrab", new AutoGrab(m_robotIntake));
    NamedCommands.registerCommand("AutoShoot", new AutoShoot(m_robotIntake, m_robotArm));
    NamedCommands.registerCommand("AutoShootAlgae", new AutoShootAlgae(m_robotIntake, m_robotArm));
    NamedCommands.registerCommand("AutoStow", new AutoStow(m_robotArm));
    NamedCommands.registerCommand("AutoStowAlgae", new AutoStowAlgae(m_robotArm));
    NamedCommands.registerCommand("AutoC1", new AutoC1(m_robotArm));
    NamedCommands.registerCommand("AutoC2", new AutoC2(m_robotArm));
    NamedCommands.registerCommand("AutoC3", new AutoC3(m_robotArm));
    NamedCommands.registerCommand("AutoA1", new AutoA1(m_robotArm));
    NamedCommands.registerCommand("AutoA2", new AutoA2(m_robotArm));
    NamedCommands.registerCommand("AutoCoralAdjust", new AutoCoralAdjust(m_robotIntake));
    NamedCommands.registerCommand("AutoRight", new AutoRight(m_robotArm));
    NamedCommands.registerCommand("AutoLeft", new AutoLeft(m_robotArm));
    NamedCommands.registerCommand("AutoSource", new AutoSource(m_robotArm));
    NamedCommands.registerCommand("AutoGrabAlgae", new AutoGrabAlgae(m_robotIntake));
    NamedCommands.registerCommand("AutoHoldAlgae", new AutoHoldAlgae(m_robotIntake));
    NamedCommands.registerCommand("AutoFBOverride", new AutoFBOverride(m_robotDrive));
    NamedCommands.registerCommand("AutoBackup", new AutoBackup(m_robotDrive));
    NamedCommands.registerCommand("ResetLights", new ResetLights(m_robotDrive));
    NamedCommands.registerCommand("AutoClearOverrides", new AutoClearOverride(m_robotDrive));
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

   // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() * Math.abs(m_driverController.getRightX()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                true,
                "yes"),
            m_robotDrive));
    m_robotIntake.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotIntake.spinny(m_shooterController.getLeftTriggerAxis()-m_shooterController.getRightTriggerAxis()),
            m_robotIntake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
     new JoystickButton(m_driverController, Button.kX.value)
         .whileTrue(new AutoDrive("approach", m_robotArm, m_robotDrive));
    new JoystickButton(m_driverController, Button.kY.value)
         .whileTrue(new RunCommand(
            () -> m_robotDrive.setcolor(0),
             m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() * Math.abs(m_driverController.getRightX()))*(1-(m_driverController.getRightTriggerAxis()*0.8)), OIConstants.kDriveDeadband),
                false,
                "yes"),
            m_robotDrive));


        //Stow arm
    new JoystickButton(m_shooterController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotArm.flex(ArmConstants.kStow[0], ArmConstants.kStow[1], "ST", true, true)
        ));


        //Algae processor
    new JoystickButton(m_shooterController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotArm.flex(ArmConstants.kProcesser[0], ArmConstants.kProcesser[1], "AP", true, true)
        ));


        //Intake from source
    new JoystickButton(m_shooterController, Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_robotArm.flex(ArmConstants.kSource[0], ArmConstants.kSource[1], "CS", true, true)
        ));

    new JoystickButton(m_shooterController, Button.kBack.value)
        .whileTrue(new AutoGrab(m_robotIntake)
        );
        new JoystickButton(m_shooterController, Button.kRightStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotArm.flex(12, 200, "apickup", true, true), m_robotArm)
        );
        new JoystickButton(m_shooterController, Button.kLeftStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotIntake.trough(), m_robotArm)
        );

        //Cycle through algae positions
    new JoystickButton(m_shooterController, Button.kB.value)
        .whileTrue(new AlgaeCycle(m_robotArm)
        );
//     new JoystickButton(m_shooterController, Button.kA.value)
//     .whileTrue((m_robotArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));


//     //Algae processor
// new JoystickButton(m_shooterController, Button.kX.value)
//     .whileTrue((m_robotArm.sysIdDynamic(SysIdRoutine.Direction.kReverse)));
    


//     //Intake from source
// new JoystickButton(m_shooterController, Button.kY.value)
//     .whileTrue((m_robotArm.sysIdDynamic(SysIdRoutine.Direction.kForward)));


//     //Cycle through algae positions
// new JoystickButton(m_shooterController, Button.kB.value)
//     .whileTrue((m_robotArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));

    //Cycle through algae positions
    new POVButton(m_shooterController, 0)
        .whileTrue(new CoralCycle(true, m_robotArm)
        );
    new POVButton(m_shooterController, 180)
        .whileTrue(new CoralCycle(false, m_robotArm)
        );
    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
        () -> m_robotClimb.pushnpull(true)
    ));

        new JoystickButton(m_shooterController, Button.kRightBumper.value)
    .whileTrue(new SetTrim(1, 0, m_robotArm, m_robotDrive)
);
        new JoystickButton(m_shooterController, Button.kLeftBumper.value)
    .whileTrue(new SetTrim(-1, 0, m_robotArm, m_robotDrive)
);
new JoystickButton(m_driverController, Button.kA.value)
.whileTrue(new SetTrim(0, -0.01, m_robotArm, m_robotDrive)
);
new JoystickButton(m_driverController, Button.kB.value)
.whileTrue(new SetTrim(0, 0.01, m_robotArm, m_robotDrive)
);
    new JoystickButton(m_driverController, Button.kBack.value)
        .whileTrue(new RunCommand(
        () -> m_robotClimb.pushnpull(false)
    ));
    new JoystickButton(m_shooterController, Button.kStart.value)
    .whileTrue(new RunCommand(
    () -> m_robotArm.flex(8, 161, "rest", true, true)
));
    new POVButton(m_driverController, 270)
        .whileTrue(new RunCommand(
        () -> m_robotArm.offsetset(false), m_robotArm)
        );
    new POVButton(m_driverController, 90)
    .whileTrue(new RunCommand(
        () -> m_robotArm.offsetset(true), m_robotArm)
        );
  //  new JoystickButton(m_shooterController, Button.k)
  }
//public class RobotContainer() {
   // public RobotContainer() {
        // Subsystem initialization
       // swerve = new Swerve();
       // DriveSubsystem = new DriveSubsystem();

        // Register Named Commands
       // NamedCommands.registerCommand("", swerve.autoBalanceCommand());
       // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
       // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

        // Do all other initialization
       // configureButtonBindings();

        // ...
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(DriveConstants.kDriveKinematics);

//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         m_robotDrive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         m_robotDrive::setModuleStates,
//         m_robotDrive);

//     // Reset odometry to the starting pose of the trajectory.
//     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, " "));
//   }

