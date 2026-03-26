// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.states.SetRobotStateCommand;
import frc.robot.commands.states.RobotStateCommands.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.agitator.AgitatorIO;
import frc.robot.subsystems.indexer.agitator.AgitatorIOTalonFX;
import frc.robot.subsystems.indexer.conveyor.ConveyorIO;
import frc.robot.subsystems.indexer.conveyor.ConveyorIOTalonFX;
import frc.robot.subsystems.indexer.kicker.KickerIO;
import frc.robot.subsystems.indexer.kicker.KickerIOTalonFX;
import frc.robot.subsystems.indexer.rollers.RollersIO;
import frc.robot.subsystems.indexer.rollers.RollersIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollerIOTalonFX;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIO;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  @SuppressWarnings("unused")
  private final Vision vision;
  public static Drive drive;
  public static Shooter shooter;
  public static Intake intake;
  public static Indexer indexer;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorController = new CommandJoystick(1);

  public static LoggedDashboardChooser<Command> autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(cameraFrontName, drive::getRotation),
            new VisionIOLimelight(cameraLeftName, drive::getRotation),
            new VisionIOLimelight(cameraRightName, drive::getRotation));

        shooter = new Shooter(new FlywheelIOTalonFX(), new TurretIOTalonFX(), new HoodIOTalonFX());
        indexer = new Indexer(new AgitatorIOTalonFX(), new KickerIOTalonFX(), new ConveyorIOTalonFX(), new RollersIOTalonFX());
        intake = new Intake(new IntakeRollerIOTalonFX(), new IntakeWristIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(cameraFrontName, robotToFrontCam, drive::getPose),
                new VisionIOPhotonVisionSim(cameraLeftName, robotToLeftCam, drive::getPose),
                new VisionIOPhotonVisionSim(cameraRightName, robotToRightCam, drive::getPose));
        shooter = new Shooter(
          new FlywheelIO() {},
          new TurretIO() {},
          new HoodIO() {}
        );
        indexer = new Indexer(
          new AgitatorIO(){},
          new KickerIO(){},
          new ConveyorIO(){},
          new RollersIO(){}
        );
        intake = new Intake(
          new IntakeRollersIO(){},
          new IntakeWristIO(){}
        );
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(
          new FlywheelIO() {},
          new TurretIO() {},
          new HoodIO() {});
        indexer = new Indexer(
          new AgitatorIO(){},
          new KickerIO(){},
          new ConveyorIO(){},
          new RollersIO(){}
        );
        intake = new Intake(
          new IntakeRollersIO(){},
          new IntakeWristIO(){}
        );
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
    registerNamedCommand();
    autoSelector = new LoggedDashboardChooser<>("Auto Selection", AutoBuilder.buildAutoChooser());
  }

  /** Runs after the command scheduler; keeps shooter/indexer aligned with drive. */
  public void periodic() {
    shooter.setRobotEstimatedPose(drive.getPose());
    shooter.setRobotSpeed(drive.getChassisSpeeds());
    indexer.setReadyToShoot(shooter.readyToShoot());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Tare
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Command groups must be freshly built each schedule; reusing a finished group breaks the scheduler.
    driverController
        .leftTrigger()
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.INTAKE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.DEFAULT), Set.of()));
    driverController
        .rightTrigger()
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.TELE_SHOOT), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.DEFAULT), Set.of()));
    driverController
        .povRight()
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.OUTTAKE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.DEFAULT), Set.of()));

    driverController
        .a()
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(3)
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(4)
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(5)
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(1)
        .onTrue(Commands.defer(() -> new SetRobotStateCommand(RobotState.CUSTOM), Set.of()))
        .onFalse(Commands.defer(() -> new SetRobotStateCommand(RobotState.DEFAULT), Set.of()));
  }

  void registerNamedCommand() {
    // Defer builds a fresh CommandGroup each schedule; reusing a finished group causes scheduler bugs.
    NamedCommands.registerCommand(
        "TRENCH",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.TRENCH), Set.of()));
    NamedCommands.registerCommand(
        "INTAKE",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.INTAKE), Set.of()));
    NamedCommands.registerCommand(
        "DEFAULT",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.AUTODEFAULT), Set.of()));
    NamedCommands.registerCommand(
        "HUB_FOCUS",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.HUB_FOCUS), Set.of()));
    NamedCommands.registerCommand(
        "HUB_SHOOT",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.AUTOSTARTSHOOT), Set.of()));
    NamedCommands.registerCommand(
        "PASS_FOCUS",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.PASS_FOCUS), Set.of()));
    NamedCommands.registerCommand(
        "PASS_SHOOT",
        Commands.defer(() -> new SetRobotStateCommand(RobotState.AUTO_SHOOT_PASS), Set.of()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }
}
