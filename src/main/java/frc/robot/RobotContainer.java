// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.states.RobotStateCommands;
import frc.robot.commands.states.RobotStateCommands.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.indexerAgitator.IndexerAgitatorIO;
import frc.robot.subsystems.indexer.indexerAgitator.IndexerAgitatorIOTalonFX;
import frc.robot.subsystems.indexer.indexerConveyor.IndexerConveyorIO;
import frc.robot.subsystems.indexer.indexerConveyor.IndexerConveyorIOTalonFX;
import frc.robot.subsystems.indexer.indexerKicker.IndexerKickerIO;
import frc.robot.subsystems.indexer.indexerKicker.IndexerKickerIOTalonFX;
import frc.robot.subsystems.indexer.indexerRollers.IndexerRollersIO;
import frc.robot.subsystems.indexer.indexerRollers.IndexerRollersIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIO;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIOTalonFX;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
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

  /**
   * Constructed for side effects only: {@link Vision} registers with {@link
   * edu.wpi.first.wpilibj2.command.CommandScheduler} and runs {@code periodic()} each loop.
   */
  @SuppressWarnings("unused")
  private final Vision vision;
  public static Drive drive;
  public static Flywheel flywheel;
  public static Hood hood;
  public static Turret turret;
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
                new VisionIOLimelight(
                    cameraFrontName,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose().getRotation()),
                new VisionIOLimelight(
                    cameraLeftName,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose().getRotation()),
                new VisionIOLimelight(
                    cameraRightName,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose().getRotation()));
        configureShooterMechanismDefaults(
            new Flywheel(new FlywheelIOTalonFX()),
            new Turret(new TurretIOTalonFX()),
            new Hood(new HoodIOTalonFX()));
        indexer =
            new Indexer(
                new IndexerAgitatorIOTalonFX(),
                new IndexerKickerIOTalonFX(),
                new IndexerConveyorIOTalonFX(),
                new IndexerRollersIOTalonFX());
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
                new VisionIOPhotonVisionSim(
                    cameraFrontName,
                    robotToFrontCam,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose()),
                new VisionIOPhotonVisionSim(
                    cameraLeftName,
                    robotToLeftCam,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose()),
                new VisionIOPhotonVisionSim(
                    cameraRightName,
                    robotToRightCam,
                    () -> frc.robot.RobotState.getInstance().getEstimatedPose()));
        configureShooterMechanismDefaults(
            new Flywheel(new FlywheelIO() {}),
            new Turret(new TurretIO() {}),
            new Hood(new HoodIO() {}));
        indexer =
            new Indexer(
                new IndexerAgitatorIO() {},
                new IndexerKickerIO() {},
                new IndexerConveyorIO() {},
                new IndexerRollersIO() {});
        intake = new Intake(new IntakeRollerIO() {}, new IntakeWristIO() {});
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
        vision =
            new Vision(
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        configureShooterMechanismDefaults(
            new Flywheel(new FlywheelIO() {}),
            new Turret(new TurretIO() {}),
            new Hood(new HoodIO() {}));
        indexer =
            new Indexer(
                new IndexerAgitatorIO() {},
                new IndexerKickerIO() {},
                new IndexerConveyorIO() {},
                new IndexerRollersIO() {});
        intake = new Intake(new IntakeRollerIO() {}, new IntakeWristIO() {});
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
    registerNamedCommand();
    autoSelector = new LoggedDashboardChooser<>("Auto Selection", AutoBuilder.buildAutoChooser());
  }

  private void configureShooterMechanismDefaults(Flywheel fw, Turret tr, Hood hd) {
    flywheel = fw;
    turret = tr;
    hood = hd;
    flywheel.setDefaultCommand(flywheel.runTrackTargetCommand());
    hood.setDefaultCommand(hood.runTrackTargetCommand());
    turret.setDefaultCommand(turret.runTrackTargetCommand());
  }

  /**
   * Invoked from {@link Robot#robotPeriodic()} after {@code CommandScheduler.getInstance().run()} so
   * mechanism {@code periodic()} has updated sensors, and before {@link
   * frc.robot.util.FullSubsystem#runAllPeriodicAfterScheduler()}.
   */
  public static void runShooterCoordinationAfterScheduler() {
    if (flywheel != null && hood != null && turret != null) {
      ShooterCalculator.getInstance().coordinateAfterScheduler(flywheel, hood, turret);
    }
  }

  /**
   * Defines operator-interface bindings. Uses {@link Commands#defer} so each schedule builds a new
   * command graph; reusing finished groups can break the scheduler.
   */
  private void configureButtonBindings() {
    // Field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Reset odometry heading (field-relative reference)
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driverController
        .leftTrigger()
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.INTAKE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.STOP_INTAKE), Set.of()));
    driverController
        .rightTrigger()
        .whileTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.TELE_SHOOT), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.STOP_SHOOT), Set.of()));
    driverController
        .povRight()
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.OUTTAKE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.STOP_INTAKE), Set.of()));
    driverController
        .a()
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.UN_AGITATE), Set.of()));

    operatorController
        .button(3)
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(4)
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(5)
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AGITATE), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.UN_AGITATE), Set.of()));
    operatorController
        .button(1)
        .onTrue(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.CUSTOM), Set.of()))
        .onFalse(Commands.defer(() -> RobotStateCommands.commandFor(RobotState.STOP_SHOOT), Set.of()));
  }

  /** Registers PathPlanner {@link NamedCommands} used by auto routines. */
  void registerNamedCommand() {
    NamedCommands.registerCommand(
        "TRENCH",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.TRENCH), Set.of()));
    NamedCommands.registerCommand(
        "INTAKE",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.INTAKE), Set.of()));
    NamedCommands.registerCommand(
        "DEFAULT",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AUTODEFAULT), Set.of()));
    NamedCommands.registerCommand(
        "HUB_FOCUS",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.HUB_FOCUS), Set.of()));
    NamedCommands.registerCommand(
        "HUB_SHOOT",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AUTO_START_SHOOT), Set.of()));
    NamedCommands.registerCommand(
        "PASS_FOCUS",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.PASS_FOCUS), Set.of()));
    NamedCommands.registerCommand(
        "PASS_SHOOT",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AUTO_START_PASS), Set.of()));
    NamedCommands.registerCommand(
        "AGITATE",
        Commands.defer(() -> RobotStateCommands.commandFor(RobotState.AGITATE), Set.of()));
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
