package frc.robot.subsystems.indexer.rollers;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer/Rollers/kP", IndexerConstants.RollersConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer/Rollers/kI", IndexerConstants.RollersConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer/Rollers/kD", IndexerConstants.RollersConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Indexer/Rollers/kA", IndexerConstants.RollersConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer/Rollers/kV", IndexerConstants.RollersConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer/Rollers/kS", IndexerConstants.RollersConstants.kS);

  public enum RollersGoal {
    ZERO(new LoggedTunableNumber("Indexer/Rollers/Goals/ZERO", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Rollers/Goals/AGITATE", 1500)),
    SHOOT(new LoggedTunableNumber("Indexer/Rollers/Goals/SHOOT", 6000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Rollers/Goals/OUTTAKE", -3000)),
    SPIT(new LoggedTunableNumber("Indexer/Rollers/Goals/SPIT", 2000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Rollers/Goals/CUSTOM", 100));

    private final DoubleSupplier RollersSetpointSupplier;

    private RollersGoal(DoubleSupplier RollersSetpointSupplier) {
      this.RollersSetpointSupplier = RollersSetpointSupplier;
    }

    private double getRPM() {
      return RollersSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Rollers/GoalSetpoint") private RollersGoal goalSetpoint = RollersGoal.ZERO;

  boolean closedLoop = true;

  private double goalRPM = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(RollersGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Rollers", inputs);

    // motion magic setpoint code
    if (closedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.motorMeasuredVelocityRPM, goalRPM, 5);
    Logger.recordOutput("Indexer/Rollers/nearGoal", nearGoal);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);
  }

  public void getSetVoltage(double goal) {
    setManualVoltage(goal);
  }
  ;

  public void Setpoint(RollersGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
  ;
}
