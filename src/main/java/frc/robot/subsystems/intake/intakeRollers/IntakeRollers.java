package frc.robot.subsystems.intake.intakeRollers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

public class IntakeRollers { 
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/Rollers/kP", IndexerConstants.RollersConstants.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/Rollers/kI", IndexerConstants.RollersConstants.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/Rollers/kD", IndexerConstants.RollersConstants.kD);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/Rollers/kA", IndexerConstants.RollersConstants.kA);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/Rollers/kV", IndexerConstants.RollersConstants.kV);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/Rollers/kS", IndexerConstants.RollersConstants.kS);

    public enum IntakeRollersGoal {
        ZERO(new LoggedTunableNumber("Intake/Roller/Goals/ZERO", 0.0)),
        INTAKE(new LoggedTunableNumber("Intake/Roller/Goals/INTAKE", 12)),
        OUTTAKE(new LoggedTunableNumber("Intake/Roller/Goals/OUTTAKE", -12)),
        AGITATE(new LoggedTunableNumber("Intake/Roller/Goals/AGITATE", 2)),
        CUSTOM(new LoggedTunableNumber("Intake/Roller/Goals/CUSTOM", 0));

        private final DoubleSupplier intakeRollerSetpointSupplier;

        private IntakeRollersGoal(DoubleSupplier intakeRollerSetpointSupplier) {
            this.intakeRollerSetpointSupplier = intakeRollerSetpointSupplier;
        }

        private double getVoltage() {
        return intakeRollerSetpointSupplier.getAsDouble();
        }
    }

    @AutoLogOutput private IntakeRollersGoal goalSetpoint = IntakeRollersGoal.ZERO;

    private boolean closedLoop = false;

    private double goalVoltage = 0.0;

    private boolean nearGoal = false;

    public void setGoalSetPoint(IntakeRollersGoal goal){
        closedLoop = true;
        this.goalSetpoint = goal;
    }

    public IntakeRollers(IntakeRollersIO io){
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);

        if (closedLoop){
            goalVoltage = goalSetpoint.getVoltage();
            io.runSetVoltage(goalVoltage);
        }

        nearGoal = EqualsUtil.epsilonEquals(inputs.appliedVolts, goalVoltage,1);
        Logger.recordOutput("Intake/Roller/nearGoal", nearGoal);
    }

    public void Setpoint(IntakeRollersGoal goalSetPoint) {
        setGoalSetPoint(goalSetPoint);
    };
}