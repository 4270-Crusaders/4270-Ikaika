package frc.robot.subsystems.intake.intakeRollers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

public class IntakeRollers { 
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    public enum IntakeRollersGoal {
        ZERO(new LoggedTunableNumber("Intake/Rollers/Goals/Zero", 0.0)),
        INTAKE(new LoggedTunableNumber("Intake/Rollers/Goals/Intake", 10)),
        OUTTAKE(new LoggedTunableNumber("Intake/Rollers/Goals/Outtake", -5)),
        AGITATE(new LoggedTunableNumber("Intake/Rollers/Goals/Agitate", 2)),
        CUSTOM(new LoggedTunableNumber("Intake/Rollers/Goals/Custom", 0));

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
        Logger.processInputs("Intake/Rollers", inputs);

        if (closedLoop){
            goalVoltage = goalSetpoint.getVoltage();
            io.runSetVoltage(goalVoltage);
        }

        nearGoal = EqualsUtil.epsilonEquals(inputs.appliedVolts, goalVoltage,1);
        Logger.recordOutput("Intake/Rollers/nearGoal", nearGoal);
    }

    public void Setpoint(IntakeRollersGoal goalSetPoint) {
        setGoalSetPoint(goalSetPoint);
    };
}