package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetRobotStateCommand extends SequentialCommandGroup{
    public static enum ROBOT_STATE {
        DEFAULT,
        AUTODEFAULT,
        TRENCH,

        INTAKE,
        OUTTAKE,
        STOP_INTAKE,
        SPIT,
        AGITATE,
        UN_AGITATE,

        SHOOT,
        AIM,

        STOP_SHOOT,

        AUTO_SHOOT_HUB,
        AUTO_SHOOT_PASS,
        AUTO_AIM_HUB,
        AUTO_AIM_PASS,
    }

    public SetRobotStateCommand(ROBOT_STATE state) {
        switch (state) {
            case STOP_SHOOT:
                addCommands(RobotStateCommands.stopShootState());
                break;
            case STOP_INTAKE:
                addCommands(RobotStateCommands.stopIntakeState());
                break;
            case DEFAULT:
                addCommands(RobotStateCommands.defaultState());
                break;
            case AUTODEFAULT:
                addCommands(RobotStateCommands.autoDefaultState());
                break;
            case TRENCH:
                addCommands(RobotStateCommands.trenchState());
                break;
            case INTAKE:
                addCommands(RobotStateCommands.intakeState());
                break;
            case OUTTAKE:
                addCommands(RobotStateCommands.outtakeState());
                break;
            case SPIT:
                addCommands(RobotStateCommands.spitState());
                break;
            case AGITATE:
                addCommands(RobotStateCommands.agitateState());
                break;
            case UN_AGITATE:
                addCommands(RobotStateCommands.unAgitate());
                break;
            case AIM:
                addCommands(RobotStateCommands.aimState());
                break;
            case SHOOT:
                addCommands(RobotStateCommands.shootState());
                break;
            case AUTO_AIM_HUB:
                addCommands(RobotStateCommands.autoAimHubState());
                break;
            case AUTO_SHOOT_HUB:
                addCommands(RobotStateCommands.autoShootHubState());
                break;
            case AUTO_AIM_PASS:
                addCommands(RobotStateCommands.autoAimPassState());
                break;
            case AUTO_SHOOT_PASS:
                addCommands(RobotStateCommands.autoShootPassState());
                break;
        }
    }
}