package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetRobotStateCommand extends SequentialCommandGroup{
    public static enum ROBOT_STATE {
        DEFAULT,
        AUTODEFAULT,
        TRENCH,

        INTAKE,
        OUTTAKE,
        SPIT,
        AGITATE,
        UN_AGITATE,

        AUTOSTARTSHOOT,
        HUB_FOCUS,
        HUB_SHOOT
    }

    public SetRobotStateCommand(ROBOT_STATE state) {
        switch (state) {
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
            case AUTOSTARTSHOOT:
                addCommands(RobotStateCommands.autoShootStateCommand());
                break;
            case AGITATE:
                addCommands(RobotStateCommands.agitateState());
                break;
            case UN_AGITATE:
                addCommands(RobotStateCommands.unAgitate());
                break;
            case HUB_FOCUS:
                addCommands(RobotStateCommands.hubFocusState());
                break;
            case HUB_SHOOT:
                addCommands(RobotStateCommands.shootHubState());
                break;
        }
    }
}