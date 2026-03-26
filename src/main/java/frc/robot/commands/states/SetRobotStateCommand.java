package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.states.RobotStateCommands.RobotState;

/**
 * Schedules a composed {@link RobotStateCommands#commandFor(RobotState)} group. Kept as a named
 * type for OI bindings.
 */
public class SetRobotStateCommand extends SequentialCommandGroup {

  public SetRobotStateCommand(RobotState state) {
    super(RobotStateCommands.commandFor(state));
  }
}
