
package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer.INDEXER_STATE;
import frc.robot.subsystems.intake.Intake.INTAKE_STATE;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;


public class RobotStateCommands {
    // //TODO: fix this shit
    // public static Command home(){
    //     return new ParallelCommandGroup(
    //         Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
    //         Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
    //         Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter)
    //     );
    // }

    // public static Command testShoot(){
    //     return new ParallelCommandGroup(
    //         Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
    //         Intake.getSetStateCommand(INTAKE_STATE.SHOOT, RobotContainer.intake),
    //         Shooter.getSetStateCommand(SHOOTER_STATE.HUB, RobotContainer.shooter)
    //     );
    // }

    
    //TODO
    public static Command defaultState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Down         done
             * Indexer: Zero        done
             * Shooter: Hub         done
             * Climber: --
             */
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter)
        );
    }

    public static Command autoDefaultState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Down         done
             * Indexer: Zero        done
             * Shooter: Hub         done
             * Climber: --
             */
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOHOME, RobotContainer.shooter)
        );
    }

    public static Command trenchState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Down         done
             * Indexer: Zero        done
             * Shooter: HOME        done
             * Climber: Down
             */

            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter)
        );
    }

    //TODO
    public static Command aimState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Intake
             * Indexer: Stop   
             * Shooter: Hub        
             * Climber: --
             */
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AIM, RobotContainer.shooter)
        );
    }

    public static Command autoAimState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Intake
             * Indexer: Stop   
             * Shooter: Hub        
             * Climber: --
             */
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIM, RobotContainer.shooter)
        );
    }

    //TODO
    public static Command shootState() {
        return new ParallelCommandGroup(
            /**
             * Intake: --
             * Indexer: Stop   
             * Shooter: Pass        
             * Climber: --
             */
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AIM, RobotContainer.shooter)
        );
    }

    //TODO
    public static Command autoShootState() {
        return new ParallelCommandGroup(
            /**
             * Intake: --
             * Indexer: Shoot        done
             * Shooter: HUB          done
             * Climber: --
             */

            Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIM, RobotContainer.shooter)
        );
    }

    //TODO
    public static Command intakeState() {
        return new ParallelCommandGroup(
            /**
             * Intake: Intake           done
             * Indexer: Intake          done
             * Shooter: --
             * Climber: --
             */

            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.INTAKE, RobotContainer.indexer)
        );
    }

    //TODO
    public static Command unAgitate() {
        return new ParallelCommandGroup(
            /**
             * Intake: Intake           done
             * Indexer: Intake          done
             * Shooter: --
             * Climber: --
             */

            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake)
        );
    }

    //TODO
    public static Command outtakeState() {
        return new ParallelCommandGroup(
            /**
             * Intake: outtake           done
             * Indexer: outtake         done
             * Shooter: --            
             * Climber: --
             */

            Intake.getSetStateCommand(INTAKE_STATE.OUTTAKE, RobotContainer.intake)
           // Indexer.getSetStateCommand(INDEXER_STATE.OUTTAKE, RobotContainer.indexer)
        );
    }

    //TODO
    public static Command spitState() {
        return new SequentialCommandGroup(
            /**
             * Intake: --
             * Indexer: spit         done
             * Shooter: --
             * Climber: --
             */

            Indexer.getSetStateCommand(INDEXER_STATE.SPIT, RobotContainer.indexer)
        );
    }

    //TODO
    public static Command agitateState() {
        return new SequentialCommandGroup(
            /**
             * Intake: Agitate          done
             * Indexer: --
             * Shooter: --         
             * Climber: --
             */

            Intake.getSetStateCommand(INTAKE_STATE.AGITATE, RobotContainer.intake)
        );
    }
}
