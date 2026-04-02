// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem with an extra callback after {@link edu.wpi.first.wpilibj2.command.CommandScheduler}
 * finishes so hardware outputs are applied after command {@code execute()} runs.
 */
public abstract class FullSubsystem extends SubsystemBase {
  private static final List<FullSubsystem> instances = new ArrayList<>();

  protected FullSubsystem() {
    super();
    instances.add(this);
  }

  protected FullSubsystem(String name) {
    super(name);
    instances.add(this);
  }

  /** Apply motor/controller outputs; invoked once per loop after the scheduler. */
  public abstract void periodicAfterScheduler();

  /**
   * Invokes {@link #periodicAfterScheduler()} on every registered full subsystem when the robot is
   * enabled. While disabled, skips hardware writes to reduce CAN load and main-loop time.
   */
  public static void runAllPeriodicAfterScheduler() {
    if (DriverStation.isDisabled()) {
      return;
    }
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
