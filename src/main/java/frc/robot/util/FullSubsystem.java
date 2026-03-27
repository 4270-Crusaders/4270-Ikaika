// Copyright (c) 2021-2026 Littleton Robotics
//
// Pattern from Mechanical-Advantage/RobotCode2026Public FullSubsystem.

package frc.robot.util;

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

  /** Invoke {@link #periodicAfterScheduler()} on every registered full subsystem. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
