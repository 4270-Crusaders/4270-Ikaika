// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final double loopOverrunThresholdScale = 1.5;
  public static final boolean logLoopTimingVerbose = false;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static Mode getRobot() {
    return currentMode;
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /**
   * Defaults for Phoenix 6 Talon FX / CANcoder config retries and status-frame rates in subsystem IO
   * implementations.
   */
  public static final class TalonFxIo {
    public static final int CONFIG_RETRY_COUNT = 5;
    public static final double CONFIG_APPLY_TIMEOUT_SEC = 0.25;
    public static final double STATUS_SIGNAL_UPDATE_HZ = 50.0;

    private TalonFxIo() {}
  }
}
