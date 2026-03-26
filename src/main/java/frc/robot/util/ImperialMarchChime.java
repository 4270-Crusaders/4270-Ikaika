package frc.robot.util;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Plays a short Imperial March–style motif through TalonFX devices using {@link MusicTone}.
 * Register each motor with {@link #registerChimeMotor(TalonFX)} from hardware IO construction, then
 * call {@link #playAsyncIfRegistered()} from {@code robotInit()}.
 */
public final class ImperialMarchChime {
  private static final CopyOnWriteArrayList<TalonFX> chimeMotors = new CopyOnWriteArrayList<>();
  private static final AtomicBoolean playing = new AtomicBoolean(false);
  private static final AtomicBoolean chimeScheduled = new AtomicBoolean(false);

  private ImperialMarchChime() {}

  /** True while the chime thread is driving any registered motor; IO should skip conflicting setControl. */
  public static boolean isPlaying() {
    return playing.get();
  }

  /** Register a TalonFX to play the startup chime (each motor plays once, in registration order). */
  public static void registerChimeMotor(TalonFX motor) {
    if (motor != null && !chimeMotors.contains(motor)) {
      chimeMotors.add(motor);
    }
  }

  /**
   * Starts a daemon thread that plays the chime on every registered motor, sequentially. No-op on
   * non-real robots or if no motors were registered. Runs at most once per JVM boot.
   */
  public static void playAsyncIfRegistered() {
    if (!RobotBase.isReal()) {
      return;
    }
    if (!chimeScheduled.compareAndSet(false, true)) {
      return;
    }
    List<TalonFX> motors = List.copyOf(chimeMotors);
    if (motors.isEmpty()) {
      return;
    }
    Thread thread =
        new Thread(
            () -> {
              playing.set(true);
              try {
                for (TalonFX motor : motors) {
                  try {
                    playMotifOnMotor(motor);
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                  }
                  try {
                    Thread.sleep(150);
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                  }
                }
              } finally {
                for (TalonFX motor : motors) {
                  try {
                    motor.setControl(SILENCE);
                  } catch (RuntimeException ignored) {
                    // Motor may be absent on partial hardware bring-up.
                  }
                }
                playing.set(false);
              }
            },
            "ImperialMarchChime");
    thread.setDaemon(true);
    thread.start();
  }

  // Opening phrase + short tag (approximate Hz, ms). G4=392, Eb4=311.13, Bb3=233.08, G3=196, etc.
  private static final double[][] NOTES_HZ_MS = {
    {392.0, 220}, {392.0, 220}, {392.0, 220}, {311.13, 680},
    {233.08, 290}, {196.0, 290}, {311.13, 920},
    {392.0, 220}, {293.66, 220}, {349.23, 220}, {311.13, 720},
    {261.63, 420}, {311.13, 420}, {392.0, 650},
  };

  private static final MusicTone SILENCE = new MusicTone(0).withUpdateFreqHz(0);

  private static void playMotifOnMotor(TalonFX motor) throws InterruptedException {
    motor
        .getConfigurator()
        .apply(
            new AudioConfigs()
                .withAllowMusicDurDisable(true)
                .withBeepOnBoot(false)
                .withBeepOnConfig(false));
    Thread.sleep(400);
    for (double[] note : NOTES_HZ_MS) {
      motor.setControl(new MusicTone(note[0]).withUpdateFreqHz(0));
      Thread.sleep((long) note[1]);
    }
    motor.setControl(SILENCE);
  }
}
