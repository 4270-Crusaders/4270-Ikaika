package frc.robot.util;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Plays a short Imperial March–style motif through a TalonFX using {@link MusicTone}. Register the
 * speaker motor with {@link #registerChimeMotor(TalonFX)} from hardware IO construction, then call
 * {@link #playAsyncIfRegistered()} from {@code robotInit()}.
 */
public final class ImperialMarchChime {
  private static volatile TalonFX chimeMotor;
  private static final AtomicBoolean playing = new AtomicBoolean(false);

  private ImperialMarchChime() {}

  /** True while the chime thread is driving the motor; flywheel IO should not call setControl. */
  public static boolean isPlaying() {
    return playing.get();
  }

  /** Call once with the TalonFX that should buzz (e.g. flywheel lead). */
  public static void registerChimeMotor(TalonFX motor) {
    chimeMotor = motor;
  }

  /**
   * Starts a daemon thread that plays the chime once. No-op on non-real robots or if no motor was
   * registered.
   */
  public static void playAsyncIfRegistered() {
    if (!RobotBase.isReal()) {
      return;
    }
    TalonFX motor = chimeMotor;
    if (motor == null) {
      return;
    }
    Thread thread = new Thread(() -> play(motor), "ImperialMarchChime");
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

  private static void play(TalonFX motor) {
    playing.set(true);
    try {
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
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } finally {
      motor.setControl(SILENCE);
      playing.set(false);
    }
  }
}
