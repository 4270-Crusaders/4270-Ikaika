// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {
  public static final double TURRET_HEIGHT = Units.inchesToMeters(20.5); // inches multiplied by meter conversion

  //TODO -> Double Check Values of offsets
  public static final double SHOOTER_X_OFFSET = Units.inchesToMeters(-7.75); //Right
  public static final double SHOOTER_Y_OFFSET = Units.inchesToMeters(-5.75); //Forward

  public static Transform3d robotToTurret = new Transform3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET, TURRET_HEIGHT, Rotation3d.kZero);

  public static Translation2d passPointLeft = new Translation2d(3.67,6);
  public static Translation2d passPointRight = new Translation2d(3.67,2.043);

  public static double passPoint = 4.425;

  public final static double TrenchZoneStart = 4;

  public final static double INCREASE = 0;

  private LauncherConstants() {}
}