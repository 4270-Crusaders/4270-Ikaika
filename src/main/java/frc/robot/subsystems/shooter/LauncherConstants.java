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

public class LauncherConstants {
  public static final double TURRET_HEIGHT = 0.4826; // inches multiplied by meter conversion

  //TODO -> Double Check Values of offsets
  public static final double SHOOTER_X_OFFSET = -0.2032;
  public static final double SHOOTER_Y_OFFSET = -0.1905;

  public static Transform3d robotToTurret = new Transform3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET, TURRET_HEIGHT, Rotation3d.kZero);

  public static Translation2d passPointLeft = new Translation2d(3.67,6);
  public static Translation2d passPointRight = new Translation2d(3.67,2.043);

  private LauncherConstants() {}
}