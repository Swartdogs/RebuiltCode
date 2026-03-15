package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.Turret.TurretState;

public record ShotSolution(boolean valid, Angle turretAngle, Distance distance, TurretState mode, Pose2d targetPose, boolean hasVisionTarget)
{
}
