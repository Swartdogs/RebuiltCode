package frc.robot;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Utilities;

public class Autos
{
    private enum AutoMode
    {
        DoNothing, ShootOnly, ShootWithDelay, ShootThenDrive, ShootWithDelayThenDrive
    }

    private enum StartPosition
    {
        LeftTrench(ChoreoVars.Poses.LeftTrench), LeftBump(ChoreoVars.Poses.LeftBump), HubStart(ChoreoVars.Poses.HubStart), RightBump(ChoreoVars.Poses.RightBump), RightTrench(ChoreoVars.Poses.RightTrench);

        public final Pose2d pose;

        private StartPosition(Pose2d bluePose)
        {
            pose = bluePose;
        }
    }

    private static final String              NO_TRAJECTORY           = "None (Stay)";
    private static final AutoMode            DEFAULT_AUTO_MODE       = AutoMode.ShootOnly;
    private static final int                 DEFAULT_AUTO_DELAY_SECS = 0;
    private static final StartPosition       DEFAULT_START_POSITION  = StartPosition.LeftTrench;
    private static final String              DEFAULT_AUTO_TRAJECTORY = NO_TRAJECTORY;
    private final AutoFactory                _autoFactory;
    private final Drive                      _driveSubsystem;
    private final Shooter                    _shooterSubsystem;
    private final SwerveRequest.FieldCentric _autoRequest            = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController              _xController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _yController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _headingController      = new PIDController(0.0, 0.0, 0.0);

    public Autos(Drive driveSubsystem, Shooter shooterSubsystem)
    {
        _driveSubsystem   = driveSubsystem;
        _shooterSubsystem = shooterSubsystem;

        _headingController.enableContinuousInput(-Math.PI, Math.PI);

        // @formatter:off
        _autoFactory = new AutoFactory(
            () -> driveSubsystem.getState().Pose,
            driveSubsystem::resetPose,
            this::followTrajectory,
            true,
            driveSubsystem
        );
        // @formatter:on
    }

    public Command buildAuto()
    {
        var start    = DEFAULT_START_POSITION;
        var mode     = DEFAULT_AUTO_MODE;
        var trajName = DEFAULT_AUTO_TRAJECTORY;
        var reset    = Commands.runOnce(() -> _driveSubsystem.resetPose(flip(start.pose)));
        var shoot    = _shooterSubsystem.shoot().withTimeout(4.0);
        var delay    = Commands.waitSeconds(DEFAULT_AUTO_DELAY_SECS);
        var drive    = NO_TRAJECTORY.equals(trajName) ? Commands.none() : _autoFactory.trajectoryCmd(trajName);

        // @formatter:off
        return switch (mode)
        {
            case DoNothing              -> reset;
            case ShootOnly              -> Commands.sequence(reset, shoot);
            case ShootWithDelay         -> Commands.sequence(reset, delay, shoot);
            case ShootThenDrive         -> Commands.sequence(reset, shoot, drive);
            case ShootWithDelayThenDrive -> Commands.sequence(reset, delay, shoot, drive);
        };
        // @formatter:on
    }

    private void followTrajectory(SwerveSample sample)
    {
        var pose = _driveSubsystem.getState().Pose;

        // @formatter:off
        _driveSubsystem.setControl(_autoRequest
            .withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x))
            .withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
            .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
        );
        // @formatter:on
    }

    private Pose2d flip(Pose2d bluePose)
    {
        var pose = bluePose;

        if (Utilities.isRedAlliance())
        {
            pose = bluePose.rotateAround(new Translation2d(GeneralConstants.FIELD_SIZE_X.div(2), GeneralConstants.FIELD_SIZE_Y.div(2)), Rotation2d.k180deg);
        }

        return pose;
    }
}
