package frc.robot;

import java.util.stream.IntStream;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.ChoreoTraj;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Utilities;

public class Autos extends SubsystemBase
{
    private enum AutoMode
    {
        // @formatter:off
        DoNothing            ("Do Nothing"),
        ShootOnly            ("Shoot Only"),
        ShootWithDelay       ("Shoot with Delay"),
        ShootThenDrive       ("Shoot then Drive"),
        ShootWithDelayThenDrive("Shoot with Delay then Drive");
        // @formatter:on

        public final String displayName;

        private AutoMode(String name)
        {
            displayName = name;
        }
    }

    private enum StartPosition
    {
        // @formatter:off
        LeftTrench ("Left Trench",  ChoreoVars.Poses.LeftTrench),
        LeftBump   ("Left Bump",    ChoreoVars.Poses.LeftBump),
        HubStart   ("Hub",          ChoreoVars.Poses.HubStart),
        RightBump  ("Right Bump",   ChoreoVars.Poses.RightBump),
        RightTrench("Right Trench", ChoreoVars.Poses.RightTrench);

        // @formatter:on

        public String displayName;
        public Pose2d pose;

        private StartPosition(String name, Pose2d bluePose)
        {
            displayName = name;
            pose        = bluePose;
        }
    }

    private static final String                  NO_TRAJECTORY      = "None (Stay)";
    private final AutoFactory                    _autoFactory;
    private final Drive                          _driveSubsystem;
    private final Shooter                        _shooterSubsystem;
    private final SwerveRequest.FieldCentric     _autoRequest       = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController                  _xController       = new PIDController(0.0, 0.0, 0.0);
    private final PIDController                  _yController       = new PIDController(0.0, 0.0, 0.0);
    private final PIDController                  _headingController = new PIDController(0.0, 0.0, 0.0);
    private final SendableChooser<AutoMode>      _modeChooser       = new SendableChooser<AutoMode>();
    private final SendableChooser<Integer>       _autoDelay         = new SendableChooser<Integer>();
    private final SendableChooser<String>        _trajChooser       = new SendableChooser<String>();
    private final SendableChooser<StartPosition> _startChooser      = new SendableChooser<StartPosition>();
    private final Field2d                        _field;
    private StartPosition                        _startPosition     = null;

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

        _modeChooser.setDefaultOption(AutoMode.ShootOnly.displayName, AutoMode.ShootOnly);
        _modeChooser.addOption(AutoMode.ShootWithDelay.displayName, AutoMode.ShootWithDelay);
        _modeChooser.addOption(AutoMode.ShootThenDrive.displayName, AutoMode.ShootThenDrive);
        _modeChooser.addOption(AutoMode.ShootWithDelayThenDrive.displayName, AutoMode.ShootWithDelayThenDrive);
        _modeChooser.addOption(AutoMode.DoNothing.displayName, AutoMode.DoNothing);

        _autoDelay.setDefaultOption("0", 0);
        IntStream.range(1, 6).forEach(n -> _autoDelay.addOption(String.valueOf(n), n));

        _startChooser.setDefaultOption(StartPosition.LeftTrench.displayName, StartPosition.LeftTrench);
        _startChooser.addOption(StartPosition.LeftBump.displayName, StartPosition.LeftBump);
        _startChooser.addOption(StartPosition.HubStart.displayName, StartPosition.HubStart);
        _startChooser.addOption(StartPosition.RightBump.displayName, StartPosition.RightBump);
        _startChooser.addOption(StartPosition.RightTrench.displayName, StartPosition.RightTrench);

        _trajChooser.setDefaultOption(NO_TRAJECTORY, NO_TRAJECTORY);
        ChoreoTraj.ALL_TRAJECTORIES.keySet().stream().sorted().forEach(name -> _trajChooser.addOption(name, name));

        _field = new Field2d();

        SmartDashboard.putData("Auto Mode", _modeChooser);
        SmartDashboard.putData("Auto Trajectory", _trajChooser);
        SmartDashboard.putData("Auto Delay", _autoDelay);
        SmartDashboard.putData("Start Position", _startChooser);
        SmartDashboard.putData("Autonomous Mode", _field);
    }

    @Override
    public void periodic()
    {
        var currentStart = _startChooser.getSelected();

        if (_startPosition != currentStart)
        {
            _startPosition = currentStart;
            _field.setRobotPose(flip(_startPosition.pose));
        }
    }

    public Command buildAuto()
    {
        var start    = _startChooser.getSelected();
        var mode     = _modeChooser.getSelected();
        var trajName = _trajChooser.getSelected();
        var reset    = Commands.runOnce(() -> _driveSubsystem.resetPose(flip(start.pose)));
        var shoot    = _shooterSubsystem.shoot();
        var delay    = Commands.waitSeconds(_autoDelay.getSelected());
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
