package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Utilities;

public class Autos extends SubsystemBase
{
    private enum StartPosition
    {
        // @formatter:off
        LeftTrench ("Left Trench",  ChoreoVars.Poses.LeftTrench,  RPM.of(4500)),
        LeftBump   ("Left Bump",    ChoreoVars.Poses.LeftBump,    RPM.of(3200)),
        HubStart   ("Hub",          ChoreoVars.Poses.HubStart,    RPM.zero()),
        RightBump  ("Right Bump",   ChoreoVars.Poses.RightBump,   RPM.of(3200)),
        RightTrench("Right Trench", ChoreoVars.Poses.RightTrench, RPM.of(4500));
        // @formatter:on

        public String          displayName;
        public Pose2d          pose;
        public AngularVelocity flywheelVelocity;

        private StartPosition(String name, Pose2d bluePose, AngularVelocity velocity)
        {
            displayName      = name;
            pose             = bluePose;
            flywheelVelocity = velocity;
        }
    }

    // private final AutoFactory _autoFactory;
    private final Drive   _driveSubsystem;
    private final Shooter _shooterSubsystem;
    // private final SwerveRequest.FieldCentric _autoFollowingRequest = new
    // SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final PIDController _xController = new PIDController(0.0, 0.0, 0.0);
    // private final PIDController _yController = new PIDController(0.0, 0.0, 0.0);
    // private final PIDController _headingController = new PIDController(0.0, 0.0,
    // 0.0);
    private final SendableChooser<Integer>       _autoDelay     = new SendableChooser<Integer>();
    private final SendableChooser<StartPosition> _startChooser  = new SendableChooser<StartPosition>();
    private final String                         _shootNTKey    = "Shoot";
    private final Field2d                        _field;
    private StartPosition                        _startPosition = null;

    public Autos(Drive driveSubsystem, Shooter shooterSubsystem)
    {
        _driveSubsystem   = driveSubsystem;
        _shooterSubsystem = shooterSubsystem;

        // @formatter:off
        // _autoFactory = new AutoFactory
        // (
        //     () -> driveSubsystem.getState().Pose,
        //     driveSubsystem::resetPose,
        //     this::followTrajectory,
        //     true,
        //     driveSubsystem
        // );
        // @formatter:on

        _autoDelay.setDefaultOption("0", 0);
        IntStream.range(1, 6).forEach(n -> _autoDelay.addOption(String.valueOf(n), n));

        _startChooser.setDefaultOption(StartPosition.LeftTrench.displayName, StartPosition.LeftTrench);
        _startChooser.addOption(StartPosition.LeftBump.displayName, StartPosition.LeftBump);
        _startChooser.addOption(StartPosition.HubStart.displayName, StartPosition.HubStart);
        _startChooser.addOption(StartPosition.RightBump.displayName, StartPosition.RightBump);
        _startChooser.addOption(StartPosition.RightTrench.displayName, StartPosition.RightTrench);

        _field = new Field2d();

        SmartDashboard.putData("Auto Delay", _autoDelay);
        SmartDashboard.putData("Start Position", _startChooser);
        SmartDashboard.putData("Autonomous Mode", _field);

        if (!SmartDashboard.containsKey(_shootNTKey))
        {
            SmartDashboard.putBoolean(_shootNTKey, true);
        }
    }

    @Override
    public void periodic()
    {
        StartPosition currentStart = _startChooser.getSelected();

        if (_startPosition != currentStart)
        {
            _startPosition = currentStart;
            _field.setRobotPose(flip(_startPosition.pose));
        }
    }

    // private void followTrajectory(SwerveSample sample)
    // {
    // // Get the current pose of the robot
    // Pose2d pose = _driveSubsystem.getState().Pose;

    // // Build up "request" based on "sample"
    //     // @formatter:off
    //     _autoFollowingRequest
    //         .withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x))
    //         .withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
    //         .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
    //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    //     // @formatter:on

    // _driveSubsystem.setControl(_autoFollowingRequest);
    // }

    // public Command followPath(String pathName)
    // {
    //     // @formatter:off
    //     return Commands.sequence
    //     (
    //         _autoFactory.resetOdometry(pathName),
    //         _autoFactory.trajectoryCmd(pathName),
    //         Commands.runOnce(() -> _driveSubsystem.setControl(new SwerveRequest.Idle()), _driveSubsystem)
    //     );
    //     // @formatter:on
    // }

    public Command buildAuto()
    {
        StartPosition start = _startChooser.getSelected();

        // @formatter:off
        return Commands.sequence
        (
            Commands.runOnce(() ->
            {
                _driveSubsystem.resetPose(flip(start.pose));
                _shooterSubsystem._turret.setDisabled(true);
            }),
            Commands.waitSeconds(_autoDelay.getSelected()),
            Commands.sequence
            (
                _shooterSubsystem.setFlywheelVelocity(start.flywheelVelocity),
                Commands.waitUntil(() -> _shooterSubsystem._flywheel.atSpeed()),
                _shooterSubsystem.runFeeder()
            )
            .onlyIf(() -> SmartDashboard.getBoolean(_shootNTKey, false) && start != StartPosition.HubStart)
            .finallyDo(() ->
            {
                _shooterSubsystem._turret.setDisabled(false);
                _shooterSubsystem._flywheel.stop();
            })
        );
        // @formatter:on
    }

    private Pose2d flip(Pose2d bluePose)
    {
        Pose2d pose = bluePose;

        if (Utilities.isRedAlliance())
        {
            pose = bluePose.rotateAround(new Translation2d(GeneralConstants.FIELD_SIZE_X.div(2), GeneralConstants.FIELD_SIZE_Y.div(2)), Rotation2d.k180deg);
        }

        return pose;
    }
}
