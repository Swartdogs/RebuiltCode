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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
        DoNothing("Do Nothing"), DriveOnly("Drive Only"), ShootOnly("Shoot Only"), ShootWithDelay("Shoot + Delay"), ShootThenDrive("Shoot Then Drive"), ShootWithDelayThenDrive("Shoot + Delay + Drive");

        public final String displayName;

        private AutoMode(String displayName)
        {
            this.displayName = displayName;
        }
    }

    private record AutoDriveOption(String displayName, String trajectoryName, Pose2d endPoseBlue)
    {
        boolean staysPut()
        {
            return trajectoryName == null;
        }
    }

    private enum StartPosition
    {
        LeftTrench("Left Trench", ChoreoVars.Poses.LeftTrench, new AutoDriveOption("Stay Put", null, ChoreoVars.Poses.LeftTrench),
                new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftTrenchToDepot.name(), ChoreoTraj.LeftTrenchToDepot.endPoseBlue())),
        LeftBump("Left Bump", ChoreoVars.Poses.LeftBump, new AutoDriveOption("Stay Put", null, ChoreoVars.Poses.LeftBump), new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftBumpToDepot.name(), ChoreoTraj.LeftBumpToDepot.endPoseBlue()),
                new AutoDriveOption("Drive To Tower", ChoreoTraj.LeftBumpToTower.name(), ChoreoTraj.LeftBumpToTower.endPoseBlue()),
                new AutoDriveOption("Pickup From Mid", ChoreoTraj.LeftBumpPickupFromMidScore.name(), ChoreoTraj.LeftBumpPickupFromMidScore.endPoseBlue())),
        HubStart("Hub", ChoreoVars.Poses.HubStart, new AutoDriveOption("Stay Put", null, ChoreoVars.Poses.HubStart), new AutoDriveOption("Drive To Depot", ChoreoTraj.HubToDepot.name(), ChoreoTraj.HubToDepot.endPoseBlue()),
                new AutoDriveOption("Drive To Outpost", ChoreoTraj.HubToOutpost.name(), ChoreoTraj.HubToOutpost.endPoseBlue()), new AutoDriveOption("Drive To Tower", ChoreoTraj.HubToTower.name(), ChoreoTraj.HubToTower.endPoseBlue())),
        RightBump("Right Bump", ChoreoVars.Poses.RightBump, new AutoDriveOption("Stay Put", null, ChoreoVars.Poses.RightBump),
                new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightBumpToOutpost.name(), ChoreoTraj.RightBumpToOutpost.endPoseBlue()),
                new AutoDriveOption("Drive To Tower", ChoreoTraj.RightBumpToTower.name(), ChoreoTraj.RightBumpToTower.endPoseBlue()),
                new AutoDriveOption("Pickup From Mid", ChoreoTraj.RightBumpPickupFromMidScore.name(), ChoreoTraj.RightBumpPickupFromMidScore.endPoseBlue())),
        RightTrench("Right Trench", ChoreoVars.Poses.RightTrench, new AutoDriveOption("Stay Put", null, ChoreoVars.Poses.RightTrench),
                new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightTrenchToOutpost.name(), ChoreoTraj.RightTrenchToOutpost.endPoseBlue()));

        public final String            displayName;
        public final Pose2d            pose;
        public final AutoDriveOption[] driveOptions;

        private StartPosition(String displayName, Pose2d pose, AutoDriveOption... driveOptions)
        {
            this.displayName  = displayName;
            this.pose         = pose;
            this.driveOptions = driveOptions;
        }
    }

    private static final AutoMode                DEFAULT_AUTO_MODE       = AutoMode.ShootOnly;
    private static final int                     DEFAULT_AUTO_DELAY_SECS = 0;
    private static final StartPosition           DEFAULT_START_POSITION  = StartPosition.HubStart;
    private final AutoFactory                    _autoFactory;
    private final Drive                          _driveSubsystem;
    private final Shooter                        _shooterSubsystem;
    private final SwerveRequest.FieldCentric     _autoRequest            = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController                  _xController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController                  _yController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController                  _headingController      = new PIDController(0.0, 0.0, 0.0);
    private final SendableChooser<AutoMode>      _modeChooser            = new SendableChooser<>();
    private final SendableChooser<Integer>       _delayChooser           = new SendableChooser<>();
    private final SendableChooser<StartPosition> _startChooser           = new SendableChooser<>();
    private SendableChooser<AutoDriveOption>     _drivePathChooser       = new SendableChooser<>();
    private final Field2d                        _previewField           = new Field2d();
    private final Alert                          _invalidAutoAlert       = new Alert("Selected auto configuration is invalid.", AlertType.kWarning);
    private StartPosition                        _lastStartPosition;

    public Autos(Drive driveSubsystem, Shooter shooterSubsystem)
    {
        _driveSubsystem   = driveSubsystem;
        _shooterSubsystem = shooterSubsystem;

        _headingController.enableContinuousInput(-Math.PI, Math.PI);

        _autoFactory = new AutoFactory(() -> driveSubsystem.getState().Pose, driveSubsystem::resetPose, this::followTrajectory, true, driveSubsystem);

        configureChoosers();
        rebuildDrivePathChooser(DEFAULT_START_POSITION);
        updateDashboardPreview();
    }

    @Override
    public void periodic()
    {
        var selectedStart = getSelectedStartPosition();

        if (_lastStartPosition != selectedStart)
        {
            rebuildDrivePathChooser(selectedStart);
        }

        updateDashboardPreview();
    }

    public Command buildAuto()
    {
        var startPosition = getSelectedStartPosition();
        var autoMode      = getSelectedMode();
        var driveOption   = getSelectedDriveOption(startPosition);
        var autoDelaySecs = getSelectedDelaySeconds();
        var resetPose     = Commands.runOnce(() -> _driveSubsystem.resetPose(flip(startPosition.pose)));
        var shoot         = _shooterSubsystem.shoot().withTimeout(4.0);
        var delay         = Commands.waitSeconds(autoDelaySecs);
        var drive         = driveOption.staysPut() ? Commands.none() : _autoFactory.trajectoryCmd(driveOption.trajectoryName());

        return switch (autoMode)
        {
            case DoNothing -> resetPose;
            case DriveOnly -> Commands.sequence(resetPose, drive);
            case ShootOnly -> Commands.sequence(resetPose, shoot);
            case ShootWithDelay -> Commands.sequence(resetPose, delay, shoot);
            case ShootThenDrive -> Commands.sequence(resetPose, shoot, drive);
            case ShootWithDelayThenDrive -> Commands.sequence(resetPose, delay, shoot, drive);
        };
    }

    private void configureChoosers()
    {
        _modeChooser.setDefaultOption(DEFAULT_AUTO_MODE.displayName, DEFAULT_AUTO_MODE);
        _modeChooser.addOption(AutoMode.DriveOnly.displayName, AutoMode.DriveOnly);
        _modeChooser.addOption(AutoMode.ShootWithDelay.displayName, AutoMode.ShootWithDelay);
        _modeChooser.addOption(AutoMode.ShootThenDrive.displayName, AutoMode.ShootThenDrive);
        _modeChooser.addOption(AutoMode.ShootWithDelayThenDrive.displayName, AutoMode.ShootWithDelayThenDrive);
        _modeChooser.addOption(AutoMode.DoNothing.displayName, AutoMode.DoNothing);

        _delayChooser.setDefaultOption(String.valueOf(DEFAULT_AUTO_DELAY_SECS), DEFAULT_AUTO_DELAY_SECS);
        IntStream.rangeClosed(1, 5).forEach(seconds -> _delayChooser.addOption(String.valueOf(seconds), seconds));

        _startChooser.setDefaultOption(DEFAULT_START_POSITION.displayName, DEFAULT_START_POSITION);
        _startChooser.addOption(StartPosition.LeftTrench.displayName, StartPosition.LeftTrench);
        _startChooser.addOption(StartPosition.LeftBump.displayName, StartPosition.LeftBump);
        _startChooser.addOption(StartPosition.HubStart.displayName, StartPosition.HubStart);
        _startChooser.addOption(StartPosition.RightBump.displayName, StartPosition.RightBump);
        _startChooser.addOption(StartPosition.RightTrench.displayName, StartPosition.RightTrench);

        SmartDashboard.putData("Auto Mode", _modeChooser);
        SmartDashboard.putData("Auto Start Position", _startChooser);
        SmartDashboard.putData("Auto Delay", _delayChooser);
        SmartDashboard.putData("Auto Drive Path", _drivePathChooser);
        SmartDashboard.putData("Autonomous Preview", _previewField);
    }

    private void rebuildDrivePathChooser(StartPosition startPosition)
    {
        var newChooser = new SendableChooser<AutoDriveOption>();
        var options    = startPosition.driveOptions;

        newChooser.setDefaultOption(options[0].displayName(), options[0]);

        for (int index = 1; index < options.length; index++)
        {
            var option = options[index];
            newChooser.addOption(option.displayName(), option);
        }

        _drivePathChooser  = newChooser;
        _lastStartPosition = startPosition;
        SmartDashboard.putData("Auto Drive Path", _drivePathChooser);
    }

    private void updateDashboardPreview()
    {
        var startPosition = getSelectedStartPosition();
        var autoMode      = getSelectedMode();
        var driveOption   = getSelectedDriveOption(startPosition);
        var startPose     = flip(startPosition.pose);
        var endPose       = flip(driveOption.endPoseBlue());

        _previewField.setRobotPose(startPose);
        _previewField.getObject("Auto End Pose").setPose(endPose);

        SmartDashboard.putString("Auto Summary", buildSummary(autoMode, startPosition, driveOption, getSelectedDelaySeconds()));
        SmartDashboard.putString("Auto Selected Trajectory", driveOption.staysPut() ? "None" : driveOption.trajectoryName());
        SmartDashboard.putBoolean("Auto Uses Trajectory", !driveOption.staysPut());
        SmartDashboard.putBoolean("Auto Selection Valid", true);
        _invalidAutoAlert.set(false);
    }

    private AutoMode getSelectedMode()
    {
        var selected = _modeChooser.getSelected();
        return selected != null ? selected : DEFAULT_AUTO_MODE;
    }

    private int getSelectedDelaySeconds()
    {
        var selected = _delayChooser.getSelected();
        return selected != null ? selected : DEFAULT_AUTO_DELAY_SECS;
    }

    private StartPosition getSelectedStartPosition()
    {
        var selected = _startChooser.getSelected();
        return selected != null ? selected : DEFAULT_START_POSITION;
    }

    private AutoDriveOption getSelectedDriveOption(StartPosition startPosition)
    {
        var selected = _drivePathChooser.getSelected();
        if (selected != null)
        {
            return selected;
        }

        return startPosition.driveOptions[0];
    }

    private String buildSummary(AutoMode mode, StartPosition startPosition, AutoDriveOption driveOption, int delaySeconds)
    {
        return switch (mode)
        {
            case DoNothing -> "Do Nothing | " + startPosition.displayName;
            case DriveOnly -> "Drive Only | " + startPosition.displayName + " | " + driveOption.displayName;
            case ShootOnly -> "Shoot Only | " + startPosition.displayName;
            case ShootWithDelay -> "Shoot Only | " + startPosition.displayName + " | Delay " + delaySeconds + "s";
            case ShootThenDrive -> "Shoot + Drive | " + startPosition.displayName + " | " + driveOption.displayName;
            case ShootWithDelayThenDrive -> "Shoot + Drive | " + startPosition.displayName + " | " + driveOption.displayName + " | Delay " + delaySeconds + "s";
        };
    }

    private void followTrajectory(SwerveSample sample)
    {
        var pose = _driveSubsystem.getState().Pose;

        _driveSubsystem.setControl(
                _autoRequest.withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x)).withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
                        .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
        );
    }

    private Pose2d flip(Pose2d bluePose)
    {
        if (Utilities.isRedAlliance())
        {
            return bluePose.rotateAround(new Translation2d(GeneralConstants.FIELD_SIZE_X.div(2), GeneralConstants.FIELD_SIZE_Y.div(2)), Rotation2d.k180deg);
        }

        return bluePose;
    }
}
