package frc.robot.util;

import java.util.List;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants;

/**
 * Utility class for reading REBUILT 2026 game state from FMS. Provides Hub
 * Active status and alliance-specific tag IDs.
 */
public final class GameState
{
    private static BooleanSubscriber _hubActiveSubscriber;

    private GameState()
    {
    }

    static
    {
        var table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        _hubActiveSubscriber = table.getBooleanTopic("HubActive").subscribe(true);
    }
    
    public static boolean isHubActive()
    {
        if (DriverStation.isAutonomous())
        {
            return true;
        }

        return _hubActiveSubscriber.get();
    }

    /**
     * Check if we are on the Blue alliance.
     */
    public static boolean isBlueAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isEmpty() || alliance.get() == Alliance.Blue;
    }

    /**
     * Check if we are on the Red alliance.
     */
    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * Get the HUB AprilTag IDs for our alliance. Use these IDs to filter Limelight
     * detections for shooting. We shoot at OUR OWN hub (not opponent's).
     */
    public static List<Double> getOurHubTagIds()
    {
        if (isRedAlliance())
        {
            return Constants.Shooter.RED_HUB_TAG_IDS;
        }
        return Constants.Shooter.BLUE_HUB_TAG_IDS;
    }
}
