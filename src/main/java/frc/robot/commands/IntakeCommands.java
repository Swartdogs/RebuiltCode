package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public final class IntakeCommands
{
    private IntakeCommands()
    {
    }

    public static Command start(Intake intake)
    {
        return intake.runOnce(() -> intake.set(IntakeState.On));
    }

    public static Command stop(Intake intake)
    {
        return intake.runOnce(() -> intake.set(IntakeState.Off));
    }

    public static Command reverse(Intake intake)
    {
        return intake.runOnce(() -> intake.set(IntakeState.Reverse));
    }
}