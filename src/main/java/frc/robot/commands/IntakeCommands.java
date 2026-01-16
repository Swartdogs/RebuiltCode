package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public static Command run(Intake intake)
    {
        return Commands.startEnd(
            () -> intake.set(IntakeState.On),
            () -> intake.set(IntakeState.Off),
            intake);
    }

    public static Command runReverse(Intake intake)
    {
        return Commands.startEnd(
            () -> intake.set(IntakeState.Reverse),
            () -> intake.set(IntakeState.Off),
            intake);
    }
}
