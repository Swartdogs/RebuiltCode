package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class SetIntake extends Command
{
    private final Intake      _intake;
    private final IntakeState _state;

    public SetIntake(Intake intake, IntakeState state)
    {
        _intake = intake;
        _state  = state;

        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        _intake.set(_state);
    }

    @Override
    public void end(boolean interrupted)
    {
        _intake.set(IntakeState.Off);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
