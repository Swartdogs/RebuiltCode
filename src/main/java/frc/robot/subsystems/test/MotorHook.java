package frc.robot.subsystems.test;

public class MotorHook extends TestHook
{
    protected int _polarity = 1;

    @Override
    public void forward()
    {
        _polarity = 1;
    }

    @Override
    public void reverse()
    {
        _polarity = -1;
    }
}
