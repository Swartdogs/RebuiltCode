package frc.robot;

public class TestHook
{
    public void stop()
    {
    }

    public void setRate(double rate)
    {
    }

    public void forward()
    {
    }

    public void reverse()
    {
    }
}

public class MotorHook extends TestHook
{
    private int _polarity = 1;

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

