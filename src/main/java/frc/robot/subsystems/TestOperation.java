package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TestHook;
import java.util.HashMap;
import java.util.Map;

public class TestOperation extends SubsystemBase
{
    private Map<String, TestHook> _hooks    = new HashMap<>();
    private TestHook[]            _slots    = { null, null, null, null, null, null, null, null, };
    private TestHook              _current  = null;
    private boolean               _active   = false;
    private double                _rate     = 0.0;
    private int                   _shiftMod = 0;

    public Command cmd_shift()
    {
        return startEnd(() -> _shiftMod = 4, () -> _shiftMod = 0);
    }

    public Command cmd_button_04()
    {
        return runOnce(() -> do_select(0));
    }

    public Command cmd_button_15()
    {
        return runOnce(() -> do_select(1));
    }

    public Command cmd_button_26()
    {
        return runOnce(() -> do_select(2));
    }

    public Command cmd_button_37()
    {
        return runOnce(() -> do_select(3));
    }

    public Command cmd_forward()
    {
        return startEnd(() -> do_forward(), () -> do_stop());
    }

    public Command cmd_reverse()
    {
        return startEnd(() -> do_reverse(), () -> do_stop());
    }

    public Command cmd_increase()
    {
        return runOnce(() -> do_increase());
    }

    public Command cmd_decrease()
    {
        return runOnce(() -> do_decrease());
    }

    public Command cmd_reset()
    {
        return runOnce(() -> do_reset());
    }

    public Command cmd_full()
    {
        return runOnce(() -> do_full());
    }

    public Command cmd_jog()
    {
        return runOnce(() -> do_jog());
    }

    public TestOperation()
    {
        _rate     = 0.1;
        _shiftMod = 0;
    }

    public void add(String name, TestHook hook)
    {
        if (name == null || hook == null) return;
        _hooks.put(name, hook);
    }

    public void connect(int slot, String name)
    {
        if (slot < 0 || slot >= _slots.length) return;
        _slots[slot] = _hooks.get(name);
    }

    public void connect(int slot, String name, String shift_name)
    {
        if (slot < 0 || slot + 4 >= _slots.length) return;
        _slots[slot]     = _hooks.get(name);
        _slots[slot + 4] = _hooks.get(shift_name);
    }

    @Override
    public void periodic()
    {
        if (_active && (null != _current))
        {
            _current.setRate(_rate);
        }
    }

    public void do_stop()
    {
        _active = false;
        _hooks.forEach((key, hook) ->
        {
            hook.stop();
        });
    }

    public void do_forward()
    {
        if (_current != null)
        {
            _current.forward();
            _current.setRate(_rate);
            _active = true;
        }
    }

    public void do_reverse()
    {
        if (_current != null)
        {
            _current.reverse();
            _current.setRate(_rate);
            _active = true;
        }
    }

    public void do_jog()
    {
        if (_current != null)
        {
            _current.forward();
            _current.setRate(_rate);
            _current.reverse();
            _current.setRate(_rate);
            _current.stop();
        }
    }

    public void do_reset()
    {
        _rate = 0.1;
    }

    public void do_full()
    {
        _rate = 1.0;
    }

    public void do_increase()
    {
        _rate = Math.min(_rate + 0.1, 1.);
    }

    public void do_decrease()
    {
        _rate = Math.max(_rate - 0.1, 0.1);
    }

    public void do_select(int slot)
    {
        if (slot < 0 || slot + _shiftMod >= _slots.length) return;
        if (_current != null)
        {
            _current.stop();
        }
        _current = _slots[slot + _shiftMod];
        _active  = false;
    }
}
