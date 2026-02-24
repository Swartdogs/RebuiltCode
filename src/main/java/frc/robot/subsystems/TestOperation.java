package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import java.util.Map ;

public class TestOperation extends SubsystemBase 
{
    // array of testhook
    // current testhook
    // fixrate
    private Map<String, TestHook> _hooks ;
    private TestHook[]            _slots = { null, null, null, null, null, null, null, null, } ;

    private TestHook    _current = null ;
    private final boolean _active = false ;
    private final double _rate = 0.0 ;
    private final int _shiftMod = 0 ;

    public Command cmd_shift() { return startEnd(() -> _shiftMod = 4, () -> _shiftMod = 0; }
    
    public  Command cmd_button_04() { return runOnce( do_select( 0 ) ) ; }
    public  Command cmd_button_15() { return runOnce( do_select( 1 ) ) ; }
    public  Command cmd_button_26() { return runOnce( do_select( 2 ) ) ; }
    public  Command cmd_button_37() { return runOnce( do_select( 3 ) ) ; }

    public TestOperation() 
    {
        _rate = 0.1 ;
        _shiftMod = 0;
    }

    public void add( String name, TestHook hook )
    {
      _hooks.put( name, hook ) ;
    }

    public void connect( int slot, String name )
    {
      _slots[slot] = _hooks.get(name) ;
    }
    public void connect( int slot, String name, String shift_name )
    {
      _slots[slot] = _hooks.get(name) ;
      _slots[slot + 4] = hooks.get(shift_name) ;
    }

    @Override
    public void periodic()
    {
        if (_active && ( null != _current )) {
          _current-> setRate( _rate ) ;
    }

    public void do_stop() 
    {
        _active = false ;
        _hooks.forEach( key, hook ) { hook-> stop() ; }
    }

    public void do_forward()
    {
        if ( _current ) {
          _current-> forward() ;
          _current-> setRate( _rate ) ;
        }
    }
    public void do_reverse()
    {
        if ( _current ) {
          _current-> reverse() ;
          _current-> setRate( _rate ) ;
        }
    }
    public void do_jog()
    {
        if ( _current ) {
          _current-> forward() ;
          _current-> setRate( _rate ) ;
          _current-> reverse() ;
          _current-> setRate( _rate ) ;
          _current-> stop() ;
        }
    }

    public void do_reset()    { _rate= 0.1 ; }
    public void do_full()     { _rate= 1.0 ; }
    public void do_increase() { _rate= Math.min( _rate + 0.1, 1. ) ; }
    public void do_decrease() { _rate= Math.max( _rate - 0.1, 0.1 ) ; }

    public void do_select( int slot )
    {
        if (_current) { _current-> stop() ; }
        _current = _slots[ slot + _shiftMod ] ;
        do_jog() ;
    }
}
