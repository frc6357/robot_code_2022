package frc.robot.utils.TimerShtuff;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.subsystems.SK22Transfer;

// TODO: This is a general purpose interface so it MUST be documented so that other people can use it.

/**
 * Document this!
 */
public interface TimerBase
{
    /**
    * Document this!
    */
    public final TimerType TIMERTYPE = TimerType.DEFAULT;

   /**
    * Document this!
    */
    public int TIMERDURATION = 0;

   /**
    * Document this!
    */
    public int TIMERPROGRESS = 0;

   /**
    * Document this!
    * @return The type of timer?
    */
    public TimerType getTimerType();

   /**
    * Document this!
    * @return The timer duration in milliseconds?
    */
    public int getTimerDuration();

   /**
    * Document this!
    * @return The current timer count in milliseconds?
    */
    public int getTimerProgress();

   /**
    * Document this!
    * @param motor The motor to control based on the timer?
    * @param speed The motor speed in the range [-1.0, 1.0]?
    */
    public void enableMotor(WPI_TalonFX motor, double speed);

   /**
    * Document this!
    * @param motor The motor to disable?
    */
    public void disableMotor(WPI_TalonFX motor);

   /**
    * Document this!
    * @param transfer The transfer subsystem for which the timer is being used.
    * @return TimerStatus
    */
    public TimerStatus periodic(SK22Transfer transfer);
}
