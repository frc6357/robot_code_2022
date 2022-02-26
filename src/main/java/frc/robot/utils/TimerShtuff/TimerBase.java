package frc.robot.utils.TimerShtuff;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.subsystems.SK22Transfer;

public interface TimerBase {
    public final TimerType timerType = TimerType.DEFAULT;
    public int timerDuration = 0;
    public int timerProgress = 0;

    public TimerType getTimerType();

    public int getTimerDuration();

    public int getTimerProgress();

    public void enableMotor(WPI_TalonFX motor, double speed);

    public void disableMotor(WPI_TalonFX motor);

    public TimerStatus periodic(SK22Transfer transfer);
}
