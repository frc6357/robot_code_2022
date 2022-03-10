package frc.robot.utils.TimerShtuff;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.SK22Transfer;

public class VerticalTimer implements TimerBase
{
    private final TimerType timerType = TimerType.VERTICAL;
    private final int timerDuration;
    private int timerProgress = 0;

    private CANSparkMax[] motors;
    private double[] speeds;

    // TODO: What units is "speeds"? RPM, [-1,1]? Looking later, you use it with motor.setVoltage which seems very odd!
    public VerticalTimer(int timerDuration, CANSparkMax[] motors, double[] speeds)
    {
        this.timerDuration = timerDuration;
        this.motors = motors;
        this.speeds = speeds;
    }

    @Override
    public TimerType getTimerType()
    {
        return timerType;
    }

    @Override
    public int getTimerDuration()
    {
        return timerDuration;
    }

    @Override
    public int getTimerProgress()
    {
        return timerProgress;
    }

    @Override
    public void enableMotor(WPI_TalonFX motor, double speed)
    {
        motor.setVoltage(speed);
    }

    @Override
    public void disableMotor(WPI_TalonFX motor)
    {
        motor.setVoltage(0.0);
    }

    @Override
    public TimerStatus periodic(SK22Transfer transfer)
    {
        if (timerProgress == 0)
        {
            int index = 0;
            for (CANSparkMax motor : motors)
            {
                motor.setVoltage(speeds[index]);
                index++;
            }

            // transfer.setTimerState(true);
        }
        else if (timerProgress < timerDuration)
        {
            timerProgress++;
        }
        else if (timerProgress >= timerDuration)
        {
            timerProgress = 0;

            // transfer.setTimerState(false);

            return TimerStatus.FINISHED;
        }
        else if (!transfer.getIsRunningTimerEnabled() && timerProgress > 0)
        {
            timerProgress = 0;
            for (CANSparkMax motor : motors)
            {
                motor.setVoltage(0.0);
            }

            // transfer.setTimerState(false);

            return TimerStatus.FINISHED;
        }

        timerProgress++;
        return TimerStatus.IN_PROGRESS;
    }
    
}
