package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelay extends CommandBase
{

    private long       startTime;
    private final long delayTime;

    public TimeDelay(long delayTimeMillis)
    {
        delayTime = delayTimeMillis;
    }

    @Override
    public void initialize()
    {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return System.currentTimeMillis() - startTime >= delayTime;
    }
}
