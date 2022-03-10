package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A simple command that waits for a given number of milliseconds
 * to elapse before finishiing.
 */
public class TimeDelayCommand extends CommandBase
{

    private long       startTime;
    private final long delayTime;

    /**
     * Constructs a TimeDelayCommand that will delay for a given number
     * of milliseconds.
     * 
     * @param delayTimeMillis Number of milliseconds to delay before exiting.
     */
    public TimeDelayCommand(long delayTimeMillis)
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
