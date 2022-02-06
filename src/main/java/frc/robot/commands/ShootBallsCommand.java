package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Vision;

// TODO: Write this command

/** A class that shoots the balls that we possess using the launcher and vision system */
public class ShootBallsCommand extends CommandBase
{
    private final SK22Launcher launcher;
    private final SK22Vision vision;

    public ShootBallsCommand(SK22Launcher launcher, SK22Vision vision)
    {
        this.launcher = launcher;
        this.vision = vision;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
