package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.base.RotatingArm;

public class moveBase extends CommandBase
{


    private final RotatingArm arm;

    public moveBase( RotatingArm arm)
    {

        this.arm = arm;
    }

    @Override
    public void initialize()
    {
        this.arm.goToAngle(90);
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
