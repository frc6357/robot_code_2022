package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExampleCommandOne extends CommandBase {
    private final ExampleSubsystem subsystem;

    public ExampleCommandOne(ExampleSubsystem subsystem){
        this.subsystem = subsystem;
        
    }

    @Override
    public void initialize()
    {
        this.subsystem.doSomethingOne();
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
