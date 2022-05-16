package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommandtwo extends CommandBase {
    private final ExampleSubsystem = subsystem;

    public ExampleCommandtwo(ExampleSubsystem subsystem){
        this.subsystem = subsystem;
        
    }

    @Override
    public void initialize()
    {
        this.subsystem.doSomethingTwo();
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
