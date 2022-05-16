package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.ExampleSubsystem;

public class ExampleCommandOne extends CommandBase {
    private final ExampleSubsystem subsystem;

    private int count = 0;

    public ExampleCommandOne(ExampleSubsystem subsystem){
        this.subsystem = subsystem;

        addRequirements(subsystem);
        
    }

    @Override
    public void initialize()
    {
        count = 0;
        this.subsystem.doSomethingOne();
        
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        count++;
        if(count == 6){
            System.out.println("One Finished ");
            return true;
        }

        else{
            return false;
        }

    }

    @Override
    public void end(boolean interrupted){
        System.out.println("One " + interrupted);
    }


}
