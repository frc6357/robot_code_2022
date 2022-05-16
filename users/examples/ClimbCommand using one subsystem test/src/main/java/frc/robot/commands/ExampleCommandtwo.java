package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.ExampleSubsystem;
import frc.robot.subsystem.ExampleSubsystemTwo;

public class ExampleCommandTwo extends CommandBase {
    private final ExampleSubsystemTwo subsystem;

    private int count = 0;

    public ExampleCommandTwo(ExampleSubsystemTwo subsystem){
        this.subsystem = subsystem;

        addRequirements(this.subsystem);
        
    }

    @Override
    public void initialize()
    {
        count = 0;
        this.subsystem.doSomethingTwo();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        count++;
        if(count == 10){
            System.out.println("Two Finished ");
            return true;
        }
        else{
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("Two " + interrupted);
    }
}
