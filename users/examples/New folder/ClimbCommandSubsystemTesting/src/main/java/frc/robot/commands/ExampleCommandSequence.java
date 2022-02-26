package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.ExampleSubsystem;

public class ExampleCommandSequence {
    private ExampleCommandSequence(){
        
    }

    public static Command step1(ExampleSubsystem subsystem){
        ParallelCommandGroup step1 = new ParallelCommandGroup();
        step1.addCommands(new ExampleCommandOne(subsystem));
        step1.addCommands(new ExampleCommandtwo(subsystem));
        return step1;
    }

    public static Command step2(ExampleSubsystem subsystem){
        return new ExampleCommandOne(subsystem);
    }
}
