package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.ExampleSubsystem;
import frc.robot.subsystem.ExampleSubsystemTwo;

public class ExampleCommandSequence extends ParallelCommandGroup{
    
    private ExampleCommandOne commandOne;
    private ExampleCommandTwo commandTwo;

    public ExampleCommandSequence(ExampleSubsystem subsystem, ExampleSubsystemTwo subsystemTwo){
        commandOne = new ExampleCommandOne(subsystem);
        commandTwo = new ExampleCommandTwo(subsystemTwo);

        addCommands(commandOne,commandTwo);
    }

}
