package frc.robot.utils;

import frc.robot.subsystems.base.SuperClasses.AutoCommands;

public interface SmartdashboardDisplay
{
    void run(String name, AutoCommands command);
}
