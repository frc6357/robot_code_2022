/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.utils.FilteredJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The Robot controllers
    private final FilteredJoystick Controller = new FilteredJoystick(Ports.joystickController);

    private SK22Transfer transferSubsystem = new SK22Transfer();

    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    public RobotContainer()
    {
        configureShuffleboard();

        transferSubsystem.setDefaultCommand(new DefaultTransferCommand(transferSubsystem));

        // Configure the button bindings
        configureButtonBindings();

    }

    private void configureShuffleboard()
    {

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        
    }
}
