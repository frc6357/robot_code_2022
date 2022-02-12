/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// TODO: Commented out after moving SK22Climb instantiation to RobotContainer.
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.REVPhysicsSim;
//import edu.wpi.first.math.system.plant.DCMotor;
//import frc.robot.simulation.RobotSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TestModeManager;

/**
 * The VM is configured to automatically run this class, and to call the functions
 * corresponding to each mode, as described in the TimedRobot documentation. If you change
 * the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    //private RobotSim robotSimulation;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot()
    {
        // TODO: May want to build or load trajectories here?
    }

    @Override
    public void robotInit()
    {
        // TODO: NO! We can't be making Climb a special case here. Why isn't it
        // instantiated with all the other subsystems?
        //robotContainer = new RobotContainer(climbSubsystem);
        //robotSimulation = new RobotSim(complexBrakePivot, complexRatchetLift);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        /*
         * Runs the Scheduler. This is responsible for polling buttons, adding
         * newly-scheduled commands, running already-scheduled commands, removing finished
         * or interrupted commands, and running subsystem periodic() methods. This must be
         * called from the robot's periodic block in order for anything in the
         * Command-based framework to work.
         */
        CommandScheduler.getInstance().run();
        // boolean IBMToggle = SmartDashboard.getBoolean("Ball Mangement Toggle", true);
        // RobotContainer.toggleBallManagement.setPressed(IBMToggle);
    }

    /**
     * This function is called at the beginning of the Autonomous Phase. At the start of
     * the Autonomous mode initiates the AutonomousCommand
     */
    @Override
    public void autonomousInit()
    {
        robotContainer.resetDriveSubsystem();
        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command (example)
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        robotContainer.resetDriveSubsystem();
        robotContainer.resetDriveDefaultCommand();

        /*
         * This makes sure that the autonomous stops running when teleop starts running.
         * If you want the autonomous to continue until interrupted by another command,
         * remove this line or comment it out.
         */
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {

    }

    @Override
    public void testInit()
    {
        TestModeManager.initializeTestMode();
        robotContainer.resetDriveSubsystem();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
        TestModeManager.testModePeriodic();
    }


    
    @Override
    public void simulationInit()
    {
        // TODO: Commented out for now after moving SK22Climb instantiation into RobotContainer
        // with the other subsystems.

        //super.simulationInit();
        //REVPhysicsSim.getInstance().addSparkMax(complexBrakePivot, DCMotor.getNEO(1));
        //REVPhysicsSim.getInstance().addSparkMax(complexRatchetLift, DCMotor.getNEO(1));
    }

    /**
     * This function is called periodically during simulation.
     */
    @Override
    public void simulationPeriodic()
    {
        // TODO: Commented out for now after moving SK22Climb instantiation into RobotContainer
        // with the other subsystems.   
        //complexBrakePivot.set(-.025);
        //complexRatchetLift.set(.25);
        //REVPhysicsSim.getInstance().run();
        //robotSimulation.update();
    }
}
