//
// SK22Gearshift: Drivetrain subsystem for the Team 6357 2022 robot
//
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.base.SuperClasses.Gear;

/**
 * The SK22Gearshift class is the subsystem that controls the drive train of the Robot.
 */
public class SK22Gearshift extends SKSubsystemBase implements AutoCloseable
{
    private final DoubleSolenoid gearShiftSolenoid;
    private Gear currentGear;

    /**
     * Creates a SK22Gearshift which accepts a double solenoid in order to gear shift
     * 
     * @param gearShiftSolenoid
     *            This is the solenoid that controls the drivetrain gear selector.
     */
    public SK22Gearshift(DoubleSolenoid gearShiftSolenoid)
    {
        this.gearShiftSolenoid = gearShiftSolenoid;

        currentGear = Gear.HIGH;
        setGear(currentGear);
    }

    @Override
    public void initializeTestMode()
    {

    }

    /**
     * Sets the gear for the drivetrain.
     * 
     * @param newGear
     *            The gear value that we want the robot to set.
     */
    public void setGear(Gear newGear)
    {
        currentGear = newGear;

        DoubleSolenoid.Value check = (currentGear == Gear.HIGH) ? DoubleSolenoid.Value.kForward
            : DoubleSolenoid.Value.kReverse;
        gearShiftSolenoid.set(check);
        
        SmartDashboard.putString("Drive Gear", (currentGear == Gear.HIGH) ? "HIGH" : "LOW");
    }

    /**
     * Query the current drivetrain gear setting.
     * 
     * @return Returns the currently-selected gear, HIGH or LOW.
     */
    public Gear getGear()
    {
        return currentGear;
    }

    @Override
    public void testModePeriodic()
    {

    }

    @Override
    public void enterTestMode()
    {


    }

    // The close() method is required to allow use with jUnit unit tests (see src/test/java).
    // To use this, the class must implement the AutoCloseable interface.
    // This is called to clean up between each test and must close all downstream objects
    // before returning.
    @Override
    public void close() throws Exception
    {
        gearShiftSolenoid.set(DoubleSolenoid.Value.kOff);
    }
}
