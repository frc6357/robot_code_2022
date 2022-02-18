package frc.robot.subsystems.base;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.math.controller.PIDController;

/**
 * SK22ComplexClimbArm is meant to serve as the robot's front climb arm in which it will reach for the high rung
 * and the traversal rung as well.
 */
public class ComplexClimbArm
{
    private final CANSparkMax complexBrakePivot;
    private final CANSparkMax complexRatchetLift;


    private final DoubleSolenoid complexRatchetPiston =
            new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE, PneumaticsModuleType.REVPH,
                Ports.COMPLEX_CLIMB_RATCHET_PISTON_FORWARD_CHANNEL,
                Ports.COMPLEX_CLIMB_RATCHET_PISTON_REVERSE_CHANNEL);
    private final RelativeEncoder brakePivotEncoder;
    private final RelativeEncoder ratchetLiftEncoder;

    private final RotatingArm pivotArm;
    private final RotatingArm ratchetArm;
    private boolean ratchetState;

    /**
     * The constructor requires the two CANSparkMax motors for rotation of the arm and the extending vertically
     * of the arm.
     * @param complexBrakePivot The motor used for the rotation of the arm
     * @param complexRatchetLift The motor used for the extention of the arm vertically
     * @param brakePivotEncoder This encoder measures the angle of the pivot
     * @param ratchetLiftEncoder This encoder measures the length travel by the ratchet
     */
    public ComplexClimbArm(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift,
                           RelativeEncoder brakePivotEncoder, RelativeEncoder ratchetLiftEncoder)
    {   
        this.complexBrakePivot = complexBrakePivot;
        this.complexRatchetLift = complexRatchetLift;
        this.brakePivotEncoder = brakePivotEncoder;
        this.ratchetLiftEncoder = ratchetLiftEncoder;
        ratchetState = false;
        pivotArm = new RotatingArm(complexBrakePivot, brakePivotEncoder, 
                                   Constants.ClimbConstants.PIVOT_MOTOR_ROTATIONS_TO_DEGREE_CONVERTER, 
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KP, 
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KI,
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KD);

        ratchetArm = new RotatingArm(complexRatchetLift, ratchetLiftEncoder, 
                                   Constants.ClimbConstants.LIFT_MOTOR_ROTATIONS_TO_DEGREE_CONVERTER,
                                   Constants.ClimbConstants.LIFT_ARM_CONTROLLER_KP, 
                                   Constants.ClimbConstants.LIFT_ARM_CONTROLLER_KI,
                                   Constants.ClimbConstants.LIFT_ARM_CONTROLLER_KD);
    }
    /**
     * The turnRatchetPistonOn method will turn on the piston to extend the hand in order to grab a rung
     */
    public void turnRatchetPistonOn()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kForward);
        ratchetState = true;
    }

    /**
     * The turnRatchetPistonOn method will turn off the piston to stop the extension of the hand to prevent
     * over extending
     */
    public void turnRatchetPistonOff()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kReverse);
        ratchetState = false;
    }

    public double getPivotArmPosition()
    {
        return pivotArm.getAngle();
    }

    public double getRachetArmPosition()
    {
        return ratchetArm.getAngle();
    }

    /**
     * This method would set position of the pivot to a certain number of degrees
     * @param degrees The specific number of degrees we want to the pivot to move
     */
    public void setPivotArmPosition(double degrees)
    {
        pivotArm.goToAngle(degrees);
        pivotArm.enable();
        pivotArm.update();
        //TODO: Check to see how when the rotatingarm update method would stop
    }

    /**
     * This method would set the position of the lift ratchet to a specific length
     * @param distance The specific distance we want the lift to move up
     */
    public void setRatchetArmPosition(double distance)
    {
        if(ratchetState == true && distance < 0.0)
        {
            return;   
        }

        ratchetArm.goToAngle(distance);
        ratchetArm.enable();
        ratchetArm.update();
        //TODO: Check to see how when the roatingarm update moethod would stop
    }

    /**
     * Stops the pivot arm from turning
     */
    public void setPivotArmPositionOff()
    {
        pivotArm.disable();
    }

    /**
     * Stops the ratchet arm from turning
     */
    public void setRatchetArmPositionOff()
    {
        ratchetArm.disable();
    }
}