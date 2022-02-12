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
    private final RelativeEncoder climbEncoder;

    private final RotatingArm pivotArm;

    /**
     * The constructor requires the two CANSparkMax motors for rotation of the arm and the extending vertically
     * of the arm.
     * @param complexBrakePivot The motor used for the rotation of the arm
     * @param complexRatchetLift The motor used for the extention of the arm vertically
     */
    public ComplexClimbArm(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift, RelativeEncoder climbEncoder)
    {
        this.complexBrakePivot = complexBrakePivot;
        this.complexRatchetLift = complexRatchetLift;
        this.climbEncoder = climbEncoder;
        pivotArm = new RotatingArm(complexBrakePivot, climbEncoder, 
                                   Constants.ClimbConstants.PIVOT_MOTOR_ROTATIONS_TO_DEGREE_CONVERTER, 
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KP, 
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KI,
                                   Constants.ClimbConstants.PIVOT_ARM_CONTROLLER_KD);
    }

    /**
     * The turnBrakePivotOn method will turn on the motor that will allow the front hand
     * to rotate in order to grab onto the high and traversal rung
     * @param speed The speed will change how fast the motor rotates to a specific angle
     */
    public void turnBrakePivotOn(double speed)
    {
        complexBrakePivot.set(speed);

    }

    /**
     * The turnBrakePivotOff method will turn off the motor that's controlling the
     * rotation of the arm so that we don't over-rotate
     */
    public void turnBrakePivotOff()
    {
        complexBrakePivot.stopMotor();
    }

    /**
     * The turnComplexRatchetLiftOn method will turn on the motor that is used to extend the arm via gears
     * @param speed The speed will determine how fast the motor will extend the arm
     */
    public void turnComplexRatchetLiftOn(int speed)
    {
        complexRatchetLift.set(speed);
    }

    /**
     * The turnComplexRatchetLiftOff method will turn off the motor so that the arm will stop after a certain
     * point once extended
     */
    public void turnComplexRatchetLiftOff()
    {
        complexRatchetLift.stopMotor();
    }

    /**
     * The turnRatchetPistonOn method will turn on the piston to extend the hand in order to grab a rung
     */
    public void turnRatchetPistonOn()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * The turnRatchetPistonOn method will turn off the piston to stop the extension of the hand to prevent
     * over extending
     */
    public void turnRatchetPistonOff()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public double getPivotArmPosition()
    {
        return pivotArm.getAngle();
    }

    public void setPivotArmPosition(double degrees){
        pivotArm.goToAngle(degrees);
        pivotArm.enable();
    }
}
