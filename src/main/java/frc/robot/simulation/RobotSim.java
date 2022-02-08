package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class RobotSim
{
    private final ArmSim armSim = new ArmSim(36, 24, 63, 90);

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d         mech2d       = new Mechanism2d(20, 70);
    private final MechanismRoot2d     baseRoot     = mech2d.getRoot("Arm Root", 0, 0);
    private final MechanismLigament2d baseMech2d   =
            baseRoot.append(new MechanismLigament2d("2 Base", armSim.getBaseLength(),
                armSim.getAngle(), 8, new Color8Bit(Color.kBlue)));
    private final MechanismRoot2d     extendRoot   =
            mech2d.getRoot("Arm Extend", 0, armSim.getBaseLength());
    private final MechanismLigament2d extendMech2d =
            extendRoot.append(new MechanismLigament2d("1 Extend", armSim.getTotalLength(),
                armSim.getAngle(), 8, new Color8Bit(Color.kRed)));

    private final CANSparkMax pivotMotor;
    private final CANSparkMax liftMotor;

    public RobotSim(CANSparkMax pivot, CANSparkMax lift)
    {
        this.pivotMotor = pivot;
        this.liftMotor = lift;
        SmartDashboard.putData("RobotPose", mech2d);
    }

    public void update()
    {
        updateExtend();
        updateArc();
    }

    public void updateArc()
    {
        double motorRPM = pivotMotor.get() * ClimbConstants.ARM_MOTOR_RPM;
        double motorRPS = motorRPM / Constants.SECONDS_PER_MINUTE;
        double motorRPT = motorRPS / Constants.SIM_TICS_PER_SECOND;
        double motorOutputPT = motorRPT / ClimbConstants.PIVOT_MOTOR_GEAR_RATIO;
        double motorTeethPT = motorOutputPT * ClimbConstants.PIVOT_MOTOR_TEETH_PER_REVOLUTION;
        double deltaRevolutions = motorTeethPT / ClimbConstants.PIVOT_ARC_TEETH_PER_REVOLUTION;
        double deltaDegrees = deltaRevolutions * Constants.DEGREES_PER_REVOLUTION;
        armSim.changeAngle(deltaDegrees);
        baseMech2d.setAngle(armSim.getAngle());
        double diffAngle = (90 - armSim.getAngle()) / 180 * Math.PI;
        baseRoot.setPosition(20, 20);
        extendRoot.setPosition(Math.sin(diffAngle) * armSim.getBaseLength(),
            Math.cos(diffAngle) * armSim.getBaseLength());
        // System.err.println(diffAngle + " " + Math.sin(diffAngle) * armSim.getBaseLength() + " "
        //     + Math.cos(diffAngle) * armSim.getBaseLength());
        extendMech2d.setAngle(armSim.getAngle());
    }

    public void updateExtend()
    {
        double motorRPM = liftMotor.get() * ClimbConstants.ARM_MOTOR_RPM;
        double motorRPS = motorRPM / Constants.SECONDS_PER_MINUTE;
        double motorRPT = motorRPS / Constants.SIM_TICS_PER_SECOND;
        double motorOutputPT = motorRPT / ClimbConstants.LIFT_MOTOR_GEAR_RATIO;
        double motorTeethPT = motorOutputPT * ClimbConstants.LIFT_MOTOR_TEETH_PER_REVOLUTION;
        double deltaInches = motorTeethPT / ClimbConstants.LIFT_RACK_TEETH_PER_INCH;
        armSim.changeExtendLength(deltaInches);
        extendMech2d.setLength(armSim.getTotalLength());
    }
}
