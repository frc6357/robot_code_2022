package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class RobotSim {
    private final ArmSim armSim = new ArmSim(36, 24, 63, 90);

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Arm Root", 10, 0);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(
            new MechanismLigament2d(
                    "Arm", armSim.getExtendLength() + armSim.getBaseLength(), 90));

    private final CANSparkMax pivotMotor;
    private final CANSparkMax liftMotor;

    public RobotSim(CANSparkMax pivot, CANSparkMax lift) {
        this.pivotMotor = pivot;
        this.liftMotor = lift;
    }

    public void update() {
        updateArc();
        updateExtend();
    }

    public void updateArc() {
        double motorRPM = pivotMotor.get() * ClimbConstants.ARM_MOTOR_RPM;
        double motorRPS = motorRPM / Constants.SECONDS_PER_MINUTE;
        double motorRPT = motorRPS / Constants.SIM_TICS_PER_SECOND;
        double motorOutputPT = motorRPT / ClimbConstants.PIVOT_MOTOR_GEAR_RATIO;
        double motorTeethPT = motorOutputPT * ClimbConstants.PIVOT_MOTOR_TEETH_PER_REVOLUTION;
        double deltaRevolutions = motorTeethPT / ClimbConstants.PIVOT_ARC_TEETH_PER_REVOLUTION;
        double deltaDegrees = deltaRevolutions / Constants.DEGREES_PER_REVOLUTION;
        armSim.changeAngle(deltaDegrees);
        armMech2d.setAngle(armSim.getAngle());
    }

    public void updateExtend() {
        double motorRPM = liftMotor.get() * ClimbConstants.ARM_MOTOR_RPM;
        double motorRPS = motorRPM / Constants.SECONDS_PER_MINUTE;
        double motorRPT = motorRPS / Constants.SIM_TICS_PER_SECOND;
        double motorOutputPT = motorRPT / ClimbConstants.LIFT_MOTOR_GEAR_RATIO;
        double motorTeethPT = motorOutputPT * ClimbConstants.LIFT_MOTOR_TEETH_PER_REVOLUTION;
        double deltaInches = motorTeethPT / ClimbConstants.LIFT_RACK_TEETH_PER_INCH;
        armSim.changeExtendLength(deltaInches);
        armMech2d.setLength(armSim.getExtendLength());
    }
}