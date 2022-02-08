package frc.robot.simulation;

public class ArmSim
{

    private final int baseLength;
    private final int maxExtendLength;
    private final int minAngle;
    private final int maxAngle;
    private double extendLength = 0;
    private double degreeLocation = 90;

    public ArmSim(int baseLength, int maxExtendLength, int minAngle, int maxAngle)
    {
        this.baseLength = baseLength;
        this.maxExtendLength = maxExtendLength;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public void setExtendLength(double length)
    {
        if (length < 0)
        {
            extendLength = 0;
            return;
        }
        if (length > maxExtendLength)
        {
            extendLength = maxExtendLength;
            return;
        }
        extendLength = length;
    }

    public void changeExtendLength(double delta)
    {
        setExtendLength(extendLength + delta);
    }

    public double getExtendLength()
    {
        return extendLength;
    }

    public int getBaseLength()
    {
        return baseLength;
    }

    public double getTotalLength()
    {
        return extendLength + baseLength;
    }

    public void changeAngle(double deltaDegrees)
    {
        setAngle(degreeLocation + deltaDegrees);
    }

    private void setAngle(double newDegreeLocation)
    {
        if (newDegreeLocation < minAngle)
        {
            degreeLocation = minAngle;
        }
        else if (newDegreeLocation > maxAngle)
        {
            degreeLocation = maxAngle;
        }
        else
        {
            degreeLocation = newDegreeLocation;
        }
    }

    public double getAngle()
    {
        return degreeLocation;
    }

}
