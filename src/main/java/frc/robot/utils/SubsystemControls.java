package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This is made for the 2022 season
 */
public class SubsystemControls
{

    private final boolean intake;
    private final boolean launcher;
    private final boolean transfer;
    private final boolean climb;
    private final boolean vision;
    private final boolean gearshift;

    /**
     * Constructs a new SubsystemControls object with the given subsystem presence.
     * 
     * @param intake
     *            indicates if the intake system is present and should be enabled
     * @param launcher
     *            indicates if the launcher system is present and should be enabled
     * @param transfer
     *            indictes if the indexer system is present and should be enabled
     * @param climb
     *            indicates if the climb system is present and should be enabled
     * @param vision
     *           indicates if the vision system is present and should be enabled
     * @param gearshift
     *           indicates if the gearshift system is present and should be enabled
     */
    public SubsystemControls(@JsonProperty(required = true, value = "intake")
                                boolean intake,
                            @JsonProperty(required = true, value = "transfer")
                                boolean transfer,
                            @JsonProperty(required = true, value = "launcher")
                                boolean launcher,
                            @JsonProperty(required = true, value = "climb")
                                boolean climb,
                            @JsonProperty(required = true, value = "vision")
                                boolean vision,
                            @JsonProperty(required = true, value = "gearshift")
                                boolean gearshift)
    {
        this.intake = intake;
        this.launcher = launcher;
        this.transfer = transfer;
        this.climb = climb;
        this.vision = vision;
        this.gearshift = gearshift;
    }

    /**
     * Returns true if the intake system is indicated as present and should be enabled.
     * 
     * @return true if the intake system is indicated as present and should be enabled;
     *         false otherwise
     */
    public boolean isIntakePresent()
    {
        return intake;
    }

    /**
     * Returns true if the launcher system is indicated as present and should be enabled.
     * 
     * @return true if the launcher system is indicated as present and should be enabled;
     *         false otherwise
     */
    public boolean isLauncherPresent()
    {
        return launcher;
    }

    /**
     * Returns true if the transfer system is indicated as present and should be enabled.
     * 
     * @return true if the transfer system is indicated as present and should be enabled;
     *         false otherwise
     */
    public boolean isTransferPresent()
    {
        return transfer;
    }

    /**
     * Returns true if the climb system is indicated as present and should be enabled.
     * 
     * @return true if the climb system is indicated as present and should be enabled;
     *         false otherwise
     */
        public boolean isClimbPresent()
    {
        return climb;
    }

    /**
     * Returns true if the climb system is indicated as present and should be enabled.
     * 
     * @return true if the climb system is indicated as present and should be enabled;
     *         false otherwise
     */
        public boolean isVisionPresent()
    {
        return vision;
    }

    /**
     * Returns true if the drivetrain geatshift system is indicated as present and should be enabled.
     * 
     * @return true if the gearshift system is indicated as present and should be enabled;
     *         false otherwise
     */
    public boolean isGearshiftPresent()
    {
        return gearshift;
    }
}
