package frc.robot.subsystems.base.SuperClasses;

/**
 * Available autonomous commands for the Robot.
 */
public enum AutoCommands
{
    // Simple Paths
    DoNothing, DriveSplineFromJSON, DriveSplineCanned, Drive1mForwardBackward, 

    // Basic Paths
    Taxi, N2_HH_R,

    // 2 Ball Autos
        // Tarmac Specific 2 Low Goal Autos
        N2_LL_1A, N2_LL_2A, N2_LL_2B,
        
        // Tarmac Specific 1 Low Goal 1 High Goal
        N2_LH_1A, N2_LH_2A, N2_LH_2B,

    // 3 Ball Autos
        // Tarmac Specific 1 Low Goal 2 High Goal
        N3_LHH_1A, N3_LHH_2A, N3_LHH_2B,

        // Tarmac Specific All High Goals
        N3_HHH_R1A, N3_HHH_R2A, N3_HHH_R2B,

        // Tarmac Specific 3 Ball Autos (with terminal ball)
            // 1 Low Goal 2 High Goals
            T3_LHH_1A, T3_LHH_2A, T3_LHH_2B,
            // All High Goals
            T3_HHH_R1A, T3_HHH_R2A, T3_HHH_R2B,
        
    // Tarmac Specific 4 Ball Autos (with terminal ball)
        // 1 Low Goal 3 High Goals
        T4_LHHH_1A, T4_LHHH_2B,
        // All High Goals
        T4_HHHH_R1A, T4_HHHH_R2B
};