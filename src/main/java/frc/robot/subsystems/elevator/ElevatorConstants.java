package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ShooterConstants{
    public static final boolean IS_INVERTED = true;//Don't know what to use here inverted or not

    public static final Distance MAX_HEIGHT = inches(0.0); //TODO: set correct value
    public static final Distance MIN_HEIGHT = inches(0.0); //TODO: set correct
    public static final Distance GROUND_HEIGHT = inches(0.0); //TODO: set correct

    public static final int LEAD_MOTOR_ID = 1; //TODO: set correct
    public static final int FOLLOWER_MOTOR_ID = 1; //TODO: set correct

    public static final double JAMMED_CURRENT_THRESHOLD = 0.0; //TODO: set correct

    public static final String SUBSYSTEM_NAME = "Elevator";

    public static final boolean DEBUGGING = true;
    public static final boolean TESTING = true;

    public static final double KP_SLOT0 = 0.0;
    public static final double KI_SLOT0 = 0.0;
    public static final double KD_SLOT0 = 0.0;
    public static final double KS_SLOT0 = 0.0;
    public static final double KV_SLOT0 = 0.0;
    public static final double KA_SLOT0 = 0.0;
    public static final double KG_SLOT0 = 0.0;

    public static final double CRUISE_VELOCITY = 0.0;
    public static final double KV_EXPO = 0.0;
    public static final double KA_EXPO = 0.0;

    public static final double PULLEY_CIRCUMFERENCE_INCHES = 0.0;//TODO: set correct
    public static final int GEAR_RATIO = 0;//TODO: set correct

    //FIXME: Need to add voltages & their specific constants here

    public enum ScoringHeight {
        HARDSTOP,

        L1,
        ABOVE_L1,
        L2,
        L3,
        L4,

        MAX_L2,
        MAX_L3,

        LOW_ALGAE,
        HIGH_ALGAE,

        BELOW_LOW_ALGAE,
        BELOW_HIGH_ALGAE,

        BARGE,
        PROCESSOR
    }

    //TODO: Set correct scoring heights
    public static final Distance L1_HEIGHT = Inches.of(0.0);
    public static final Distance ABOVE_L1_HEIGHT = Inches.of(0.0);

    public static final Distance L2_HEIGHT = Inches.of(0.0);
    public static final Distance FAR_L2_HEIGHT = Inches.of(0.0);

    public static final Distance L3_HEIGHT = Inches.of(0.0);
    public static final Distance FAR_L3_HEIGHT = Inches.of(0.0);
    public static final Distance L4_HEIGHT = Inches.of(0.0);

    public static final Distance BARGE_HEIGHT = Inches.of(0.0);
    public static final Distance PROCESSOR_HEIGHT = Inches.of(0.0);

    public static final Double FAR_SCORING_DISTANCE = Units.inchesToMeters(0.0);
    public static final Double MIN_FAR_SCORING_DISTANCE = Units.inchesToMeters(0.0);

    public static final Double FAR_SCORING_Y_TOLERANCE = Units.inchesToMeters(0.0);
    public static final Rotation2d FAR_SCORING_THETA_TOLERANCE = Rotation2d.fromDegrees(0.0);

    public static final Distance BELOW_HIGH_ALGAE_HEIGHT = Inches.of(0.0);
    public static final Distance HIGH_ALGAE_HEIGHT = Inches.of(0.0);

    public static final Distance BELOW_LOW_ALGAE_HEIGHT = Inches.of(0.0);
    public static final Distance LOW_ALGAE_HEIGHT = Inches.of(0.0);

    //TODO: Set current limits
    public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 0.0;
}