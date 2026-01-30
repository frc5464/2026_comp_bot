package frc.robot;

public class Universals {
    

    public static boolean zeroGyro = false;
    public static boolean brakeMode = false;
    public static double wait = 0;
    public static boolean zoomMode = false;

    public static double driveSpeedMultiplier = 1; // at max = 1
    public static double shootSpeedMultiplier = 1; // at max = 1
    
    // public static boolean slowMode = false;
    
    public static boolean shoot = false;
    public static boolean shootReverse = false;
    public static boolean disableShoot = false;
    public static boolean shootUptoSpeed = false;

    public static boolean intaking = false;
    public static boolean feeding = false;
    public static boolean intakeReverse = false;
    
    public static boolean climbUp = false;
    public static boolean climbDown = false;
    public static boolean autoClimbUp = false;
    public static boolean autoClimbDown = false;

    public static boolean climbOverride = false;

    public static String LEDselected = "White";

    public static boolean snapAprilHub = false;

    public static boolean closeToHub = false;

    public static boolean fuelHoming = false;
    public static boolean fuelDetected = false;

    public static boolean aprilTagsAreDetected = false;
    
    public static boolean dummyIntake = false;

    public static boolean homingPathToFuel = false;

    public static boolean translateToNote = true;

    private Universals(){

    }  
}
