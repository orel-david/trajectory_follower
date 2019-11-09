package robot;

/**
 * A class holding all of the ports on the robot.
 * Place mechanism-specific ports inside their own sub-class.
 * When accessing a mechanism-specific port, call Ports.[MECHANISM].[PORT_NAME]
 */
public class Ports {
    public static class Drivetrain {
     public static final int leftMaster = 3;
     public static final int leftSlave1 = 4;
     public static final int leftSlave2 = 5;
     public static final int rightMaster = 6;
     public static final int rightSlave1 = 7;
     public static final int rightSlave2 = 8;
    }
}
