package robot.subsystems.drivetrain;

import static robot.Constants.Drivetrain.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Constants;
import robot.Robot;

/**
 * This is a temporary subsystem from last year.
 */
public class Drivetrain extends Subsystem {

    public TalonSRX leftMaster = new TalonSRX(Ports.Drivetrain.leftMaster);
    public TalonSRX rightMaster = new TalonSRX(Ports.Drivetrain.rightMaster);
    public VictorSPX right1 = new VictorSPX(Ports.Drivetrain.rightSlave1);
    public VictorSPX left1 = new VictorSPX(Ports.Drivetrain.leftSlave1);
    public VictorSPX right2 = new VictorSPX(Ports.Drivetrain.rightSlave2);
    public VictorSPX left2 = new VictorSPX(Ports.Drivetrain.leftSlave2);

    public Drivetrain() {
        leftMaster.setInverted(true);
        left1.setInverted(true);
        left2.setInverted(true);
        rightMaster.setInverted(false);
        right1.setInverted(false);
        right2.setInverted(false);

        right1.follow(rightMaster);
        right2.follow(rightMaster);
        left1.follow(leftMaster);
        left2.follow(leftMaster);

        leftMaster.configPeakCurrentLimit(MAX_CURRENT);
        rightMaster.configPeakCurrentLimit(MAX_CURRENT);
    }

    public void setLeftSpeed(double speed) {
        leftMaster.set(ControlMode.PercentOutput, speed);
    }

    public void setRightSpeed(double speed) {
        rightMaster.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftVelocity(double velocity) {
        leftMaster.set(ControlMode.Velocity, velocity);
    }

    public void setRightVelocity(double velocity) {
        rightMaster.set(ControlMode.Velocity, velocity);
    }

    public double getLeftDistance() {
        return convertTicksToDistance(leftMaster.getSelectedSensorPosition());
    }

    public double getRightDistance() {
        return convertTicksToDistance(rightMaster.getSelectedSensorPosition());
    }

    public double getRightVelocity() {
        return convertTicksToDistance(rightMaster.getSelectedSensorVelocity()) * 10;
    }

    public double getLeftVelocity() {
        return convertTicksToDistance(leftMaster.getSelectedSensorVelocity()) * 10;
    }

    /**
     *
     * @return angle in degrees
     */
    public double getAngle(){
        return Robot.navx.getAngle();
    }

    /**
     *
     * @return return angular velocity in rad/sec
     */
    public double getAngularVelocity(){
        return Math.toRadians(Robot.navx.getRate());
    }

    public int convertDistanceToTicks(double distance) {
        return (int) (distance * TICKS_PER_METER);
    }

    /**
     * because the max input from the joystick is 1 , the joystick input * max velocity is
     * function which represent the relation
     *
     * @param joystickInput the y value from the joystick
     * @return joystick value in m/s
     */
    public double convertJoystickInputToVelocity(double joystickInput) {
        return joystickInput * MAX_VEL;
    }


    /**
     * limit the drivetrain's right side acceleration to a certain acceleration
     *
     * @param desiredVelocity the desired velocity
     * @return the desired velocity if possible, if not the current velocity plus the max acceleration
     */
    public double limitRightAcceleration(double desiredVelocity) {

        //Take the attempted acceleration and see if it is too high.
        if (Math.abs(desiredVelocity - getRightVelocity()) / TIME_STEP >= MAX_ACCELERATION) {
            return getRightVelocity() + MAX_ACCELERATION;
        }

        return desiredVelocity;
    }

    /**
     * limit the drivetrain's left side acceleration to a certain acceleration
     *
     * @param desiredVelocity the desired velocity
     * @return the desired velocity if possible, if not the current velocity plus the max acceleration
     */
    public double limitLeftAcceleration(double desiredVelocity) {

        //Take the attempted acceleration and see if it is too high.
        if (Math.abs((desiredVelocity - getLeftVelocity()) / TIME_STEP) >= MAX_ACCELERATION) {
            return getLeftVelocity() + MAX_ACCELERATION;
        }

        return desiredVelocity;
    }

    public void setArcadeVelocities(double linearVel, double angularVel){
        setLeftVelocity(linearVel+angularVel* DRIVETRAIN_WIDTH*0.5);
        setRightVelocity(linearVel-angularVel* DRIVETRAIN_WIDTH*0.5);

    }


    public double convertTicksToDistance(int tick) {
        return tick / TICKS_PER_METER;
    }

    @Override
    protected void initDefaultCommand() {

    }
}
