package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.wpilibj.command.Command;
import robot.Constants;
import robot.Robot;
import edu.wpi.first.networktables.NetworkTableEntry;


public class VelocityDrive extends Command {

    private double angularVelocity;
    private double linearVelocity;
    private double angularOutput;
    private NetworkTableEntry linearVelocityEntry = Robot.velocityTable.getEntry("distance");
    private NetworkTableEntry angularVelocityEntry = Robot.velocityTable.getEntry("angle");
    private MiniPID angular = new MiniPID(Constants.Drivetrain.angularPID[0], Constants.Drivetrain.angularPID[1], Constants.Drivetrain.angularPID[2]);


    public VelocityDrive(double angularVelocity, double linearVelocity) {
        requires(Robot.m_drivetrain);
        this.angularVelocity = angularVelocity;
        this.linearVelocity = linearVelocity;
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        angularOutput = angular.getOutput(Robot.m_drivetrain.getAngularVelocity(), angularVelocity);
        Robot.m_drivetrain.setArcadeVelocities(linearVelocity,angularVelocity + angularOutput);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {

    }

    @Override
    protected void end() {

    }
}
