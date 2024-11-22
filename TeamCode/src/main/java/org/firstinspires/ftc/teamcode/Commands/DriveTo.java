package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class DriveTo extends CommandBase {
    Drivetrain drive;
    PIDControl pid;
    double distance;
    public DriveTo(double distance, Drivetrain drive){
        addRequirements(drive);
        this.distance = distance;
        this.drive = drive;
        pid = new PIDControl(distance, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        double out = pid.get(drive.getVertical());
        Log.d("driveTo",String.format("Out: %f",out));
        Log.d("driveTo",String.format("Vertical: %f",drive.getVertical()));
        Log.d("driveTo",String.format("Distance: %f",distance));
        drive.moveRobot(out,0,0);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
