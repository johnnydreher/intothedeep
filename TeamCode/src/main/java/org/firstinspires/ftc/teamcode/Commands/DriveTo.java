package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class DriveTo {
    Drivetrain drive;
    PIDControl pid;
    double distance;
    public DriveTo(double distance, Drivetrain drive){
        this.distance = distance;
        this.drive = drive;
        pid = new PIDControl(distance, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    public boolean update(){
        double out = pid.get(drive.getVertical());
        Log.d("driveTo",String.format("Out: %f",out));
        Log.d("driveTo",String.format("Vertical: %f",drive.getVertical()));
        Log.d("driveTo",String.format("Distance: %f",distance));
        drive.moveRobot(out,0,0);
        return pid.atSetpoint();
    }
}
