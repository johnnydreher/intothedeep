package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class strafeTo {
    Drivetrain drive;
    PIDControl pid;
    double distance;
    public strafeTo(double distance, Drivetrain drive){
        this.distance = distance;
        this.drive = drive;
        pid = new PIDControl(distance, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    public boolean update(){
        double out = pid.get(drive.getHorizontal());
        drive.moveRobot(0,out,0);
        return pid.atSetpoint();
    }
}

