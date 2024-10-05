package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class TurnTo {
    Drivetrain drive;
    PIDControl pid;
    double angle;
    public TurnTo(double angle, Drivetrain drive){
        this.angle = angle;
        this.drive = drive;
        pid = new PIDControl(angle, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    public boolean update(){
        double out = pid.get(drive.getHeading());
        drive.moveRobot(0,0,-out);
        return pid.atSetpoint();
    }
}

