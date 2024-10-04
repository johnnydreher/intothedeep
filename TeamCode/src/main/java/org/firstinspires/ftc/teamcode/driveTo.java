package org.firstinspires.ftc.teamcode;

public class driveTo {
    Drivetrain drive;
    PIDControl pid;
    double distance;
    public driveTo(double distance, Drivetrain drive){
        this.distance = distance;
        this.drive = drive;
        pid = new PIDControl(distance,PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    public boolean update(){
        double out = pid.get(drive.getVertical());
        drive.moveRobot(out,0,0);
        return pid.atSetpoint();
    }
}
