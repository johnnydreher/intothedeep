package org.firstinspires.ftc.teamcode;

public class turnTo {
    Drivetrain drive;
    PIDControl pid;
    double angle;
    public turnTo(double angle, Drivetrain drive){
        this.angle = angle;
        this.drive = drive;
        pid = new PIDControl(angle,PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        drive.resetEncoders();
    }
    public boolean update(){
        double out = pid.get(drive.getHeading());
        drive.moveRobot(0,0,-out);
        return pid.atSetpoint();
    }
}

