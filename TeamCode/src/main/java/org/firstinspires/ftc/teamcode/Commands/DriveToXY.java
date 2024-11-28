package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class DriveToXY extends CommandBase {
    private Drivetrain drive;
    private PIDControl pid, pidStrafe,pidYaw;
    private ElapsedTime to = new ElapsedTime();
    public DriveToXY(double x, double y , Drivetrain drive){
        addRequirements(drive);
        this.drive = drive;
        drive.resetEncoders();
        pid = new PIDControl(x, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        pidStrafe = new PIDControl(y,PIDConstants.Kp,0, 0);
        pidYaw = new PIDControl(0,PIDConstants.Kp,0, 0);

    }
    @Override
    public void initialize() {
        to.reset();
    }
    @Override
    public void execute() {
        double out = pid.get(drive.getVertical());
        double strafe = pidStrafe.get(drive.getHorizontal());
        double yaw = pidYaw.get(-drive.getHeading(AngleUnit.DEGREES));
        if(out>0.4) out = 0.4;
        drive.moveRobot(out,strafe,yaw);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || to.milliseconds()>2500;
    }
}
