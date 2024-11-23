package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class DriveTo extends CommandBase {
    private Drivetrain drive;
    private PIDControl pid, pidStrafe,pidYaw;
    private double distance;
    private ElapsedTime to = new ElapsedTime();
    public DriveTo(double distance, Drivetrain drive){
        addRequirements(drive);
        this.distance = distance;
        this.drive = drive;
        pid = new PIDControl(distance, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        pidStrafe = new PIDControl(0,PIDConstants.Kp,0, 0);
        pidYaw = new PIDControl(0,PIDConstants.Kp,0, 0);
        drive.resetEncoders();
    }
    @Override
    public void initialize() {
        to.reset();
    }
    @Override
    public void execute() {
        double out = pid.get(drive.getVertical());
        Log.d("driveTo",String.format("Out: %f",out));
        Log.d("driveTo",String.format("Vertical: %f",drive.getVertical()));
        Log.d("driveTo",String.format("Distance: %f",distance));
        double strafe = pidStrafe.get(drive.getHorizontal());
        double yaw = pidYaw.get(-drive.getHeading(AngleUnit.DEGREES));

        drive.moveRobot(out,strafe,yaw);
    }
    @Override
    public void end(boolean interrupted) {
        drive.power(0);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || to.milliseconds()>2000;
    }
}
