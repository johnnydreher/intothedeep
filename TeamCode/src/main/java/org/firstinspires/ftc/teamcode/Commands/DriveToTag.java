package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;

public class DriveToTag extends CommandBase {
    private Drivetrain drive;
    private AprilTag aprilTag;
    private int tag = 0;
    private PIDControl pid, pidStrafe,pidYaw;
    private double distance;
    private ElapsedTime to = new ElapsedTime();
    public DriveToTag(Drivetrain drive, AprilTag aprilTag, int tag){
        addRequirements(drive);
        this.drive = drive;
        this.aprilTag = aprilTag;
        this.tag = tag;
        pid = new PIDControl(15, PIDConstants.Kp,PIDConstants.Ki, PIDConstants.Kd);
        pidStrafe = new PIDControl(0,PIDConstants.Kp,0, 0);
        pidYaw = new PIDControl(0,PIDConstants.Kp,0, 0);
        if (aprilTag.isTag(tag)) {
            distance = aprilTag.getRange(tag);
        }
        drive.resetEncoders();
    }
    @Override
    public void initialize() {
        to.reset();
    }
    @Override
    public void execute() {
        double out = pid.get(aprilTag.getRange(tag));
        Log.d("driveToTag",String.format("Out: %f",out));
        Log.d("driveToTag",String.format("Vertical: %f",drive.getVertical()));
        Log.d("driveToTag",String.format("Distance: %f",distance));
        double strafe = pidStrafe.get(drive.getHorizontal());
        double yaw = pidYaw.get(-aprilTag.getYaw(tag));

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
