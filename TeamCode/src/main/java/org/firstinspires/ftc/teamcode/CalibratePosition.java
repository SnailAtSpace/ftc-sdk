package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
@TeleOp(name = "Calirbate TeleOp position")
public class CalibratePosition extends CommonOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false);
        final Pose2d calibratedPose = new Pose2d(-fieldHalf + hLength, -fieldHalf + hWidth, 0);
        drive.setPoseEstimate(calibratedPose);
        drive.update();
        telemetry.addLine("Place the robot in the red storage unit corner, facing the warehouse.");
        waitForStart();
        drive.setPoseEstimate(calibratedPose);
        Pose2d lastPose = drive.getPoseEstimate();
        String filename = "LastPosition";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, lastPose.getX()+" "+lastPose.getY()+" "+lastPose.getHeading());
    }
}
