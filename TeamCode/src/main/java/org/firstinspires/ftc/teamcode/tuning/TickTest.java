package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@TeleOp
public class TickTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);
            int start0, start1, startp;
            waitForStart();
            start0 = localizer.par0.getPositionAndVelocity().position;
            start1 = localizer.par1.getPositionAndVelocity().position;
            startp = localizer.perp.getPositionAndVelocity().position;
            while (opModeIsActive()) {


                telemetry.addData("left", localizer.par0.getPositionAndVelocity().position);
                telemetry.addData("right", localizer.par1.getPositionAndVelocity().position - start1);
                telemetry.addData("perp", localizer.perp.getPositionAndVelocity().position - startp);
                telemetry.update();
            }
        }
    }
}
