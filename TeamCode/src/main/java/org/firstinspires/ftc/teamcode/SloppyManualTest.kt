package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range

@TeleOp(name = "Please delete this")
class SloppyManualTest : CommonOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Initialize(hardwareMap, false, BingusPipeline.StartLine.RIGHT)
        waitForStart()
        while (opModeIsActive()) {
            forward_axis = logifyInput(gamepad1.left_stick_y.toDouble(), 2)
            strafe_axis = logifyInput(gamepad1.left_stick_x.toDouble(), 2)
            turn_axis = logifyInput(gamepad1.right_stick_x.toDouble(), 2)
            for (motor in movementMotors) {
                motor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            //movementMotors[0].setPower(Range.clip(forward_axis - strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[0]?.power = gamepad2.right_stick_y.toDouble()
            movementMotors[1]?.power = Range.clip(forward_axis + strafe_axis + turn_axis, -1.0, 1.0) * restrictor
            movementMotors[2]?.power = Range.clip(forward_axis + strafe_axis - turn_axis, -1.0, 1.0) * restrictor
            movementMotors[3]?.power = Range.clip(forward_axis - strafe_axis + turn_axis, -1.0, 1.0) * restrictor
            freightServo?.position = (gamepad2.left_stick_y + 1) / 2.0
            telemetry.addData("Pos of arm: ", movementMotors[0]?.currentPosition)
            telemetry.update()
        }
    }
}