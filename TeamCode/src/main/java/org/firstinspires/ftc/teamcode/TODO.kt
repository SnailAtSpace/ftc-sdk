package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "If this shows up, you're a legend")
class TODO : LinearOpMode() {
    override fun runOpMode(){
        telemetry.addLine("YEAAAH")
        telemetry.update()
    }
}