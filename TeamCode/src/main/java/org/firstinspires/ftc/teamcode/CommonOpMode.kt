package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.BingusPipeline.RandomizationFactor
import org.firstinspires.ftc.teamcode.BingusPipeline.StartLine
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
@TeleOp
abstract class CommonOpMode : LinearOpMode() {
    var movementMotors = arrayOfNulls<DcMotorEx>(4)
    var webcam: OpenCvCamera? = null
    var pipeline: BingusPipeline? = null
    var ExecuteFlag: Boolean? = null
    var imu: BNO055IMU? = null
    var freightServo: Servo? = null
    var side = StartLine.RIGHT
    enum class Color {
        BLUE, RED
    }
    var color: Color? = null
    var ringData = RandomizationFactor.ZERO
    val LogPower = 3
    val restrictor = 1.0
    var forward_axis = 0.0
    var strafe_axis = 0.0
    var turn_axis = 0.0
    var worm_axis = 0.0
    var parameters = BNO055IMU.Parameters()
    fun Initialize(hardwareMap: HardwareMap, isAuto: Boolean, side: StartLine) {
        freightServo = hardwareMap.get(Servo::class.java, "FreightServo")
        movementMotors[0] = hardwareMap.get(DcMotor::class.java, "FRmotor") as DcMotorEx
        movementMotors[1] = hardwareMap.get(DcMotor::class.java, "RRmotor") as DcMotorEx
        movementMotors[2] = hardwareMap.get(DcMotor::class.java, "FLmotor") as DcMotorEx
        movementMotors[3] = hardwareMap.get(DcMotor::class.java, "RLmotor") as DcMotorEx
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        for (motor in movementMotors) {
            motor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        movementMotors[0]!!.direction = DcMotorSimple.Direction.REVERSE
        movementMotors[2]!!.direction = DcMotorSimple.Direction.REVERSE
        if (isAuto) {
            val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
            pipeline = BingusPipeline()
            webcam?.setPipeline(pipeline)
            pipeline!!.setSide(side)
            webcam?.openCameraDeviceAsync(AsyncCameraOpenListener { webcam?.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT) })
            telemetry.addLine("Waiting for start.")
            telemetry.addLine("Please calibrate starting position.")
            telemetry.update()
        }
        imuInitialization()
        this.side = side
    }

    fun safeSleep(millis: Int) {
        val localTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        while (localTime.time() < millis && opModeIsActive()) idle()
    }

    fun rpmToTps(rpm: Double): Double {
        return rpm * 28 / 60.0
    }

    fun imuInitialization() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "calib.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        parameters.mode = BNO055IMU.SensorMode.NDOF
        imu!!.initialize(parameters)
    }

    companion object {
        fun logifyInput(input: Double, power: Int): Double {
            return Math.abs(Math.pow(input, power.toDouble())) * Math.signum(input)
        }
    }
}