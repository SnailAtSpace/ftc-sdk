package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    DcMotor FRmotor;
    DcMotor RRmotor;
    DcMotor FLmotor;
    DcMotor RLmotor;
    DcMotor Flywheel;
    DcMotor Worm;
    Servo Grabber;
    Servo Pushrod;
    DcMotor Collector;
    DcMotorEx FlywheelEx;
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    BNO055IMU imu;
    final double rpm = 3625;
    boolean isFlywheelRunning = false;
    public BingusPipeline.RandomizationFactor ringData=BingusPipeline.RandomizationFactor.ZERO;
    final int LogPower=3;
    boolean grab = false,push = false,collector = false,flywheel = false,flick = false;
    boolean prevgrab,prevpush,prevfly,prevcoll,prevflick;
    double for_axis,strafe_axis,turn_axis,worm_axis;
    public void Initialize(HardwareMap hardwareMap,boolean isAuto) {
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Grabber = hardwareMap.get(Servo.class, "Gservo");
        Pushrod = hardwareMap.get(Servo.class, "Pservo");
        Collector = hardwareMap.get(DcMotor.class, "Cmotor");
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Grabber.scaleRange(0.16,0.66);
        Pushrod.scaleRange(0.19,0.3);
        FlywheelEx = (DcMotorEx)(Flywheel);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "calib.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        FlywheelEx.setVelocityPIDFCoefficients(
                10,
                1.5,
                4,
                FlywheelEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f
        );
        if(isAuto) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new BingusPipeline();
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            telemetry.addLine("Waiting for start.");
            telemetry.addLine("Please calibrate starting position.");
            telemetry.update();

        }
    }

    public void ReadyPeripherals(){
        Grabber.setPosition(0);
        Pushrod.setPosition(0);
        ExecuteFlag=false;
    }

    public void AutoRingLaunch(){
        MoveWithEncoder(1454, 2);
        OrientToDegrees(-10);
        LaunchSeveralRings(3);
        OrientToDegrees(0);
        if(ringData!=BingusPipeline.RandomizationFactor.ZERO) {
            MoveWithEncoder(285, 3);
            Collector.setPower(0.75);
            MoveWithEncoder(1450,0);
            Collector.setPower(-0.75);
            sleep(100);
            MoveWithEncoder(1450,2);
            MoveWithEncoder(285,1);
            OrientToDegrees(-10);
            if(ringData==BingusPipeline.RandomizationFactor.ONE)LaunchSeveralRings(1);
            else LaunchSeveralRings(3);
            OrientToDegrees(0);
        }
        OrientToDegrees(180);
    }

    public void MoveWithEncoder(int millis,int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=right, 2=forward, 3=left
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setTargetPosition((int)(millis*((direction-1)*2-1)*1.71));
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setPower(1);
        DcMotorEx FRmotorEx = (DcMotorEx)FRmotor;
        while(FRmotor.getCurrentPosition()!=FRmotor.getTargetPosition()&&opModeIsActive()&&FRmotor.isBusy()){
            RLmotor.setPower(Math.signum((direction-1)*2-1)*FRmotorEx.getVelocity());
            RRmotor.setPower(Math.signum(direction%3*2-1)*FRmotorEx.getVelocity());
            FLmotor.setPower(Math.signum(direction%3*2-1)*FRmotorEx.getVelocity());
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(250);
    }

    public void DeployArm(){
        Worm.setPower(-1);
        sleep(2100);
        Worm.setPower(0);
        Grabber.setPosition(1);
        sleep(2000);
    }

    public void RetractArm(){
        Worm.setPower(1);
        sleep(1050);
        Worm.setPower(0);
    }

    public void LaunchSeveralRings(int amount) {
        FlywheelEx.setVelocity(rpm*28/60.0);
        while (FlywheelEx.getVelocity()<rpmToTps(rpm) && opModeIsActive()) {}
        Pushrod.setPosition(1);
        safeSleep(100);
        Pushrod.setPosition(0);
        for (int i = 0; i <= amount-1; i++) {
            while (opModeIsActive()&&FlywheelEx.getVelocity()<rpmToTps(rpm)){}
            Pushrod.setPosition(1);
            safeSleep(100);
            Pushrod.setPosition(0);
        }
        Flywheel.setPower(0);
    }

    public void safeSleep(int millis){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(localTime.time()<millis&&opModeIsActive())idle();
    }

    public double rpmToTps(double rpm){
        return rpm*28/60.0;
    }

    public static double logarithmifyInput(double input, int power) {
        return Math.abs(Math.pow(input,power))*Math.signum(input);
    }

    public void composeInputs(){
        prevgrab=grab;
        prevpush=push;
        prevfly=flywheel;
        prevcoll=collector;
        prevflick=flick;
        for_axis = logarithmifyInput(gamepad1.left_stick_y,LogPower);
        strafe_axis = logarithmifyInput(gamepad1.left_stick_x,LogPower);
        turn_axis = logarithmifyInput(gamepad1.right_stick_x,LogPower);
        worm_axis = logarithmifyInput(gamepad2.left_stick_y,LogPower);
        flick = gamepad1.x;
        grab = gamepad2.left_bumper;
        flywheel = gamepad2.right_bumper;
        push = ((int)(gamepad2.right_trigger+0.25) != 0);
        collector = (gamepad2.dpad_down)||(gamepad2.dpad_up);
    }

    public void operatePeripherals(){
        FRmotor.setPower(for_axis - strafe_axis + turn_axis);
        RRmotor.setPower(for_axis + strafe_axis + turn_axis);
        FLmotor.setPower(for_axis + strafe_axis - turn_axis);
        RLmotor.setPower(for_axis - strafe_axis - turn_axis);
        Worm.setPower(worm_axis);
        if(!prevgrab && grab) {
            Grabber.setPosition(1-Grabber.getPosition());
        }
        if(!prevpush && push){
            Pushrod.setPosition(1);
            sleep(100);
            Pushrod.setPosition(0);
        }
        if(!prevfly&&flywheel){
            FlywheelEx.setVelocity((rpm-(isFlywheelRunning?1:0)*rpm)*28.0/60.0);
            isFlywheelRunning=!isFlywheelRunning;
        }
        if(!prevcoll&&collector){
            if(gamepad2.dpad_down){
                Collector.setPower(-0.75-Math.abs(Collector.getPower())*Math.signum(Math.signum(Collector.getPower())-1));
            }
            else Collector.setPower(0.75-Collector.getPower()*Math.signum(Math.signum(Collector.getPower())+1));
        }
        if(!prevflick&&flick)OrientToDegrees(-10);
    }

    public void OrientToDegrees(float angle){
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        float err;
        float prev_err=0, integral=0, derivative;
        final float pcoef=3,icoef=0.01f,dcoef=0.2f;
        while(((int)currentAngle<(int)angle-1||(int)currentAngle>(int)angle+1)&&opModeIsActive()){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            err = (currentAngle-angle)/180;
            float proportional = err*pcoef;
            integral = integral+err;
            derivative = (err-prev_err)*dcoef;
            float output = proportional+icoef*integral+derivative;
            if(prev_err*err<0)integral=0;
            prev_err=err;
            FLmotor.setPower(-output*0.3);
            RLmotor.setPower(-output*0.3);
            FRmotor.setPower(output*0.3);
            RRmotor.setPower(output*0.3);
            telemetry.addData("Heading in degrees:",currentAngle);
            telemetry.addData("P",proportional);
            telemetry.addData("I",integral*icoef);
            telemetry.addData("D",derivative);
            telemetry.addData("O",output);
            telemetry.addData("R",angle);
            telemetry.update();
        }
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        RRmotor.setPower(0);
        RLmotor.setPower(0);
        sleep(50);
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if((int)currentAngle>(int)angle+1||(int)currentAngle<(int)angle-1)OrientToDegrees(angle);
    }
}