package com.example.meepmeep;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepModule {
    public static void main(String[] args){
        MeepMeep meep = new MeepMeep(640);
        final double width = 330.29/25.4, length = 430/25.4;
        final double hWidth = width/2, hLength = length/2, fieldHalf = 70.5;
        Pose2d startPose = new Pose2d(7.25,-fieldHalf+hLength, Math.toRadians(270));
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meep)
                .setConstraints(75,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-12.3,-41.6),Math.toRadians(90))
                        .waitSeconds(1.5)//add arm and basket engaging before this line
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7,-fieldHalf+hLength, Math.toRadians(0)),Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(15,-fieldHalf+hWidth),0)
                        .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))//add temporal offset collectorMotor before this line
                        .back(2)
                        .lineTo(new Vector2d(15,-fieldHalf+hWidth))
                        .splineToConstantHeading(new Vector2d(7,-fieldHalf+hLength),Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-12.3,-41.6, Math.toRadians(270)),Math.toRadians(180))
                        .waitSeconds(1.5)
                        .build()
                );
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .addEntity(robot)
                .start();
    }
}