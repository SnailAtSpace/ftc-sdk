package com.example.meepmeep;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepModule {
    public static void main(String[] args){
        MeepMeep meep = new MeepMeep(640);
        final double width = 330.29/25.4, length = 430/25.4, diag = 542.3/25.4;
        final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
        Pose2d startPoseRed = new Pose2d(7.5,-fieldHalf+hLength, Math.toRadians(270));
        Pose2d startPoseBlue = new Pose2d(15,fieldHalf-hLength, Math.toRadians(90));
        Pose2d defaultPoseRed = new Pose2d(7.5,-fieldHalf+hWidth,Math.toRadians(0));
        Pose2d defaultPoseBlue = new Pose2d(7.5,fieldHalf-hWidth,Math.toRadians(0));
        RoadRunnerBotEntity robotred = new DefaultBotBuilder(meep)
                .setConstraints(70,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPoseRed)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-12,-41.5),Math.toRadians(180))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(0)),Math.toRadians(0))
                        .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                        .lineTo(new Vector2d(fieldHalf-hLength-15,-fieldHalf+hWidth))
                        .lineTo(defaultPoseRed.vec())
                        .splineToConstantHeading(new Vector2d(7.5,-fieldHalf+hDiag),Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-12,-41.5, Math.toRadians(270)),Math.toRadians(90))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(0)),Math.toRadians(0))
                        .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                        .lineTo(new Vector2d(fieldHalf-hLength-23,-fieldHalf+hWidth))
                        .splineToConstantHeading(new Vector2d(fieldHalf-hLength-23,-fieldHalf+hDiag+0.5),90)
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(fieldHalf-23-hLength,-fieldHalf+hWidth+20))
                        .build()
                );
        RoadRunnerBotEntity robotblue = new DefaultBotBuilder(meep)
                .setConstraints(70,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPoseBlue)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-12,41.5),Math.toRadians(180))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,fieldHalf-hDiag, Math.toRadians(0)),Math.toRadians(0))
                        .splineToConstantHeading(defaultPoseBlue.vec(),Math.toRadians(90))
                        .lineTo(new Vector2d(fieldHalf-hLength-15,fieldHalf-hWidth))
                        .lineTo(defaultPoseBlue.vec())
                        .splineToConstantHeading(new Vector2d(7.5,fieldHalf-hDiag),Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-12,41.5, Math.toRadians(90)),Math.toRadians(270))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,fieldHalf-hDiag, Math.toRadians(0)),Math.toRadians(0))
                        .splineToConstantHeading(defaultPoseBlue.vec(),Math.toRadians(90))
                        .lineTo(new Vector2d(fieldHalf-hLength-23,fieldHalf-hWidth))
                        .splineToConstantHeading(new Vector2d(fieldHalf-hLength-23,+fieldHalf-hDiag-0.5),270)
                        .turn(Math.toRadians(-90))
                        .lineTo(new Vector2d(fieldHalf-23-hLength,+fieldHalf-hWidth-20))
                        .build()
                );
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .addEntity(robotred)
                .addEntity(robotblue)
                .start();
    }
}