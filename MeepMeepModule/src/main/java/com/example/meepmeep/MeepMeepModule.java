package com.example.meepmeep;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
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
                        .splineToConstantHeading(new Vector2d(-12,-41.5),Math.toRadians(90))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(180)),Math.toRadians(270))
                        .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                        .lineTo(new Vector2d(fieldHalf-hLength-15,-fieldHalf+hWidth))
                        .lineTo(defaultPoseRed.vec())
                        .splineToConstantHeading(new Vector2d(7.5,-fieldHalf+hDiag),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-12,-41.5, Math.toRadians(270)),Math.toRadians(90))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(180)),Math.toRadians(270))
                        .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                        .lineTo(new Vector2d(fieldHalf-hLength-23,-fieldHalf+hWidth))
                        .splineToConstantHeading(new Vector2d(fieldHalf-hLength-23,-fieldHalf+hDiag+0.5),90)
                        .lineTo(new Vector2d(fieldHalf-23-hLength,-fieldHalf+hWidth+25))
                        .build()
                );
        RoadRunnerBotEntity robotblue = new DefaultBotBuilder(meep)
                .setConstraints(70,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPoseBlue)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-12,41.5),Math.toRadians(270))
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
        RoadRunnerBotEntity robotbluesupport = new DefaultBotBuilder(meep)
                .setConstraints(70,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-31.5,fieldHalf-hLength,Math.toRadians(90)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-61.3,36),Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-30,24, Math.toRadians(0)),Math.toRadians(0))
                                .waitSeconds(4.5)
                                .back(0.5)
                                .splineToConstantHeading(new Vector2d(-61,36),Math.toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity robotredsupport = new DefaultBotBuilder(meep)
                .setConstraints(70,60,Math.toRadians(270),Math.toRadians(180), 14.75)
                .setDimensions(width, length)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-39.5,-fieldHalf+hLength,Math.toRadians(270)))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-55,-36),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-30,-24),Math.toRadians(0))
                        .waitSeconds(4.5)
                        .forward(0.5)
                        .splineToSplineHeading(new Pose2d(-61,-36,Math.toRadians(270)),Math.toRadians(270))
                        .lineTo(new Vector2d(-61,-fieldHalf+7.5+hLength))
                        .waitSeconds(2.5)
                        .lineTo(new Vector2d(-61,-36))
                        .build()
                );
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .addEntity(robotred)
                .addEntity(robotblue)
                .addEntity(robotbluesupport)
                .addEntity(robotredsupport)
                .start();
    }
}