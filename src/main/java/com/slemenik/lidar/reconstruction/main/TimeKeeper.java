package com.slemenik.lidar.reconstruction.main;

import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.TimeUnit;

public class TimeKeeper {

    private static Date buildingsStartTime;
    private static Date buildingsEndTime;
    private static long buildingsTime = 0;
    public static void buildingsStartTime() {
        buildingsStartTime = new Date();
    }
    public static void buildingsEndTime() {
        buildingsEndTime = new Date();
        buildingsTime += (buildingsEndTime.getTime() - buildingsStartTime.getTime());
    }

    private static Date buildingsWriteStartTime;
    private static Date buildingsWriteEndTime;
    private static long buildingsWriteTime = 0;
    public static void buildingsWriteStartTime() {
        buildingsWriteStartTime = new Date();
    }
    public static void buildingsWriteEndTime() {
        buildingsWriteEndTime = new Date();
        buildingsWriteTime += (buildingsWriteEndTime.getTime() - buildingsWriteStartTime.getTime());
    }

    private static Date mountainsStartTime;
    private static Date mountainsEndTime;
    private static long mountainsTime = 0;
    public static void mountainsStartTime() {
        mountainsStartTime = new Date();
    }
    public static void mountainsEndTime() {
        mountainsEndTime = new Date();
        mountainsTime += (mountainsEndTime.getTime() - mountainsStartTime.getTime());
    }

    private static Date projectionStartTime;
    private static Date projectionEndTime;
    private static long projectionTime = 0;
    public static void projectionStartTime() {
        projectionStartTime = new Date();
    }
    public static void projectionEndTime() {
        projectionEndTime = new Date();
        projectionTime += (projectionEndTime.getTime() - projectionStartTime.getTime());
    }

    private static Date borderStartTime;
    private static Date borderEndTime;
    private static long borderTime = 0;
    public static void borderStartTime() {
        borderStartTime = new Date();
    }
    public static void borderEndTime() {
        borderEndTime = new Date();
        borderTime += (borderEndTime.getTime() - borderStartTime.getTime());
    }

    private static Date innerHolesStartTime;
    private static Date innerHolesEndTime;
    private static long innerHolesTime = 0;
    public static void innerHolesStartTime() {
        innerHolesStartTime = new Date();
    }
    public static void innerHolesEndTime() {
        innerHolesEndTime = new Date();
        innerHolesTime += (innerHolesEndTime.getTime() - innerHolesStartTime.getTime());
    }

    private static Date interpolationStartTime;
    private static Date interpolationEndTime;
    private static long interpolationTime = 0;
    public static void interpolationStartTime() {
        interpolationStartTime = new Date();
    }
    public static void interpolationEndTime() {
        interpolationEndTime = new Date();
        interpolationTime += (interpolationEndTime.getTime() - interpolationStartTime.getTime());
    }

    private static Date mountainsWriteStartTime;
    private static Date mountainsWriteEndTime;
    private static long mountainsWriteTime = 0;
    public static void mountainsWriteStartTime() {
        mountainsWriteStartTime = new Date();
    }
    public static void mountainsWriteEndTime() {
        mountainsWriteEndTime = new Date();
        mountainsWriteTime += (mountainsWriteEndTime.getTime() - mountainsWriteStartTime.getTime());
    }

     static void report(){
        System.out.println("--------------------------------------------------------");
        System.out.println("");
        System.out.println("buildingsTime: " + reportTime(buildingsTime));
        System.out.println("mountainsTime: " + reportTime(mountainsTime));

        System.out.println("buildingsWriteTime: " + reportTime(buildingsWriteTime));

        System.out.println("projectionTime: " + reportTime(projectionTime));
        System.out.println("borderTime: " + reportTime(borderTime));
        System.out.println("innerHolesTime: " + reportTime(innerHolesTime));
        System.out.println("interpolationTime: " + reportTime(interpolationTime));
        System.out.println("mountainsWriteTime: " + reportTime(mountainsWriteTime));

         System.out.println("INPUT_FILE_NAME: " + Main.INPUT_FILE_NAME);
         System.out.println("CREATED_POINTS_SPACING, density: " + Main.CREATED_POINTS_SPACING);
         System.out.println("DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD, radius: " + Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD);
         System.out.println("CONSIDER_EXISTING_POINTS, natural: " + Main.CONSIDER_EXISTING_POINTS);
         System.out.println("READ_CLASSIFICATION, classifications: " + Arrays.asList(Main.READ_CLASSIFICATION));
         System.out.println("MOUNTAINS_ANGLE_TOLERANCE_DEGREES, similarity: " + Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES);
         System.out.println("MOUNTAINS_NUMBER_OF_SEGMENTS, segments: " + Main.MOUNTAINS_NUMBER_OF_SEGMENTS);
         System.out.println("INTERPOLATION_STRING, interpolation: " + Main.INTERPOLATION_STRING);


         System.out.println("--------------------------------------------------------");
    }

    private static String reportTime(long time) {
        long minutes = (time / 1000) / 60;
        long seconds = (time / 1000) % 60;

        return time + "ms, " + TimeUnit.MILLISECONDS.toSeconds(time) + "s, " + minutes + ":" + seconds;
    }



}
