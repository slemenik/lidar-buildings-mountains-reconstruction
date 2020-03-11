package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.mountains.InterpolationController;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.LocalTime;
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

    static void report() {
        System.out.println("--------------------------------------------------------");
        System.out.println("");
        System.out.println("buildingsTime: " + reportTime(buildingsTime));
        System.out.println("mountainsTime: " + reportTime(mountainsTime));
        System.out.println("--");
        System.out.println("buildingsWriteTime: " + reportTime(buildingsWriteTime));
        System.out.println("--");
        System.out.println("projectionTime: " + reportTime(projectionTime));
        System.out.println("borderTime: " + reportTime(borderTime));
        System.out.println("innerHolesTime: " + reportTime(innerHolesTime));
        System.out.println("interpolationTime: " + reportTime(interpolationTime));
        System.out.println("mountainsWriteTime: " + reportTime(mountainsWriteTime));
        System.out.println("--");
        System.out.println("INPUT_FILE_NAME: " + Main.INPUT_FILE_NAME);
        System.out.println("CREATED_POINTS_SPACING, density: " + Main.CREATED_POINTS_SPACING);
        System.out.println("--");
        System.out.println("LAS_2_LAS_FILE_NAME, las2las, temp: " + Main.LAS_2_LAS_FILE_NAME.equals("las2las.exe"));
        System.out.println("DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD, radius: " + Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD);
        System.out.println("CONSIDER_EXISTING_POINTS, natural: " + Main.CONSIDER_EXISTING_POINTS);
        System.out.println("--");
        System.out.println("READ_CLASSIFICATION, classifications: " + Arrays.asList(Main.READ_CLASSIFICATION));
        System.out.println("MOUNTAINS_ANGLE_TOLERANCE_DEGREES, similarity: " + Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES);
        System.out.println("MOUNTAINS_NUMBER_OF_SEGMENTS, segments: " + Main.MOUNTAINS_NUMBER_OF_SEGMENTS);
        System.out.println("INTERPOLATION_STRING, interpolation: " + Main.INTERPOLATION_STRING);
        System.out.println("--------------------------------------------------------");

        //writeToLog
        printToLog("--------------------------------------------------------");
        printToLog("");
        printToLog("buildingsTime: " + reportTime(buildingsTime));
        printToLog("mountainsTime: " + reportTime(mountainsTime));
        printToLog("--");
        printToLog("buildingsWriteTime: " + reportTime(buildingsWriteTime));
        printToLog("--");
        printToLog("projectionTime: " + reportTime(projectionTime));
        printToLog("borderTime: " + reportTime(borderTime));
        printToLog("innerHolesTime: " + reportTime(innerHolesTime));
        printToLog("interpolationTime: " + reportTime(interpolationTime));
        printToLog("mountainsWriteTime: " + reportTime(mountainsWriteTime));
        printToLog("--");
        printToLog("INPUT_FILE_NAME: " + Main.INPUT_FILE_NAME);
        printToLog("CREATED_POINTS_SPACING, density: " + Main.CREATED_POINTS_SPACING);
        printToLog("--");
        printToLog("LAS_2_LAS_FILE_NAME, las2las, temp: " + Main.LAS_2_LAS_FILE_NAME.equals("las2las.exe"));
        printToLog("DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD, radius: " + Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD);
        printToLog("CONSIDER_EXISTING_POINTS, natural: " + Main.CONSIDER_EXISTING_POINTS);
        printToLog("--");
        printToLog("READ_CLASSIFICATION, classifications: " + Arrays.asList(Main.READ_CLASSIFICATION));
        printToLog("MOUNTAINS_ANGLE_TOLERANCE_DEGREES, similarity: " + Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES);
        printToLog("MOUNTAINS_NUMBER_OF_SEGMENTS, segments: " + Main.MOUNTAINS_NUMBER_OF_SEGMENTS);
        printToLog("INTERPOLATION_STRING, interpolation: " + Main.INTERPOLATION_STRING);
        printToLog("--------------------------------------------------------");
    }

    private static String reportTime(long time) {
        long minutes = (time / 1000) / 60;
        long seconds = (time / 1000) % 60;

        return time + "ms, " + TimeUnit.MILLISECONDS.toSeconds(time) + "s, " + minutes + ":" + seconds;
    }

    private static String[] inputFileNameArray = new String[]{
            Main.DATA_FOLDER + "GK_431_136",
            Main.DATA_FOLDER + "GK_430_136",
            Main.DATA_FOLDER + "GK_462_100",
            Main.DATA_FOLDER + "GK_429_135",

            Main.DATA_FOLDER + "GK_430_135",
            Main.DATA_FOLDER + "GK_410_138",
            Main.DATA_FOLDER + "GK_429_136",
            Main.DATA_FOLDER + "GK_431_135"
    };
    private static double[] densityList = {0.2,  2}; //0.6,
//    private static String[] tempList = {"las2las.exe", "asd.exe"};
    private static double[] radiusList = {0.2,  1.6}; //0.8,
    private static boolean[] naturalList = {true }; //false
    private static double[] similarityList = {10, 90}; //30,
    private static double[] segmentsList = {3,  30}; //15,
    private static int[] interpolationList = {  4, 0 }; //,  5, 6//10 NEAREST_N, AVERAGE_24N, LINEAR, QUADRATIC, SPLINE
    private static String[] dmrList = inputFileNameArray;
    private static Integer[][] classificationsList = new Integer[][]{
            {0, 1, 2, 7, 8},
            {0, 1, 2, 3, 6, 7, 8}
    };


    public static void testTimeFunction(int y) {
        try {
            for (int x = y; x < inputFileNameArray.length; x++) {
                Main.INPUT_FILE_NAME = inputFileNameArray[x];

                if (x <= 3) {
                    Main.CALCULATE_BUILDINGS = true;
                    Main.CALCULATE_MOUNTAINS = false;
                } else {
                    Main.CALCULATE_BUILDINGS = false;
                    Main.CALCULATE_MOUNTAINS = true;
                }

                //najprej poženemo z default vrednostmi
//                resetToDefault();
//                try {
//                    printToLog("defauflt");
//                    Main.pipelineStart();
//                } catch (Exception e) {
//                    printToLog(e.toString());
//                    printToLog(Main.INPUT_FILE_NAME);
//                }

                resetToDefault();
                for (int a = 0; a < densityList.length; a++) {
                    printToLog("densityList, " + a);
                    try {
                        Main.CREATED_POINTS_SPACING = densityList[a];
                        Main.pipelineStart();
                    } catch (Exception e) {
                        printToLog(e.toString());
                        printToLog(Main.INPUT_FILE_NAME);
                    }
                }

                Main.CALCULATE_BUILDINGS = false;
                Main.CALCULATE_MOUNTAINS = false;

                if (Main.CALCULATE_BUILDINGS) {
//                resetToDefault();
//                for (int b = 0; b < tempList.length; b++) {
//                    try {
//                        Main.LAS_2_LAS_FILE_NAME = tempList[b];
//                        Main.pipelineStart();
//                    } catch (Exception e) {
//                        printToLog(e.toString());
//                        printToLog(Main.INPUT_FILE_NAME);
//                        printToLog("tempList, " + b);
//                    }
//                }

                    resetToDefault();
                    for (int c = 0; c < radiusList.length; c++) {
                        printToLog("radiusList, " + c);
                        try {
                            Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = radiusList[c];
                            Main.pipelineStart();
                        } catch (Exception e) {
                            printToLog(e.toString());
                            printToLog(Main.INPUT_FILE_NAME);
                        }
                    }

                    resetToDefault();
                    for (int d = 0; d < naturalList.length; d++) {
                        printToLog("naturalList, " + d);
                        try {
                            Main.CONSIDER_EXISTING_POINTS = naturalList[d];
                            Main.pipelineStart();
                        } catch (Exception e) {
                            printToLog(e.toString());
                            printToLog(Main.INPUT_FILE_NAME);
                        }
                    }
                }

                if (Main.CALCULATE_MOUNTAINS) {
//                    resetToDefault();
//                    for (int e = 0; e < similarityList.length; e++) {
//                        printToLog("similarityList, " + e);
//                        try {
//                            Main.DMR_FILE_NAME = Main.INPUT_FILE_NAME + ".asc";
//                            Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES = similarityList[e];
//                            Main.pipelineStart();
//                        } catch (Exception ee) {
//                            printToLog(ee.toString());
//                            printToLog(Main.INPUT_FILE_NAME);
//                        }
//                    }

//                    resetToDefault();
//                    for (int f = 0; f < segmentsList.length; f++) {
//                        printToLog("segmentsList, " + f);
//                        try {
//                            Main.MOUNTAINS_NUMBER_OF_SEGMENTS = segmentsList[f];
//                            Main.pipelineStart();
//                        } catch (Exception e) {
//                            printToLog(e.toString());
//                            printToLog(Main.INPUT_FILE_NAME);
//                        }
//                    }

                    resetToDefault();
                    for (int g = 0; g < interpolationList.length; g++) {
                        printToLog("interpolationList, " + g);
                        try {
                            Main.INTERPOLATION_STRING = InterpolationController.Interpolation.values()[interpolationList[g]].name();
                            Main.pipelineStart();
                        } catch (Exception e) {
                            printToLog(e.toString());
                            printToLog(Main.INPUT_FILE_NAME);
                        }
                    }

//                    resetToDefault();
//                    for (int h = 0; h < classificationsList.length; h++) {
//                        printToLog("classificationsList, " + h);
//                        try {
//                            Main.READ_CLASSIFICATION = classificationsList[h];
//                            Main.pipelineStart();
//                        } catch (Exception e) {
//                            printToLog(e.toString());
//                            printToLog(Main.INPUT_FILE_NAME);
//                        }
//                    }
                } // if (Main.CALCULATE_MOUNTAINS) {
            } //end for inputFiles
        } catch (Exception e) {
            printToLog(e.toString());
            printToLog(Main.INPUT_FILE_NAME);
            y++;
            testTimeFunction(y);
        }
    }

    public static void printToLog(String a){
        File log = new File(Main.DATA_FOLDER + "log.txt");
        try{
            PrintWriter out = new PrintWriter(new FileWriter(log, true));
            out.append("[" + LocalTime.now().withNano(0) + "] " + a + "\n");
            out.close();
        }catch(Exception e){
            System.out.println("COULD NOT LOG!!");
        }
    }

    public static void resetToDefault(){
        Main.CREATED_POINTS_SPACING = 0.6;
        Main.LAS_2_LAS_FILE_NAME = "las2las.exe";
        Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8;
        Main.CONSIDER_EXISTING_POINTS = false;
        Main.DMR_FILE_NAME = "";
        Main.READ_CLASSIFICATION = new Integer[]{0, 1, 2, 3, 4, 5, 6, 7, 8};
        Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES = 30;
        Main.MOUNTAINS_NUMBER_OF_SEGMENTS = 15;
        Main.INTERPOLATION_STRING = "SPLINE";

        //reset timer
        buildingsTime = 0;
        buildingsWriteTime = 0;
        mountainsTime = 0;
        projectionTime = 0;
        borderTime = 0;
        innerHolesTime = 0;
        interpolationTime = 0;
        mountainsWriteTime = 0;
    }

}
