package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.buildings.ColorController;
import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.slemenik.lidar.reconstruction.buildings.BuildingController;
import com.slemenik.lidar.reconstruction.mountains.MountainController;

import com.slemenik.lidar.reconstruction.mountains.triangulation.Triangulation;



public class Main {

    public static final String DATA_FOLDER = ".";

    private static final String INPUT_FILE_NAME = DATA_FOLDER + "out2.laz";
    private static final String OUTPUT_FILE_NAME = DATA_FOLDER + "out.laz";
    private static final String TEMP_FILE_NAME = DATA_FOLDER + "temp.laz";
    private static final String DMR_FILE_NAME = DATA_FOLDER + "GK1_410_137.asc";

    private static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
    private static final double CREATED_POINTS_SPACING = 0.5;//2.0;//0.2;
    private static final boolean CONSIDER_EXISTING_POINTS = false; //rešetke
    private static final double BOUNDING_BOX_FACTOR = 1.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    private static final boolean CREATE_TEMP_FILE = true;
    private static final int[] TEMP_BOUNDS = new int[]{462264, 100575, 462411, 100701};
    private static final int COLOR = 6; //rdeča //5rumena*/; //4-zelena*/;//2-rjava;//3;

    public static void main(String[] args) {
        System.out.println("start main");
        long startTime = System.nanoTime();
        List<double[]> points2Insert = mainTest();

        if (!points2Insert.isEmpty()) {
            System.out.println("zacetek pisanja... ");
            double[][] pointListDoubleArray = points2Insert.toArray(new double[][]{});
            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME, COLOR);
            System.out.println(returnValue);
        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Sekunde izvajanja: " + time);
        System.out.println("end");
    }

    public static List<double[]> mainTest() {


        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        MountainController mc = new MountainController(arr, CREATED_POINTS_SPACING);
        mc.interpolation = MountainController.Interpolation.CONSTANT;
        return mc.getPointsFromFieldArray(mc.getBooleanPointField(arr), true);

//        List<double[]> arr2 = testMountains();
//        List<double[]> arr2 = testBuildingCreation();
//        return arr2;
//        return new ArrayList<double[]>(Arrays.asList(arr));
//        return new ArrayList<>();


    }

    public static List<double[]> testBoundary() {
        double[][] arr = JniLibraryHelpers.getPointArray( INPUT_FILE_NAME);
        MountainController mc = new MountainController(arr, CREATED_POINTS_SPACING);

        boolean[][] field = mc.getBooleanPointField(arr);

        boolean[][] newField = mc.getBoundaryField(field);
//        boolean[][] newField = field;


        return mc.getPointsFromFieldArray(newField, true);

    }

    public static List<double[]> testMountains() {
        System.out.println("method testMountains()");
        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        MountainController mc = new MountainController(arr, CREATED_POINTS_SPACING);
        return mc.fillHoles(arr);
    }

    public static List<double[]> testBuildingCreation() {
        BuildingController bc = new BuildingController();
        bc.inputLazFileName = INPUT_FILE_NAME;
        bc.createTempLazFile = CREATE_TEMP_FILE;
        bc.tempLazFileName = TEMP_FILE_NAME;
        bc.boundingBoxFactor = BOUNDING_BOX_FACTOR;
        bc.createdPointsSpacing = CREATED_POINTS_SPACING;
        bc.distanceFromOriginalPointThreshold = DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD;
        bc.considerExistingPoints = CONSIDER_EXISTING_POINTS;
        bc.outputFileName = OUTPUT_FILE_NAME;
        bc.shpFileName = DATA_FOLDER + "BU_STAVBE_P.shp";
        bc.write(TEMP_BOUNDS);
        return bc.points2Insert;
    }

    public static void testAscFile() {
        new MountainController().readAscFile(DMR_FILE_NAME);
    }

    public static void testTriangulation() {
        Triangulation.triangulate(DMR_FILE_NAME);
    }

    public static void testGoogleMaps() {
        ColorController cc = new ColorController();
        try {
            String a = cc.getHTML("http://maps.googleapis.com/maps/api/streetview?size=600x400&location=12420+timber+heights,+Austin&key=AIzaSyCkUOdZ5y7hMm0yrcCQoCvLwzdM6M8s5qk");
            System.out.println(a);
        }catch (Exception e) {
            System.out.println(e);
        }
    }

}
