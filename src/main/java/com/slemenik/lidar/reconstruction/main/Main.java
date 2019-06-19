package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.buildings.ColorController;
import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.slemenik.lidar.reconstruction.buildings.BuildingController;
import com.slemenik.lidar.reconstruction.mountains.EvenFieldController;
import com.slemenik.lidar.reconstruction.mountains.InterpolationController;
import com.slemenik.lidar.reconstruction.mountains.InterpolationController.Interpolation;

import com.slemenik.lidar.reconstruction.mountains.MountainController;
import com.slemenik.lidar.reconstruction.mountains.triangulation.Triangulation;
import com.sun.j3d.utils.geometry.GeometryInfo;
import com.sun.j3d.utils.geometry.NormalGenerator;
import com.sun.j3d.utils.geometry.Stripifier;
import delaunay_triangulation.Delaunay_Triangulation;
import delaunay_triangulation.Point_dt;
import delaunay_triangulation.Triangle_dt;
import javafx.geometry.Point3D;
import javafx.scene.shape.TriangleMesh;
import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.math3.geometry.Vector;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.locationtech.jts.geom.Coordinate;

import javax.media.j3d.GeometryArray;
import javax.media.j3d.Transform3D;
import javax.vecmath.*;


public class Main {

    public static final String DATA_FOLDER = ".";

    public static final String INPUT_FILE_NAME = DATA_FOLDER + "410_137_triglav.laz";//"triglav okrnjen.laz";
    private static String OUTPUT_FILE_NAME = DATA_FOLDER + "out.laz";
    private static final String TEMP_FILE_NAME = DATA_FOLDER + "temp.laz";
    private static final String DMR_FILE_NAME = DATA_FOLDER + "GK1_410_137.asc";

    private static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
    private static final double CREATED_POINTS_SPACING = 0.6;//2.0;//0.2;
    private static final boolean CONSIDER_EXISTING_POINTS = false; //rešetke
    private static final double BOUNDING_BOX_FACTOR = 1.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    private static final boolean CREATE_TEMP_FILE = true;
    private static final int[] TEMP_BOUNDS = new int[]{462264, 100575, 462411, 100701};
    private static  int COLOR = 6; //6rdeča //5rumena*/; //4-zelena*/;//2-rjava;//3;

    public static void main(String[] args) {
        System.out.println("start main");
        long startTime = System.nanoTime();
        List<double[]> points2Insert = mainTest(args);

        if (!points2Insert.isEmpty()) {
            System.out.println("zacetek pisanja... ");
            double[][] pointListDoubleArray = points2Insert.toArray(new double[][]{});
            System.out.println("Points to write: " + pointListDoubleArray.length);
            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME, COLOR);
            System.out.println("End writing. Points written: " + returnValue);

            ////////temp////////////7
//            if (args.length > 0) {
                int a = args.length+1;
                System.out.println("jovo na novo");
                COLOR = a;
                main(new String[a]);


//            }
            ////////temp////////////7

        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Sekunde izvajanja: " + time);
        System.out.println("end");
    }

    private static List<double[]> mainTest(String[] args) {

                OUTPUT_FILE_NAME = DATA_FOLDER + "nevem.laz";

//        List<double[]> arr2 = new MountainController().parseFolder("C:/Users/Matej/Documents/My Documents/Šola/4. FRI/Magistrska/DMV1000", DATA_FOLDER + "410_137_triglav.laz");


//        List<double[]> arr2 = testMountains(Interpolation.SPLINE);
//        List<double[]> arr2 = testBuildingCreation();
//        List<double[]> arr2 = ArrayList<double[]>(Arrays.asList(arr));
        List<double[]> arr2 = new ArrayList<>();
//        testHeapSpace();
        if (args.length <= 0) {
            arr2 = testMountainController(args);
        }
//        Triangulation.triangulate(DMR_FILE_NAME);




        return arr2;

    }

    public static void testHeapSpace(){

        double[][] arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        double[][] arr2 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        double[][] arr3 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        double[][] arr4 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);

    }

    public static void writeOBJ() {
        MountainController mc = new MountainController(new double[][]{});
        Point_dt[] list = mc.getDmrFromFile(DMR_FILE_NAME);
        Delaunay_Triangulation dt = new Delaunay_Triangulation(list);
        try { dt.write_smf("smf.obj"); }catch (Exception e) {}
    }


    public static List<double[]> testMountainController(String[] args) {
        double[][] arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        MountainController mc = new MountainController(arr);
        arr = null; //!! essential, without it we get OutOfMemoryError: Java heap space
        mc.dmrFileName = DMR_FILE_NAME;
        mc.stopCount = args.length;
        List<double[]> newPoints = mc.start();
//        List <double[]> result = mc.temp();
        OUTPUT_FILE_NAME = DATA_FOLDER + "tests" + args.length + ".laz";
        return newPoints;
    }

    public static List<double[]> testMountainGrid3d(){
        //naredi triglav, 3d, ne naredi praznih mest, ampak tista ki so že nafilana, ampak so x,y koordinate diskretne, mreža
        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        OUTPUT_FILE_NAME = DATA_FOLDER + "triglav grid diskretno.laz";
        EvenFieldController ef = new EvenFieldController(arr, CREATED_POINTS_SPACING);
        ef.interpolation = Interpolation.OWN_VALUE;
        return ef.getPointsFromFieldArray(ef.getBooleanPointField(arr), true);
    }

    public static List<double[]> testBoundary() {
        double[][] arr = JniLibraryHelpers.getPointArray( INPUT_FILE_NAME);
        EvenFieldController mc = new EvenFieldController(arr, CREATED_POINTS_SPACING);

        boolean[][] field = mc.getBooleanPointField(arr);

        boolean[][] newField = mc.getBoundaryField(field);
//        boolean[][] newField = field;


        return mc.getPointsFromFieldArray(newField, true);

    }

    public static List<double[]> testMountains(Interpolation interp) {
        System.out.println("method testMountains()");
        OUTPUT_FILE_NAME = DATA_FOLDER + "triglav4 spacing-"+ CREATED_POINTS_SPACING + " interpolation-"+ interp.toString()+ ".laz";
        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        EvenFieldController efc = new EvenFieldController(arr, CREATED_POINTS_SPACING);
        efc.interpolation = interp;
        return efc.fillHoles(arr);
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
