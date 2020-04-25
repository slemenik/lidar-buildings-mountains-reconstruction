package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import java.awt.*;
import java.sql.Array;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.slemenik.lidar.reconstruction.buildings.BuildingController;
import com.slemenik.lidar.reconstruction.mountains.EvenFieldController;
import com.slemenik.lidar.reconstruction.mountains.InterpolationController.Interpolation;

import com.slemenik.lidar.reconstruction.mountains.MountainController;
import org.apache.commons.lang3.ArrayUtils;

import javax.vecmath.Point3d;


public class Main {

    public static final String DATA_FOLDER = ".";

    public static String LAS_2_LAS_FILE_NAME = "las2las.exe";
    private static boolean CALCULATE_BUILDINGS = true;
    private static boolean CALCULATE_MOUNTAINS = true;

    public static  String DLL_FILE_NAME = "C:/Users/Matej/source/repos/Project2/x64/Debug/Project2.dll";
    public static String INPUT_FILE_NAME = DATA_FOLDER + "GK_431_136_bled_grad";//"original+odsek";//"GK_431_136_bled_grad";//"GK_410_137_triglav_okrnjen";//"GK_458_109";//"410_137_triglav";//"original+odsek";//"triglav okrnjen.laz";
    public static String OUTPUT_FILE_NAME = DATA_FOLDER + "out"; //todo dodaj nazaj .laz, tud na inputu, če spreminjaš to, potem moraš pogledat povsod kjer vpliva, tudi na DMR_INPUT_FILE
    public static String TEMP_FILE_NAME = DATA_FOLDER + "temp";
    public static String DMR_FILE_NAME = DATA_FOLDER + "GK_410_137.asc";//"GK1_410_137.asc";
//    public static final String DMR_FILE_NAME = INPUT_FILE_NAME + ".asc";//DATA_FOLDER + "GK1_458_109.asc";//"GK1_410_137.asc";
//    public static final String DMR_FILE_NAME = INPUT_FILE_NAME.substring(0, DATA_FOLDER.length() + 10) + ".asc";//DATA_FOLDER + "GK1_458_109.asc";//"GK1_410_137.asc";
    public static  String SHP_FILE_NAME = DATA_FOLDER + "stavbe/BU_STAVBE_P.shp";
    private static final double MAX_POINT_NUMBER_IN_MEMORY_MILLION = 1; // največje število točk originalne datoteke, ki ga še preberemo v pomnilnik
                                                                      // če je točk več, se razdeli v več ločenih branj
    private static final int SEGMENT_LENGTH_X_COORDINATE_LAS_FILE = 5; // the difference between min and max X coordinate,
                                                                        // used for limiting read points so we read them
                                                                        // segment by segments. By default the full file
                                                                        // has a range of 1000, ex. 410000-411000
                                                                        //~15mio points , range of 1 ~15k points

    public static double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
    public static double CREATED_POINTS_SPACING = 0.6;//2.0;//0.2;
    public static boolean CONSIDER_EXISTING_POINTS = false; //rešetke
    public static final double BOUNDING_BOX_FACTOR = 1.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    public static final boolean CREATE_TEMP_FILE = true;
    //public static final double[] TEMP_BOUNDS = new double[]{462264, 100575, 462411, 100701};
    public static Integer[] READ_CLASSIFICATION = new Integer[]{0,1,2,3,4,5,6,7,8};
    public enum Classification {
        NEVER_CLASSIFIED,   //0 - ustvarjana, vendar nikoli klasificirane točke//črna
        UNASSIGNED,         //1- neklasificirane točke //bela
        GROUND,             //2- tla(ang. ground) //temno rjava
        VEGETATION_LOW,     //3- nizka vegetacija, do 1 m // temno zelena
        VEGETATION_MEDIUM,  //4- srednja vegetacija, 1 m do 3 m //svetlo zelena
        VEGETATION_HIGH,    //5 - visoka vegetacija, nad 3 m //rumena
        BUILDING,           //6 - zgradbe //rdeča
        LOW_POINT,          //7- nizka točka (šum)
        NONE,               //8 - reserved (ni v ARSO)
        WATER               //9 - modra, voda (ni v ARSO)
        ;
        public int getIntValue() {
            return this.ordinal();
        }

        public double getDoubleValue() {
            return this.ordinal();
        }
    }

    public static double MOUNTAINS_ANGLE_TOLERANCE_DEGREES = 30;
    public static double MOUNTAINS_NUMBER_OF_SEGMENTS = 5;

    public static String INTERPOLATION_STRING = Interpolation.SPLINE.name();
//    public static Interpolation INTERPOLATION = Interpolation.valueOf(INTERPOLATION_STRING);//Interpolation.AVERAGE_8N;

    public static void main(String[] args) {
        long startTime = System.nanoTime();
        System.out.println("start main");
        setArgs(args);

        long heapMaxSize = Runtime.getRuntime().maxMemory();
        System.out.println("heapmaxsize "+HelperClass.formatHeapSize(heapMaxSize));

        pipelineStart();
//        double[][] pointListDoubleArray = new double[][]{};
//        double[][] pointListDoubleArray = mainTest(args);

//        if (pointListDoubleArray.length > 0) {
//            System.out.println("zacetek pisanja... ");
//            System.out.println("Points to write: " + pointListDoubleArray.length);
//            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME);
//            System.out.println("End writing. Points written: " + returnValue);
//        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Minute izvajanja: " + time/60);
        System.out.println("end");
    }

    private static void setArgs(String[] args) {
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "-in":
                    Main.INPUT_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-dll":
                    Main.DLL_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-out":
                    Main.OUTPUT_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-density":
                    Main.CREATED_POINTS_SPACING = Double.parseDouble(args[i+1]);
                    i++;
                    break;
                case "-buildings":
                    Main.CALCULATE_BUILDINGS = Boolean.parseBoolean(args[i+1]);
                    i++;
                    break;
                case "-mountains":
                    Main.CALCULATE_MOUNTAINS = Boolean.parseBoolean(args[i+1]);
                    i++;
                    break;
                case "-shp":
                    Main.SHP_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-las2las":
                    Main.LAS_2_LAS_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-temp_file":
                    Main.TEMP_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-radius":
                    Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = Double.parseDouble(args[i+1]);
                    i++;
                    break;
                case "-natural":
                    Main.CONSIDER_EXISTING_POINTS = Boolean.parseBoolean(args[i+1]);
                    i++;
                    break;
                case "-dmr":
                    Main.DMR_FILE_NAME = args[i+1];
                    i++;
                    break;
                case "-classifications":
                    List<Integer> classifications = new ArrayList<>();
                    for (int j = i+1; !args[j].contains("-"); j++) {
                        classifications.add(Integer.parseInt(args[j]));
                        i = j;
                        if (args.length == i+1) { //prevent indexOutOfbounds in next step
                            break;
                        }
                    }
                    Main.READ_CLASSIFICATION = classifications.toArray(Integer[]::new);
                    break;
                case "-angle_similarity":
                    Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES = Double.parseDouble(args[i+1]);
                    i++;
                    break;
                case "-segments":
                    Main.MOUNTAINS_NUMBER_OF_SEGMENTS = Double.parseDouble(args[i+1]);
                    i++;
                    break;
                case "-interpolation":
                    Main.INTERPOLATION_STRING = args[i+1];
                    i++;
                    break;
            }
        }
    }

    public static void pipelineStart(){
        HelperClass.printLine(" ", "Pipeline start.");
        double[] header = JniLibraryHelpers.getHeaderInfo(INPUT_FILE_NAME);
        DTO.LasHeader headerDTO = new DTO.LasHeader(header);

        double[][] newBuildingPoints = new double[0][];
        double[][] newMountainsPoints = new double[0][];

        if (CALCULATE_BUILDINGS) {
            TimeKeeper.buildingsStartTime();
            BuildingController bc = new BuildingController(headerDTO);
            newBuildingPoints = HelperClass.toResultDoubleArray(bc.getNewPoints(), Classification.BUILDING);
            TimeKeeper.buildingsWriteStartTime();
            JniLibraryHelpers.writePointListWithClassification(newBuildingPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME+ "-building"); //temp
            TimeKeeper.buildingsWriteEndTime();
            TimeKeeper.buildingsEndTime();
        }

        double segmentLength =  (headerDTO.maxX-headerDTO.minX)/(headerDTO.pointRecordsNumber/(MAX_POINT_NUMBER_IN_MEMORY_MILLION * 1000000));
//        double segmentLength =  MAX_POINT_NUMBER_IN_MEMORY_MILLION * 1000000 / (headerDTO.pointRecordsNumber/1000)//15000;
        double absMaxX = headerDTO.maxX;
        double absMinX = headerDTO.minX;
        HelperClass.printLine(", ", "segmentLength: ", segmentLength, "absMaxX: ", absMaxX, "absMinX:", absMinX);
        double currMinX = absMinX;
        int fileCount = 1;

        if (CALCULATE_MOUNTAINS) {
            List<Point3d> newMountainsPointsList = new ArrayList<>();
            TimeKeeper.mountainsStartTime();
            while (currMinX < absMaxX) {

                //change current min and max X to that of current reduce-sized point file
                headerDTO.minX = currMinX;
                headerDTO.maxX = Double.min(currMinX + segmentLength, absMaxX); //if currMinX + segmentLength is bigger than absMaxX, use absMaxX

                String[] params = getJNIparams(currMinX, headerDTO.maxX);
                double[][] oldPoints = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, params);
//            double[][] oldPoints = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, currMinX, headerDTO.maxX);
                //JniLibraryHelpers.writePointListWithClassification(oldPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME+"-old");

                HelperClass.memory();
                MountainController mc = new MountainController(headerDTO, oldPoints);
                newMountainsPointsList.addAll(mc.getNewPoints());

                currMinX += segmentLength;
            }
            newMountainsPoints = HelperClass.toResultDoubleArray(newMountainsPointsList, Classification.GROUND);
            TimeKeeper.mountainsWriteStartTime();
            JniLibraryHelpers.writePointListWithClassification(newMountainsPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME + "-mountain"); //temp
            TimeKeeper.mountainsWriteEndTime();
            TimeKeeper.mountainsEndTime();
        }
        TimeKeeper.report();

        if (Main.CALCULATE_MOUNTAINS && Main.CALCULATE_BUILDINGS) {
            double[][] newPoints = new double[0][];
            newPoints = ArrayUtils.addAll(newPoints, newBuildingPoints.clone());
            newPoints = ArrayUtils.addAll(newPoints, newMountainsPoints.clone());
            JniLibraryHelpers.writePointListWithClassification(newPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME + "-united");
        }
        HelperClass.printLine("", "End pipeline.");
        Toolkit.getDefaultToolkit().beep(); //end program with a beep
    }

    public static String[] getJNIparams(double minX, double maxX) {
        String[] result = new String[READ_CLASSIFICATION.length + 5];
        result[0] = "dummy";
        result[1] = "-keep_x";
        result[2] = String.valueOf(minX);
        result[3] = String.valueOf(maxX);
        if (READ_CLASSIFICATION.length > 0) {
            result[4] = "-keep_class";
            int i = 5;
            for (Integer classification : READ_CLASSIFICATION) {
                result[i] = String.valueOf(classification);
                i++;
            }
        }
        return result;
    }

//    public static void writeOBJ() {
//        Point_dt[] list = MountainController.getDmrFromFile(DMR_FILE_NAME, null);
//        Delaunay_Triangulation dt = new Delaunay_Triangulation(list);
//        try { dt.write_smf("smf.obj"); } catch (Exception e) {}
//    }
//    public static double[][] testMountains(double[][] arr, double minX, double maxX, double minY, double maxY) {
//        EvenFieldController efc = new EvenFieldController( minX, maxX, minY, maxY, CREATED_POINTS_SPACING);
//        efc.interpolation = Interpolation.BICUBIC;
//        return HelperClass.toResultDoubleArray(efc.fillHoles(arr));
//    }
//
//    public static double[][] testMountains(double[][] arr) {
//        EvenFieldController efc = new EvenFieldController(arr, CREATED_POINTS_SPACING);
//        efc.interpolation = Interpolation.SPLINE;
//        return HelperClass.toResultDoubleArray(efc.fillHoles(arr));
//    }
//
//    public static double[][] testMountains(Interpolation interp) {
//        System.out.println("method testMountains()");
//        OUTPUT_FILE_NAME = DATA_FOLDER + "triglav4 spacing-"+ CREATED_POINTS_SPACING + " interpolation-"+ interp.toString()+ ".laz";
//        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
//        EvenFieldController efc = new EvenFieldController(arr, CREATED_POINTS_SPACING);
//        efc.interpolation = interp;
//        return HelperClass.toResultDoubleArray(efc.fillHoles(arr));
//    }

}
