package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.buildings.ColorController;
import com.slemenik.lidar.reconstruction.buildings.ShpController;
import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import java.util.Arrays;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;

import com.slemenik.lidar.reconstruction.buildings.BuildingController;
import com.slemenik.lidar.reconstruction.mountains.EvenFieldController;
import com.slemenik.lidar.reconstruction.mountains.InterpolationController.Interpolation;

import com.slemenik.lidar.reconstruction.mountains.MountainController;
import com.slemenik.lidar.reconstruction.mountains.triangulation.Triangulation;
import delaunay_triangulation.Delaunay_Triangulation;
import delaunay_triangulation.Point_dt;


import javax.vecmath.Point3d;


public class Main {

    public static final String DATA_FOLDER = ".";

    private static boolean CALCULATE_BUILDINGS = false;
    private static boolean CALCULATE_MOUNTAINS = true;

    public static final String INPUT_FILE_NAME = DATA_FOLDER + "GK_410_137_triglav_okrnjen";//"GK_431_136_bled_grad";//"GK_410_137_triglav_okrnjen";//"GK_458_109";//"410_137_triglav";//"original+odsek";//"triglav okrnjen.laz";
    public static String OUTPUT_FILE_NAME = DATA_FOLDER + "out";
    public static final String TEMP_FILE_NAME = DATA_FOLDER + "temp";
    public static final String DMR_FILE_NAME = INPUT_FILE_NAME.substring(0, DATA_FOLDER.length() + 10) + ".asc";//DATA_FOLDER + "GK1_458_109.asc";//"GK1_410_137.asc";
    public static final String SHP_FILE_NAME = DATA_FOLDER + "/stavbe/BU_STAVBE_P.shp";
    private static final double MAX_POINT_NUMBER_IN_MEMORY_MILLION = 1; // največje število točk originalne datoteke, ki ga še preberemo v pomnilnik
                                                                      // če je točk več, se razdeli v več ločenih branj
    private static final int SEGMENT_LENGTH_X_COORDINATE_LAS_FILE = 5; // the difference between min and max X coordinate,
                                                                        // used for limiting read points so we read them
                                                                        // segment by segments. By default the full file
                                                                        // has a range of 1000, ex. 410000-411000
                                                                        //~15mio points , range of 1 ~15k points

    public static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
    public static final double CREATED_POINTS_SPACING = 0.6;//2.0;//0.2;
    public static final boolean CONSIDER_EXISTING_POINTS = false; //rešetke
    public static final double BOUNDING_BOX_FACTOR = 1.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    public static final boolean CREATE_TEMP_FILE = true;
    //public static final double[] TEMP_BOUNDS = new double[]{462264, 100575, 462411, 100701};
    private static Classification[] READ_CLASSIFICATION = Arrays.asList(Classification.values()).subList(0,8).toArray(Classification[]::new);
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

    public static final double MOUNTAINS_ANGLE_TOLERANCE_DEGREES = 30;
    public static final double MOUNTAINS_NUMBER_OF_SEGMENTS = 3;

    public static Interpolation INTERPOLATION = Interpolation.AVERAGE_8N;

    public static void main(String[] args) {
        long startTime = System.nanoTime();
        System.out.println("start main");

        long heapMaxSize = Runtime.getRuntime().maxMemory();
        System.out.println("heapmaxsize "+HelperClass.formatHeapSize(heapMaxSize));

        tempTestFunction();
        double[][] pointListDoubleArray = new double[][]{};
//        double[][] pointListDoubleArray = mainTest(args);

        if (pointListDoubleArray.length > 0) {
            System.out.println("zacetek pisanja... ");
            System.out.println("Points to write: " + pointListDoubleArray.length);
            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME);
            System.out.println("End writing. Points written: " + returnValue);

            ////////temp////////////7
//            if (args.length > 0) {
                int a = args.length+1;
                System.out.println("jovo na novo");
                main(new String[a]);


//            }
            ////////temp////////////7

        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Minute izvajanja: " + time/60);
        System.out.println("end");
    }

    public static void pipelineStart(){
        HelperClass.printLine(" ", "Pipeline start.");
        double[] header = JniLibraryHelpers.getHeaderInfo(INPUT_FILE_NAME);
        DTO.LasHeader headerDTO = new DTO.LasHeader(header);

        double[][] newPoints = new double[0][];

        double segmentLength =  (headerDTO.maxX-headerDTO.minX)/(headerDTO.pointRecordsNumber/(MAX_POINT_NUMBER_IN_MEMORY_MILLION * 1000000));
//        double segmentLength =  MAX_POINT_NUMBER_IN_MEMORY_MILLION * 1000000 / (headerDTO.pointRecordsNumber/1000)//15000;
        double absMaxX = headerDTO.maxX;
        double absMinX = headerDTO.minX;
        HelperClass.printLine(", ", "segmentLength: ", segmentLength, "absMaxX: ", absMaxX, "absMinX:", absMinX);
        double currMinX = absMinX;
        //split full file to multiple files, read and write one by one
        while (currMinX < absMaxX) {

            //change current min and max X to that of current reduce-sized point file
            headerDTO.minX = currMinX;
            headerDTO.maxX = Double.min(currMinX + segmentLength, absMaxX); //if currMinX + segmentLength is bigger than absMaxX, use absMaxX

            if (CALCULATE_BUILDINGS) {
                BuildingController bc = new BuildingController(headerDTO);
                double[][] newBuildingPoints = HelperClass.toResultDoubleArray(bc.getNewPoints(), Classification.BUILDING);
                JniLibraryHelpers.writePointListWithClassification(newBuildingPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME+ "-building"); //temp
            }

            if (CALCULATE_MOUNTAINS) {
                String[] params = getJNIparams(currMinX, headerDTO.maxX);
                double[][] oldPoints = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, params);
//            double[][] oldPoints = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, currMinX, headerDTO.maxX);
                //JniLibraryHelpers.writePointListWithClassification(oldPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME+"-old");

                HelperClass.memory();
                MountainController mc = new MountainController(headerDTO, oldPoints);
                double[][] newMountainsPoints = HelperClass.toResultDoubleArray(mc.getNewPoints(), Classification.GROUND);
                JniLibraryHelpers.writePointListWithClassification(newMountainsPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME + "-mountain"); //temp
            }

            //newPoints = ArrayUtils.addAll(newBuildingPoints, newMountainsPoints);

            currMinX += segmentLength;
        }
        JniLibraryHelpers.writePointListWithClassification(newPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME + "-united");
        HelperClass.printLine("", "End pipeline.");
    }

    private static void examplePoint2Index() {

        String input = DATA_FOLDER + "gore presek/triglav okrnjen - Copy";
        String[] params = getJNIparams(410000, 411000);
        double[][] oldPoints = JniLibraryHelpers.getPointArray(input, params);
//            double[][] oldPoints = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, currMinX, headerDTO.maxX);
        //JniLibraryHelpers.writePointListWithClassification(oldPoints, INPUT_FILE_NAME, OUTPUT_FILE_NAME+"-old");
        double[] header = JniLibraryHelpers.getHeaderInfo(input);
        DTO.LasHeader headerDTO = new DTO.LasHeader(header);
        MountainController mc = new MountainController(headerDTO, oldPoints);
        mc.calculateRotationTransformation(0, -1, 0);


        double transMinX = Double.MAX_VALUE;
        double transMaxX = 0;
        double transMinY = Double.MAX_VALUE;
        double transMaxY = 0;
        double transMinZ = Double.MAX_VALUE;
        double transMaxZ = 0;
        SortedSet<Point3d> pointList = new TreeSet<>((p1, p2) -> {
            int compareX = Double.compare(p1.x, p2.x);
            int compareY = Double.compare(p1.y, p2.y);
            // int compareZ = Double.compare(p1.z, p2.z); //lowset to highest
            int compareZ = Double.compare(p2.z, p1.z); //highest to lowest
            if (compareX == 0 && compareY == 0 && compareZ == 0 ) {
                return 0; //if point is exactly the same, return 0 so there are no duplicates
            } else {
                if (Double.compare(p2.z, p1.z) == 0){ //temp
                    int tempa = 35;
                }
                int temp = compareZ != 0 ? compareZ : ( compareX != 0 ? compareX : compareY );
                if (temp == 0){
                    int tempb = 24;
                }
                //in case of not being duplicates, order them by Z
                //or if Z is same order by X or if same order by Y
                return compareZ != 0 ? compareZ : ( compareX != 0 ? compareX : compareY );
            }
        });
        Point3d originalPoint;
        for (double[] point : mc.originalPointArray) {
            originalPoint = new Point3d(point[0], point[1], point[2]);
            Point3d newPoint = new Point3d();
            mc.transformation.transform(originalPoint, newPoint);
            newPoint.z = 0;
            pointList.add(newPoint);

            if (transMinX > newPoint.x) transMinX = newPoint.x;
            if (transMaxX < newPoint.x) transMaxX = newPoint.x;

            if (transMinY > newPoint.y) transMinY = newPoint.y;
            if (transMaxY < newPoint.y) transMaxY = newPoint.y;
        }

        EvenFieldController efc = new EvenFieldController( transMinX, transMaxX, transMinY, transMaxY, Main.CREATED_POINTS_SPACING);
        boolean[][] fieldAllPoints = efc.getBooleanPointField(pointList);
        HelperClass.createFieldPointFile(fieldAllPoints, transMinX, transMinY, CREATED_POINTS_SPACING);
        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(pointList), input, OUTPUT_FILE_NAME + "orig");

    }



    public static String[] getJNIparams(double minX, double maxX) {
        String[] result = new String[READ_CLASSIFICATION.length + 5];
        result[0] = "dummy";
        result[1] = "-keep_x";
        result[2] = String.valueOf(minX);
        result[3] = String.valueOf(maxX);
        result[4] = "-keep_class";
        for (int i = 5; i < result.length; i++) {
            result[i] = String.valueOf(READ_CLASSIFICATION[i-5].getIntValue());
        }
        return result;
    }

    public static void tempTestFunction() {

//        public static int point2Index(double coordinate, double min, double pointSpace) {//temp spremeni v private
//            return (int) ((coordinate - min) / pointSpace);
//        }
//
//        public static double index2Point(int x, double min, double pointSpace) {//temp spremeni v private
//            return ((double) x) * pointSpace + min;
//        }

//        System.out.println(EvenFieldController.point2Index(410839.68766673, 410531.88766673254, 0.6));
//        double min = 410531.88766673254;
//        System.out.println(EvenFieldController.point2Index(410839.6876667325, min, 0.6));
//        double coordinate = 410839.6876667325;
//
//        double pointSpace = 0.6;
//        double temp = (coordinate - min) / pointSpace;
//
//        System.out.println(EvenFieldController.index2Point(512, min, 0.6));
//        System.out.println(EvenFieldController.index2Point(513, min, 0.6));
//        double[][] a = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
//        List<double[]> c0 = new ArrayList<>();
//        List<double[]> c1 = new ArrayList<>();
//        List<double[]> c2 = new ArrayList<>();
//        List<double[]> c3 = new ArrayList<>();
//        List<double[]> c4 = new ArrayList<>();
//        List<double[]> c5 = new ArrayList<>();
//        List<double[]> c6 = new ArrayList<>();
//        List<double[]> c7 = new ArrayList<>();
//        List<double[]> c8 = new ArrayList<>();
//        for (double[] b: a
//             ) {
//            switch ((int)b[3]) {
//                case 0: c0.add(b); break;
//                case 1: c1.add(b); break;
//                case 2: c2.add(b); break;
//                case 3: c3.add(b); break;
//                case 4: c4.add(b); break;
//                case 5: c5.add(b); break;
//                case 6: c6.add(b); break;
//                case 7: c7.add(b); break;
//                case 8: c8.add(b); break;
//                default: System.out.println(b[3]);
//            }
//        }
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c0), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 0);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c1), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 1);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c2), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 2);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c3), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 3);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c4), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 4);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c5), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 5);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c6), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 6);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c7), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 7);
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(c8), INPUT_FILE_NAME, OUTPUT_FILE_NAME + 8);

        String params[] = new String[]{"dummy", "-keep_x", "410500", "410600", "-keep_class", "0", "1", "2", "3", "4" };
//        double[][] t = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME, params);
//        JniLibraryHelpers.writePointListWithClassification(t, INPUT_FILE_NAME, OUTPUT_FILE_NAME + 8);

        int i = 1;
//        for (Interpolation interpolation: Interpolation.values()) {
//            if (i++ <= 4) continue; //napiši kolik ohočše da se jih spusti
            INTERPOLATION = Interpolation.BICUBIC;
            OUTPUT_FILE_NAME = DATA_FOLDER + "out-" + INTERPOLATION;
            pipelineStart();
//        }

//        examplePoint2Index();
//        test123();
    return;


    }

    private static void test123() {
        String input = DATA_FOLDER + "primeri za nalogo/notranjost3";
        double[][] arr = JniLibraryHelpers.getPointArray(input);
        double[] lasHeaderParams =JniLibraryHelpers.getHeaderInfo(input);
        DTO.LasHeader header = new DTO.LasHeader(lasHeaderParams);
        boolean[][] newArr = EvenFieldController.initField(header.minX, header.maxX, header.minY, header.maxY, 0.6);

        for (int i = 0; i < newArr.length; i++) {
            Arrays.fill(newArr[i], true);
        }
        HelperClass.createFieldPointFile(newArr, header.minX, header.minY, 0.6);
        int a = 5;




    }

    private static double[][] mainTest(String[] args) {

                OUTPUT_FILE_NAME = DATA_FOLDER + "nevem.laz";

//        List<double[]> arr2 = new MountainController().parseFolder("C:/Users/Matej/Documents/My Documents/Šola/4. FRI/Magistrska/DMV1000", DATA_FOLDER + "410_137_triglav.laz");


//        List<double[]> arr2 = testMountains(Interpolation.SPLINE);
//        List<double[]> arr2 = testBuildingCreation();
//        List<double[]> arr2 = ArrayList<double[]>(Arrays.asList(arr));
        double[][] arr2 = new double[][]{};
//        testHeapSpace();
        if (args.length <= 0) {
            arr2 = testMountainController(args);
        }
        return arr2;

    }

    public static void testHeapSpace(){

//        double[][] arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
//        double[][] arr2 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
//        double[][] arr3 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
//        double[][] arr4 = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);

    }

    public static void writeOBJ() {
        Point_dt[] list = MountainController.getDmrFromFile(DMR_FILE_NAME, null);
        Delaunay_Triangulation dt = new Delaunay_Triangulation(list);
        try { dt.write_smf("smf.obj"); } catch (Exception e) {}
    }


    public static double[][] testMountainController(String[] args) {
        HelperClass.memory();
        double[] lasHeaderParams = JniLibraryHelpers.getHeaderInfo((INPUT_FILE_NAME));
        MountainController mc = new MountainController(new DTO.LasHeader(lasHeaderParams), null);

        mc.originalPointArray = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        HelperClass.memory();

        mc.dmrFileName = DMR_FILE_NAME;
        mc.tempStopCount = args.length;
        mc.similarAngleToleranceDegrees = MOUNTAINS_ANGLE_TOLERANCE_DEGREES;
//        MountainController.numberOfSegments = 25; //temp - 19 is infinte loop

        double[][] newPoints;
        boolean allNormals = true;
        if (allNormals) {
            newPoints = mc.start();
        } else { //temp, for testing only
//            Transform3D transformation = mc.getRotationTransformation(-46.30999999997357, 0.010000000009313226, 0.010000000009313226); //standard test example
//            mc.calculateRotationTransformation(0.6900000000000546, -0.01999999999998181, 1.0);
//            mc.calculateRotationTransformation(-1, -1, 0.0);
            mc.calculateRotationTransformation(-1, -1, 1.0); //trakovi so še zmerej tu, glej takoj prvi sloj, DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
                                                                                        //public static final double CREATED_POINTS_SPACING = 0.6;
                                                                                        //public static double similarAngleToleranceDegrees = 10;
                                                                                        //public static double numberOfSegments = 15;
                                                                                            //edini možen popravek je drugačen način odkrivanja borderja, da odkrije pač vse
            List<Point3d> newUntransformedPoints = mc.getNewUntransformedPoints(mc.transformation);
            newPoints = new double[newUntransformedPoints.size()][3];

            for(int i = 0; i<newUntransformedPoints.size(); i++) {
                Point3d point = newUntransformedPoints.get(i);
                mc.transformationBack.transform(point);
                newPoints[i] = new double[]{point.x, point.y, point.z};
            }
        }
        ////////
        OUTPUT_FILE_NAME = DATA_FOLDER + "allTheAddedPoints.rotatedback";
        return newPoints;
    }

    //spodnja funckija getPointsFromFieldArray je napačna, prej popravi če hočeš tole klicat
    public static List<double[]> testMountainGrid3d(){
        //naredi triglav, 3d, ne naredi praznih mest, ampak tista ki so že nafilana, ampak so x,y koordinate diskretne, mreža
        double[][]  arr = JniLibraryHelpers.getPointArray(INPUT_FILE_NAME);
        OUTPUT_FILE_NAME = DATA_FOLDER + "triglav grid diskretno.laz";
        EvenFieldController ef = new EvenFieldController(arr, CREATED_POINTS_SPACING);
        //popravi tole Main.INTERPOLATION = Interpolation.OWN_VALUE;
        return ef.getPointsFromFieldArray(ef.getBooleanPointField(arr), true);
    }

    public static List<double[]> testBoundary() {
        double[][] arr = JniLibraryHelpers.getPointArray( INPUT_FILE_NAME);
        return testBoundary(arr);
    }
    //spodnja funckija getPointsFromFieldArray je napačna, prej popravi če hočeš tole klicat
    public static List<double[]> testBoundary(double[][] arr) {

        EvenFieldController mc = new EvenFieldController(arr, CREATED_POINTS_SPACING);
        boolean[][] field = mc.getBooleanPointField(arr);
        boolean[][] newField = mc.getBoundaryField(field);
//        boolean[][] newField = field;

        return mc.getPointsFromFieldArray(newField, true);

    }

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
        bc.write(ShpController.getBoundsFromFilename(INPUT_FILE_NAME));
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
