package com.slemenik.lidar.reconstruction.mountains;


import com.slemenik.lidar.reconstruction.buildings.ShpController;
import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import com.slemenik.lidar.reconstruction.main.DTO;
import com.slemenik.lidar.reconstruction.main.HelperClass;
import com.slemenik.lidar.reconstruction.main.Main;
import com.slemenik.lidar.reconstruction.main.TimeKeeper;
import delaunay_triangulation.Delaunay_Triangulation;
import delaunay_triangulation.Point_dt;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.locationtech.jts.geom.Coordinate;

import javax.media.j3d.Transform3D;
import javax.vecmath.GMatrix;
import javax.vecmath.GVector;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Scanner;
import java.util.SortedSet;
import java.util.TreeSet;

public class MountainController {

    public String dmrFileName;
    public String outputName;
    public double similarAngleToleranceDegrees;
    public double[][] originalPointArray;
    private double[] boundsX;

    public int tempStopCount = 0;
    private int tempCount = 0;
    private int tempCount2 = 0;
    private int tempColor = 0;
    public static boolean debug = false;

    public Transform3D transformation = new Transform3D(); //used for rotating points to desired angle
    public Transform3D transformationBack = new Transform3D(); //used for rotating points back to original view
    private Transform3D translationCenter = new Transform3D(); // used for translating points so that center matches origin (0,0,0)
    private Transform3D translationBack = new Transform3D(); // used for translating points back to original state
    private List<double[]> pastTransformationsAngles = new ArrayList<>(); //store angles of transformations we already did,
                                                                            // list item: {aroundZToXAngle, aroundYtoZAngle}


//    private double originalMaxX = 0;
//    private double originalMinX = 0;
//    private double originalMaxY = 0;
//    private double originalMinY = 0;
//    private double originalMaxZ = 0;
//    private double originalMinZ = 0;

    public MountainController(DTO.LasHeader lasHeader, double[][] oldPoints) {
//        dmrFileName = new ArrayList<>();
        double centerX = (lasHeader.minX + lasHeader.maxX)/2;
        double centerY = (lasHeader.minY + lasHeader.maxY)/2;
        double centerZ = (lasHeader.minZ +lasHeader. maxZ)/2;

        translationCenter.setTranslation(new Vector3d(centerX, centerY, centerZ));
        translationBack.setTranslation(new Vector3d(-centerX, -centerY, -centerZ));

        this.dmrFileName = Main.DMR_FILE_NAME;
        this.similarAngleToleranceDegrees = Main.MOUNTAINS_ANGLE_TOLERANCE_DEGREES;
        this.originalPointArray = oldPoints;
        this.boundsX = new double[] {lasHeader.minX, lasHeader.maxX};
    }

    public double[][] start() {
        return HelperClass.toResultDoubleArray(getNewPoints());
    }

    public TreeSet<Point3d> getNewPoints() {
//        List<Transform3D> transformationList = new ArrayList<>();

        System.out.println("similarAngleToleranceDegrees:" + similarAngleToleranceDegrees);
        TreeSet<Point3d> points2write = new TreeSet<Point3d>((p1, p2) -> {
            if (p1.distance(p2) < Main.CREATED_POINTS_SPACING) {
                return 0; //we dont add points that are very close to together
            } else {
                int compareX = Double.compare(p1.x, p2.x);
                int compareY = Double.compare(p1.y, p2.y);
                int compareZ = Double.compare(p1.z, p2.z);
                //order by X, or if same order by Y or if same order by Z
                return compareX != 0 ? compareX : ( compareY != 0 ? compareY : compareZ );
            }
        });

        Point_dt[] dmrPointList = getDmrFromFile(dmrFileName, boundsX);
        if (dmrPointList.length == 0) {
            HelperClass.printLine(" ","problem reading dmr, no points found, Add points from default angles");
            //we add points basaed on a couple of standard angles
            getDefaultAngles().stream().forEach(normal -> {
                points2write.addAll(getPoints2WriteFromNormalAngle(normal));
            });
            HelperClass.printLine(" ","Points from default angles added");
        } else {
            HelperClass.printLine(" ", "Start triangulation");
            Delaunay_Triangulation dt = new Delaunay_Triangulation(dmrPointList);
            HelperClass.printLine(" ", "End triangulation");

            //final Integer[] i = {0};
            dmrPointList = null; //garbage collector optimization
            HelperClass.printLine(" ", " add points based on DMR.");
            dt.trianglesIterator().forEachRemaining(triangleDt -> {
                if (!triangleDt.isHalfplane()) {
//                HelperClass.memory();
                    Vector3D normal = getNormalVector(triangleDt.p1(), triangleDt.p2(), triangleDt.p3());
                    points2write.addAll(getPoints2WriteFromNormalAngle(normal));
                    //System.out.println(i[0]++);
                }

            });
        }
        //all, 1998099, actual transofirn , 8523, difference, 1989576, similarAngleToleranceDegrees=10
        HelperClass.printLine(", ","all", tempCount, "actual transofirn ", tempCount2, "difference", tempCount-tempCount2);
//        dt = null;
//        transformationList.forEach(this::calculateNewPoints);

        return points2write;

    }

    private static List<Vector3D> getDefaultAngles () {
        List<Vector3D> normals = new ArrayList<>();
        for (int x = -1; x <= 1; x++ ) {
            for (int y = -1; y <= 1; y++ ) {
                for (int z = 0; z <= 1; z++ ) {
                    if ( !(+x == 0 && y == 0) ) {
                        normals.add(new Vector3D(x,y,z));
                    }
                }
            }
        }
//        normals.add(new Vector3D(1,0,0));
//        normals.add(new Vector3D(-1,0,0));
//        normals.add(new Vector3D(0,1,0));
//        normals.add(new Vector3D(0,-1,0));
//        normals.add(new Vector3D(1,0,0));
        return normals;
    }

    private List<Point3d> getPoints2WriteFromNormalAngle(Vector3D normal) {
        TimeKeeper.projectionStartTime();
        List<Point3d> points2write = new ArrayList<>();
        //-46.30999999997357, 0.010000000009313226, 0.010000000009313226
        calculateRotationTransformation(normal.getX(), normal.getY(), normal.getZ());
        if (transformation != null) { // transformation == null if we already did a transformation with similar angles //todo preglej a res pravilno izločimo nramle
//                      transformationList.add(transformation);
            System.out.println("-------------------------------------------------------------------------------------.");
            HelperClass.printLine(" ","searching for points in rotation: ", normal.getX(), normal.getY(), normal.getZ());
            List<Point3d> newPoints = getNewUntransformedPoints(transformation);
            newPoints.forEach(x->{
                transformationBack.transform(x);
                points2write.add(x);
            });
            newPoints = null;//garbage collector optimization?
//            if (tempCount2 % 10 == 0) {
                System.out.println("So far we have "+points2write.size()+" new points from " + (tempCount2+1)+" different angles");
                if (debug) {
                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(points2write), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME+ "-" +"-FinResultKota-"+normal+"-"+tempCount2);
                }
//            }
            tempCount2++;
        }
        tempCount++;
        return points2write;
    }

    /*return a list of new points that are positioned accorindg to transformation param*/
    public List<Point3d> getNewUntransformedPoints(Transform3D transformation){
        HelperClass.printLine("","function getNewUntransformedPoints()");
        TreeSet<Point3d> rotatedPointsTreeSet = new TreeSet<>((p1, p2) -> {
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

//        points2write.clear();//temp

//        outputName = "test ." +tempCount + ".laz";
//        System.out.println(tempCount);
//        System.out.println("na zacetku je vseh tock" + originalPointList.size());
//                originalPointList.clear();
//        points2writeTemp = null;
//        points2writeTemp = new double[originalPointList.size()][3];
        double transMinX = Double.MAX_VALUE;
        double transMaxX = 0;
        double transMinY = Double.MAX_VALUE;
        double transMaxY = 0;
        double transMinZ = Double.MAX_VALUE;
        double transMaxZ = 0;
        Point3d originalPoint;
        for (double[] point : originalPointArray) {
            double classification = point[3];
            originalPoint = new Point3d(point[0], point[1], point[2]);
            Point3d newPoint = new Point3d();
            transformation.transform(originalPoint, newPoint);
            rotatedPointsTreeSet.add(newPoint);

            if (transMinX > newPoint.x) transMinX = newPoint.x;
            if (transMaxX < newPoint.x) transMaxX = newPoint.x;

            if (transMinY > newPoint.y) transMinY = newPoint.y;
            if (transMaxY < newPoint.y) transMaxY = newPoint.y;

            if (transMinZ > newPoint.z) transMinZ = newPoint.z;
            if (transMaxZ < newPoint.z) transMaxZ = newPoint.z;
        }

        Main.OUTPUT_FILE_NAME = Main.INPUT_FILE_NAME + "+rotate";
        if (debug) JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(rotatedPointsTreeSet), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME, tempColor++);
//        if (true){//temp
//            return new ArrayList<>(rotatedPointsTreeSet);
//        }
//
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(rotatedPointsTreeSet.stream().map(dd2->{
//            return new Point3d(dd2.x, dd2.y, 0);
//         }).collect(Collectors.toList())), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME+".flat");

        double fragmentSize = (transMaxZ- transMinZ)/Main.MOUNTAINS_NUMBER_OF_SEGMENTS;
//        double currentMaxZDepth = transMinZ + fragmentSize; // we start in front and move to back until maxDepth
        double currentMaxZDepth = transMaxZ - fragmentSize; // we start at the back (1st point has highest Z, move to front (each next point has smaller z)
        Iterator<Point3d> treeSetIterator = rotatedPointsTreeSet.iterator();
        int pointNumCurrent = 0;
        int tempFileWritten = 0;
        SortedSet<Point3d> pointsCurrent;
        List<Point3d> addedUntransformedPointsList = new ArrayList<>(); //added points that are still in rotated form, before putting them in original coo. system
//        Point3d tempPoint = null;
//        Point3d tempPoint2 = null;
        while (treeSetIterator.hasNext()) {
            Point3d point = treeSetIterator.next();
//            tempPoint2 = point;
//            if (point.z > currentMaxZDepth) { //reached current threshold, find missing points for current segment of points
            if (point.z <= currentMaxZDepth) { //reached current threshold, find missing points for current segment of points
                pointsCurrent = rotatedPointsTreeSet.headSet(point);

                List<Point3d> missingPoints;
                if (pointsCurrent.size() < 6) {
                    missingPoints = new ArrayList<>(); //if point number is so small, it makes no sense to calculate anything
                } else {
                    missingPoints = getMissingPointsFromExisting(pointsCurrent, transMinX, transMaxX, transMinY, transMaxY); //todo mogoče namesto new ArrayList daš kar direkt Set in nato popraviš ostalo ustrezno
                }

                if (debug) {
                    String outputfileTemp = Main.OUTPUT_FILE_NAME + ".segmentTo" + currentMaxZDepth;
//                    SortedSet<Point3d> temp123 = tempPoint == null ? rotatedPointsTreeSet.headSet(point) : rotatedPointsTreeSet.subSet(tempPoint, point);
//                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(temp123), Main.INPUT_FILE_NAME, outputfileTemp+"123");
                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(pointsCurrent), Main.INPUT_FILE_NAME, outputfileTemp);
//                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(pointsCurrent.stream().map(dd2 -> new Point3d(dd2.x, dd2.y, 0)).collect(Collectors.toList())), Main.INPUT_FILE_NAME, outputfileTemp + ".flat");
                    outputfileTemp += ".missingPoints";
//                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(missingPoints), Main.INPUT_FILE_NAME, outputfileTemp);
//                    JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(missingPoints.stream().map(dd2 -> new Point3d(dd2.x, dd2.y, 0)).collect(Collectors.toList())), Main.INPUT_FILE_NAME, outputfileTemp + ".flat");

                    if (pointsCurrent.size() == 2271442) {
                        System.exit(-222);

                    }
                }
                TimeKeeper.projectionStartTime(); //ponovno naredimo saj se bo v zanki potem kasenje ponovno klical projectionEndTime()
                addedUntransformedPointsList.addAll(missingPoints);

                //add new points of current segment to treeSet, treat them as other points
                HelperClass.printLine(" ", "najdene nove točke: ", missingPoints.size());
                rotatedPointsTreeSet.addAll(missingPoints);
                treeSetIterator = rotatedPointsTreeSet.iterator(); //reset iterator
                currentMaxZDepth -= fragmentSize;
//                tempPoint = point;
//                currentMaxZDepth += numberOfSegments; //uncomment this, and comment upper if we go from front to back Z coordinate
                pointNumCurrent=0;
//                System.out.println("-----------------------reset iterator----------------------------------");

            }
            pointNumCurrent++;
        }
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(rotatedPointsTreeSet.subSet(tempPoint, tempPoint2)), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME+"1234");
        rotatedPointsTreeSet.clear();
        rotatedPointsTreeSet = null;
        System.out.println("end calculateNewPoints(), we found " + addedUntransformedPointsList.size() + " new points.");
        System.out.println("-------------------------------------------------------------------------------------.");
        if (debug) JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(addedUntransformedPointsList), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME+ "untrans");
        return addedUntransformedPointsList;
    }

    private List<Point3d> getMissingPointsFromExisting(SortedSet<Point3d> existingPoints, double minX, double maxX, double minY, double maxY) {
        EvenFieldController efc = new EvenFieldController( minX, maxX, minY, maxY, Main.CREATED_POINTS_SPACING);

        return efc.fillHoles(existingPoints);
    }

    /*input: x,y,z of a vector of direction we want to translate z-axis to*/
    /*output: array of two Transform3D objects, repesenting transformation to wanted angle and back to original angle*/
    public/*temp- dej na private*/ void calculateRotationTransformation(double x, double y, double z) {

        //calculate two angles: 1. from current direcion to x-axis (rotate around z-axis)
        //                      2. from x-axis to z-axid (rotate around y-axis)
        // http://www.fundza.com/mel/axis_to_vector/index.html
        double xyLength = Math.sqrt(x * x + y * y);
        double aroundZToXAngle = Math.acos(x / xyLength); //radians
        if (y > 0) {
            aroundZToXAngle *= -1; //rotate backwards to x
            //(Math.abs(y)/-y);
        }

        double xyzLength = Math.sqrt(x*x + y*y + z*z); //normal length
        double aroundYtoZAngle = Math.asin(xyLength/xyzLength); // z cannot be < 0; direction always + (from +x to +z)

        //check if similiar transformation was alredy done
        if (similarTransformationExists(aroundZToXAngle, aroundYtoZAngle, pastTransformationsAngles, similarAngleToleranceDegrees)) {
            transformation = null;
            return;
        } else {
            pastTransformationsAngles.add(new double[]{aroundZToXAngle, aroundYtoZAngle});
        }
//        HelperClass.printLine(Math.toDegrees(aroundZToXAngle), Math.toDegrees(aroundYtoZAngle));


        boolean temp = false;
        if (temp) {


            //create transformation matrix based on rotation angles
            //transformation order: translate to origin (0,0,0), rotate by z-axis, rotate by y-axis, then translate back to start
            Transform3D rotationZ = new Transform3D();
            Transform3D rotationY = new Transform3D();
            transformation = new Transform3D();
            transformationBack = new Transform3D();

            rotationZ.rotZ(aroundZToXAngle);
            rotationY.rotY(aroundYtoZAngle);

            transformation.mul(translationCenter);
            transformation.mul(rotationZ);
            transformation.mul(rotationY);
            transformation.mul(translationBack);

            rotationY.invert();
            rotationZ.invert();
            transformationBack.mul(translationCenter);
            transformationBack.mul(rotationY);
            transformationBack.mul(rotationZ);
            transformationBack.mul(translationBack);
        } else {

            Transform3D rotation = new Transform3D();
            transformation = new Transform3D();
            transformationBack = new Transform3D();


            //https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/2672702#2672702
            GVector a = new GVector(new double[]{x,y,z});
            GVector b = new GVector(new double[]{0,0,1}) ; //we want to transform vector a to vector b

            GVector sum = new GVector(3);
            sum.add(a, b);

            GMatrix upper = new GMatrix(3,3);
            upper.mul(sum, sum);

            GMatrix lower = new GMatrix(1,1);
            GMatrix sumMatrix  = new GMatrix(3,1, new double[]{sum.getElement(0), sum.getElement(1), sum.getElement(2)});
            lower.mulTransposeLeft(sumMatrix, sumMatrix);
            double lowerDouble = lower.getElement(0,0);
            double quotient = 2/lowerDouble;

            Matrix3d matrix = new Matrix3d();
            upper.get(matrix);
            matrix.mul(quotient);


            Matrix3d identity = new Matrix3d();
            identity.setIdentity();
            matrix.sub(identity);

            rotation.setRotation(matrix);


//            rotationX.rotX(-90);

            transformation.mul(translationCenter);
            transformation.mul(rotation);
            transformation.mul(translationBack);

//            transformation.setRotation(new Matrix3d(-1, 0,0, 0,0,-1, 0,-1,0));

            rotation.invert();
            transformationBack.mul(translationCenter);
            transformationBack.mul(rotation);
            transformationBack.mul(translationBack);
        }
    }

    private static boolean similarTransformationExists(double potentialAngle1, double potentialAngle2, List<double[]> angleList, double similarAngleToleranceDegrees) {
        for (double[] angles : angleList) {
            double existingAngle1 = angles[0];
            double existingAngle2 = angles[1];
            //check if angle we want to translate to is close to one of existing angles; close= somehwhere before or after existing angle (+/- tolerance)
            if (HelperClass.isBetween(potentialAngle1, existingAngle1-Math.toRadians(similarAngleToleranceDegrees), existingAngle1+Math.toRadians(similarAngleToleranceDegrees))
                    && HelperClass.isBetween(potentialAngle2, existingAngle2-Math.toRadians(similarAngleToleranceDegrees), existingAngle2+Math.toRadians(similarAngleToleranceDegrees))
            ){
                return true;
            }
        }
        return false;
    }

    /*returns array of object Point_dt with coordinates from DMR file*/
    /*used for triangulation*/
    public static Point_dt[] getDmrFromFile(String ascFileName, double[] bounds) {
        System.out.println("method getDmrFromFile()");
        List<Point_dt> list = new ArrayList<>();
        int i = 0;
        int j = 0;
        Coordinate center = new Coordinate(410809.150, 137774.920, 2761.130);
        double radius = 300.0;

        try {
            InputStream is = new FileInputStream(ascFileName);
            BufferedReader br=new BufferedReader(new InputStreamReader(is));
            for (String line = br.readLine(); line != null; line = br.readLine()) {
                String[] coordinates = line.split(";");
                double x = Double.parseDouble(coordinates[0]);
                double y = Double.parseDouble(coordinates[1]);
                double z = Double.parseDouble(coordinates[2]);
//                if (center.distance3D(new Coordinate(x,y,z)) < radius) {
                if (bounds == null || (bounds[0] <= x && x <= bounds[1])) {
                    i++;
                    list.add(new Point_dt(x,y, z)); //if no bounds are given or x is inside bounds we write to list
                }

                j++;
//                }

            }
            br.close();
            is.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        HelperClass.printLine(", ", "i: ", i, "j: ", j);
        System.out.println("end method getDmrFromFile(), return list.length=" + list.size());
        return list.toArray(new Point_dt[0]);

    }


    public List<double[]> parseFolder(String folderPath, String lazFilename) {
        List<double[]> points2Insert = new ArrayList<>();
        double bounds[] = ShpController.getBoundsFromFilename(lazFilename);
        File fileList[] = new File(folderPath).listFiles();
        for(File file : fileList) {
            System.out.println("File z imenom:" + file.getName());
            try {
                Scanner scanner = new Scanner(file);
                while (scanner.hasNextLine()) {
                    String line = scanner.nextLine();
                    String coordinates[] = line.split("\\s+");
                    double x = Double.parseDouble(coordinates[0]);
                    double y = Double.parseDouble(coordinates[1]);
                    double z = Double.parseDouble(coordinates[2]);

//                    if (bounds[0] <= x && bounds[1] <= y && bounds[2] >= x && bounds[3] >= y) {
                        points2Insert.add(new double[]{x, y, z});
//                    }
                    if (points2Insert.size() > 16 * 1000000) {
                        System.out.println("too much");
                        return points2Insert;
                    }
                }
                scanner.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
            System.out.println("trenutno tcck:" + points2Insert.size());
        }
        return points2Insert;
    }

    //http://www.java-gaming.org/topics/reopened-calculating-normal-vectors/33838/view.html
    //https://www.khronos.org/opengl/wiki/Calculating_a_Surface_Normal
    private static Vector3D getNormalVector(Point_dt p1, Point_dt p2, Point_dt p3){
//        System.out.println("method getNormalVector()");
//
//        System.out.println(p1.toString());
//        System.out.println(p2.toString());
//        System.out.println(p3.toString());

        Vector3D vectorP1 = new Vector3D(p1.x(), p1.y(), p1.z());
        Vector3D vectorP2 = new Vector3D(p2.x(), p2.y(), p2.z());
        Vector3D vectorP3 = new Vector3D(p3.x(), p3.y(), p3.z());

        Vector3D U = vectorP2.subtract(vectorP1);
        Vector3D V = vectorP3.subtract(vectorP1);

        Vector3D normal = U.crossProduct(V);

        if (normal.getZ() < 0) { // if normal points down, change direction of normal
            normal = V.crossProduct(U);
        }

//        System.out.println("normal vector: " + normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
        return normal.normalize();
//        return getNormalAngle(normal.getX(),normal.getY(), normal.getZ());
//        return rotate(new Vector3d(normal.getX(),normal.getY(), normal.getZ()));
//        System.out.println("end getNormalVector()");
    }

}
