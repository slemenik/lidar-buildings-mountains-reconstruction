package com.slemenik.lidar.reconstruction.mountains;



import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import com.slemenik.lidar.reconstruction.main.HelperClass;
import com.slemenik.lidar.reconstruction.main.Main;
import delaunay_triangulation.Point_dt;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;
import java.util.stream.Collectors;

import delaunay_triangulation.Delaunay_Triangulation;
import org.locationtech.jts.geom.Coordinate;

import static com.slemenik.lidar.reconstruction.buildings.ShpController.getBoundsFromFilename;
import static com.slemenik.lidar.reconstruction.main.Main.*;

public class MountainController {

    public String dmrFileName;
    public String outputName;
    public static double similarAngleToleranceDegrees = 10;
    public static double numberOfSegments = 5;

    public int tempStopCount = 0;
    private int tempCount = 0;
    private int tempCount2 = 0;
    private int tempColor = 0;

    private Transform3D translationCenter = new Transform3D(); // used for translating points so that center matches origin (0,0,0)
    private Transform3D translationBack = new Transform3D(); // used for translating points back to original state
    private List<double[]> pastTransformationsAngles = new ArrayList<>(); //store angles of transformations we already did,
                                                                            // list item: {aroundZToXAngle, aroundYtoZAngle}

    private List<Point3d> originalPointList = new ArrayList<>();
    private List<Point3d> transformedPointList = new ArrayList<>();
    private List<Point3d> points2write = new ArrayList<>();
    public double[][] points2writeTemp;

//    private double originalMaxX = 0;
//    private double originalMinX = 0;
//    private double originalMaxY = 0;
//    private double originalMinY = 0;
//    private double originalMaxZ = 0;
//    private double originalMinZ = 0;

    public MountainController(double[][] arr) {
//        dmrFileName = new ArrayList<>();
        setParams(arr);
    }

    /*sets class variables: all max and min values per coordiante, pointList, translation*/
    private void setParams(double[][] arr){
        double minX = Double.MAX_VALUE;
        double maxX = 0;
        double minY = Double.MAX_VALUE;
        double maxY = 0;
        double minZ = Double.MAX_VALUE;
        double maxZ = 0;

        for (double[] point : arr) {

            //fill point3d list
            Point3d p = new Point3d(point[0], point[1], point[2]);
            originalPointList.add(p);

            //set max and min
            if (point[0] > maxX) maxX = point[0];
            if (point[0] < minX) minX = point[0];

            if (point[1] > maxY) maxY = point[1];
            if (point[1] < minY) minY = point[1];

            if (point[2] > maxZ) maxZ = point[2];
            if (point[2] < minZ) minZ = point[2];
        }

//        this.originalMaxX = maxX;
//        this.originalMinX = minX;
//        this.originalMaxY = maxY;
//        this.originalMinY = minY;
//        this.originalMaxZ = maxZ;
//        this.originalMinZ = minZ;

        double centerX = (minX + maxX)/2;
        double centerY = (minY + maxY)/2;
        double centerZ = (minZ + maxZ)/2;

        translationCenter.setTranslation(new Vector3d(centerX, centerY, centerZ));
        translationBack.setTranslation(new Vector3d(-centerX, -centerY, -centerZ));
    }

    public double[][] start() {

        Point_dt[] dmrPointList = getDmrFromFile(dmrFileName);

        System.out.println("Start triangulation");
        Delaunay_Triangulation dt = new Delaunay_Triangulation(dmrPointList);
        System.out.println("End triangulation");

        dmrPointList = null; //garbage collector optimization
        //create normals
        System.out.println("Create normals");
        List<Transform3D> transformationList = new ArrayList<>();

        System.out.println(similarAngleToleranceDegrees);

        dt.trianglesIterator().forEachRemaining(triangleDt -> {
            if (!triangleDt.isHalfplane()) {
                Vector3D normal = getNormalVector(triangleDt.p1(), triangleDt.p2(), triangleDt.p3());
                //todo maybe calculate here directly instead of creating a list and then iterating it
//                normalList.add(normal);
//                if (tempCount % 2 ==0 ) temoObjectList.add(new Object());

                    //-46.30999999997357, 0.010000000009313226, 0.010000000009313226
                  Transform3D[] transformations = getRotationTransformation(normal.getX(), normal.getY(), normal.getZ());
                  if (transformations != null) { // transformations == null if we already did a transformation with similar angles //todo preglej a res pravilno izločimo nramle
//                      HelperClass.printLine(normal.getX(), normal.getZ(), normal.getZ());
//                      int a = 5/0;
                      transformationList.add(transformations[0]);
                      tempCount2++;
                  }
                  tempCount++;
            }
        });
        //all, 1998099, actual transofirn , 8523, difference, 1989576, similarAngleToleranceDegrees=10
        HelperClass.printLine(", ","all", tempCount, "actual transofirn ", tempCount2, "difference", tempCount-tempCount2);
        dt = null;

        transformationList.forEach(this::calculateNewPoints);

        return HelperClass.toResultDoubleArray(points2write);

    }

    public void calculateNewPoints(Transform3D transformation){

        TreeSet<Point3d> rotatedPointsTreeSet = new TreeSet<>((p1, p2) -> {
            if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
                return 0; //if point is exactly the same, return 0 so there are no duplicates
            } else {
                //in case of not being duplicates, order them by z
//                return Double.compare(p1.z, p2.z); //lowset to highest
                return Double.compare(p2.z, p1.z); //highest to lowest
            }
        });

        points2write.clear();//temp

//        outputName = "test ." +tempCount + ".laz";
//        System.out.println(tempCount);
        System.out.println("na zacetku je vseh tock" + originalPointList.size());
//                originalPointList.clear();
//        points2writeTemp = null;
        int i = 0;
//        points2writeTemp = new double[originalPointList.size()][3];
        double transMinX = Double.MAX_VALUE;
        double transMaxX = 0;
        double transMinY = Double.MAX_VALUE;
        double transMaxY = 0;
        double transMinZ = Double.MAX_VALUE;
        double transMaxZ = 0;
        for(Point3d originalPoint : originalPointList) {
            Point3d newPoint = new Point3d();
            transformation.transform(originalPoint, newPoint);
//                         tempPoint2 = (new double[]{originalPoint.x, originalPoint.y, originalPoint.z});
//                        points2write.add(tempPoint2);
//                        points2write.add(new double[]{newPoint.x, newPoint.y, newPoint.z});
//                points2writeTemp[i] = new double[]{newPoint.x, newPoint.y, newPoint.z};
            rotatedPointsTreeSet.add(newPoint);

            if (transMinX > newPoint.x) transMinX = newPoint.x;
            if (transMaxX < newPoint.x) transMaxX = newPoint.x;

            if (transMinY > newPoint.y) transMinY = newPoint.y;
            if (transMaxY < newPoint.y) transMaxY = newPoint.y;

            if (transMinZ > newPoint.z) transMinZ = newPoint.z;
            if (transMaxZ < newPoint.z) transMaxZ = newPoint.z;
            i++;
        }

        OUTPUT_FILE_NAME = INPUT_FILE_NAME + "+rotate";
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(rotatedPointsTreeSet), INPUT_FILE_NAME, OUTPUT_FILE_NAME, tempColor++);
//
//        JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(rotatedPointsTreeSet.stream().map(dd2->{
//            return new Point3d(dd2.x, dd2.y, 0);
//         }).collect(Collectors.toList())), INPUT_FILE_NAME, OUTPUT_FILE_NAME+".flat", tempColor++);

//        double currentMaxZDepth = transMinZ + fragmentSize; // we start in front and move to back until maxDepth
        double fragmentSize = (transMaxZ- transMinZ)/numberOfSegments;
        Double currentMaxZDepth = transMaxZ - fragmentSize; // we start at the back (1st point has highest Z, move to front (each next point has smaller z)
        Iterator<Point3d> treeSetIterator = rotatedPointsTreeSet.iterator();
        int pointNumCurrent = 0;
        int tempFileWritten = 0;
        SortedSet<Point3d> pointsCurrent;
        List<double[]> temp = new ArrayList<>();
        while (treeSetIterator.hasNext()) {
            Point3d point = treeSetIterator.next();
//            if (point.z > currentMaxZDepth) { //reached current threshold, find missing points for current segment of points
            if (point.z < currentMaxZDepth) { //reached current threshold, find missing points for current segment of points
                pointsCurrent = rotatedPointsTreeSet.headSet(point);

                String outputfileTemp = OUTPUT_FILE_NAME + ".segmentTo" + currentMaxZDepth;
//                JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(pointsCurrent), INPUT_FILE_NAME, outputfileTemp, (tempColor++));
//                JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(pointsCurrent.stream().map(dd2->{
//                    return new Point3d(dd2.x, dd2.y, 0);
//                }).collect(Collectors.toList())), INPUT_FILE_NAME, outputfileTemp + ".flat", tempColor++);


                List<double[]> missingPoints = Main.testMountains(pointsCurrent.stream().map(p -> new double[]{p.x, p.y, p.z}).toArray(double[][]::new), transMinX, transMaxX, transMinY, transMaxY);
                outputfileTemp += ".missingPoints";
//                JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(missingPoints), INPUT_FILE_NAME, outputfileTemp, (tempColor++));
//
//                JniLibraryHelpers.writePointList(HelperClass.toResultDoubleArray(missingPoints.stream().map(dd2->{
//                    return new double[]{dd2[0], dd2[1], 0};
//                }).collect(Collectors.toList())), INPUT_FILE_NAME, outputfileTemp + ".flat", tempColor++);

                temp.addAll(missingPoints);
                for (double[] p: missingPoints) {
                    //limit new added points to current segment
//                    if (p[2])
                    p[2] = Math.max(p[2], currentMaxZDepth);
//                    p[2] = Math.min(p[2], currentMaxZDepth + fragmentSize);
                }


                // todo poglej a je kul da je tam povprečje vseh točk na istmu indexu, potem probavaj z različnimi interpolacijami


                //add new points of current segment to treeSet, treat them as other points
                HelperClass.printLine(" ", "najdene nove točke: ", missingPoints.size());
                HelperClass.printLine(" ","prej je bilo točk: ", rotatedPointsTreeSet.size());
                rotatedPointsTreeSet.addAll(missingPoints.stream().map(p -> new Point3d(p[0], p[1], p[2])).collect(Collectors.toList()));
                HelperClass.printLine(" ","zdej smo jih dodali in jih je: ", rotatedPointsTreeSet.size());
                treeSetIterator = rotatedPointsTreeSet.iterator(); //reset iterator
                currentMaxZDepth -= fragmentSize;
//                currentMaxZDepth += numberOfSegments;
                System.out.println("reset iterator, we read points: " + pointNumCurrent);
                pointNumCurrent=0;
                System.out.println("-----------------------reset iteratorja----------------------------------");

            }
            pointNumCurrent++;
        }

        points2writeTemp = HelperClass.toResultDoubleArray(temp);

//        Main.testBoundary()

//                if (tempCount == 3) {//temp
//                    double[][] pointListDoubleArray = points2write.toArray(new double[][]{});
//                    System.out.println("Points to write: " + outputName);
//            JniLibraryHelpers.writePointList(points2writeTemp, INPUT_FILE_NAME, DATA_FOLDER + outputName, (tempCount % 6)+1);
//                }

        // - odstrani iz rotatedPoints tiste ki so dlje
        //       - probaj dat točke v evenFieldController, spremeni tam da se upoštevajo ustrezne koordinate: z se ne sme več ignoritat
//            System.out.println("transformation ended, get next normal");




        System.out.println("end");

    }

    /*input: x,y,z of a vector of direction we want to translate z-axis to*/
    /*output: array of two Transform3D objects, repesenting transformation to wanted angle and back to original angle*/
    public/*temp- dej na private*/ Transform3D[] getRotationTransformation(double x, double y, double z) {

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
        if (similarTransformationExists(aroundZToXAngle, aroundYtoZAngle, pastTransformationsAngles)) {
            return null;
        } else {
            pastTransformationsAngles.add(new double[]{aroundZToXAngle, aroundYtoZAngle});
        }
//        HelperClass.printLine(Math.toDegrees(aroundZToXAngle), Math.toDegrees(aroundYtoZAngle));

        //create transformation matrix based on rotation angles
        //transformation order: translate to origin (0,0,0), rotate by z-axis, rotate by y-axis, then translate back to start
        Transform3D rotationZ = new Transform3D();
        Transform3D rotationY = new Transform3D();
        Transform3D transformation = new Transform3D();
        Transform3D transformationBack = new Transform3D(); //reverse step for returning new points to original coo. system

        rotationZ.rotZ(aroundZToXAngle);
        rotationY.rotY(aroundYtoZAngle);

        transformation.mul(translationCenter);
        transformation.mul(rotationZ);
        transformation.mul(rotationY);
        transformation.mul(translationBack);

        transformationBack.mul(translationBack);
        transformationBack.mul(rotationY);
        transformationBack.mul(rotationZ);
        transformationBack.mul(translationCenter);

        return new Transform3D[]{transformation, transformationBack};
    }

    private static boolean similarTransformationExists(double potentialAngle1, double potentialAngle2, List<double[]> angleList) {
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
    public Point_dt[] getDmrFromFile(String ascFileName) {
        System.out.println("method getDmrFromFile()");
        List<Point_dt> list = new ArrayList<>();
//        int i = 0;
        Coordinate center = new Coordinate(410809.150, 137774.920, 2761.130);
        double radius = 300.0;

        try {
            Scanner scanner = new Scanner(new File(ascFileName));
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] coordinates = line.split(";");
                double x = Double.parseDouble(coordinates[0]);
                double y = Double.parseDouble(coordinates[1]);
                double z = Double.parseDouble(coordinates[2]);
//                if (center.distance3D(new Coordinate(x,y,z)) < radius) {
                    list.add(new Point_dt(x,y, z));
//                }

            }
            scanner.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.out.println("end method getDmrFromFile(), return list.length=" + list.size());
        return list.toArray(new Point_dt[0]);

    }

    public List<double[]> parseFolder(String folderPath, String lazFilename) {
        List<double[]> points2Insert = new ArrayList<>();
        int bounds[] = getBoundsFromFilename(lazFilename);
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
        return normal;
//        return getNormalAngle(normal.getX(),normal.getY(), normal.getZ());
//        return rotate(new Vector3d(normal.getX(),normal.getY(), normal.getZ()));
//        System.out.println("end getNormalVector()");
    }

}
