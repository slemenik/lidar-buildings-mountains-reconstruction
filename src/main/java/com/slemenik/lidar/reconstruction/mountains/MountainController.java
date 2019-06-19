package com.slemenik.lidar.reconstruction.mountains;



import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import com.slemenik.lidar.reconstruction.main.HelperClass;
import delaunay_triangulation.Point_dt;
import delaunay_triangulation.Triangle_dt;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

import delaunay_triangulation.Delaunay_Triangulation;
import org.locationtech.jts.geom.Coordinate;

import static com.slemenik.lidar.reconstruction.buildings.ShpController.getBoundsFromFilename;
import static com.slemenik.lidar.reconstruction.main.Main.DATA_FOLDER;
import static com.slemenik.lidar.reconstruction.main.Main.INPUT_FILE_NAME;

public class MountainController {

    public String dmrFileName;
    public String outputName;

    public int stopCount = 0;
    private int tempCount = 0;

    private Transform3D translationCenter = new Transform3D(); // used for translating points so that center matches origin (0,0,0)
    private Transform3D translationBack = new Transform3D(); // used for translating points back to original state
    private List<double[]> pastTransformationsAngles = new ArrayList<>();

    private List<Point3d> originalPointList = new ArrayList<>();
    private List<Point3d> transformedPointList = new ArrayList<>();
    private List<Point3d> points2write = new ArrayList<>();
    private double[][] points2writeTemp;

    public MountainController(double[][] arr) {
//        dmrFileName = new ArrayList<>();
        setPointListAndTranslationCenter(arr);
    }

    private void setPointListAndTranslationCenter(double[][] arr){
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
        List<Vector3D> normalList = new ArrayList<>();

        dt.trianglesIterator().forEachRemaining(triangleDt -> {
            if (!triangleDt.isHalfplane()) {
                Vector3D normal = getNormalVector(triangleDt.p1(), triangleDt.p2(), triangleDt.p3());
                //todo maybe calculate here directly instead of creating a list and then iterating it
//                normalList.add(normal);
//                if (tempCount % 2 ==0 ) temoObjectList.add(new Object());
                  Transform3D transformation = getRotationTransformation(normal.getX(), normal.getY(), normal.getZ());
                  calculateNewPoints(transformation);

                  tempCount++;
            }
        });
        dt = null;
        return HelperClass.toResultDoubleArray(points2write);

    }

    public void calculateNewPoints(Transform3D transformation){

//
                PriorityQueue<Point3d> rotatedPointsQueue = new PriorityQueue<>(Comparator.comparingDouble(p -> p.z));
//            if (normal.getZ() < 0.9 && normal.getZ() > 0.1) {
//                System.out.println("transform points by normal:" + normal.getX() + ", " + normal.getY() + ", " + normal.getZ() + " still to go: " + (normalList.size() - count.get(0)));
//            }

                points2write.clear();//temp

                outputName = "test ." +tempCount + ".laz";
                System.out.println(tempCount);
                System.out.println(originalPointList.size());
//                originalPointList.clear();
                points2writeTemp = null;
                points2writeTemp = new double[15664254][3];
                int i = 0;

                try {
                    for(Point3d originalPoint : originalPointList) {
                        Point3d newPoint = new Point3d();
                        transformation.transform(originalPoint, newPoint);
//                    rotatedPointsQueue.add(newPoint);
//                         tempPoint2 = (new double[]{originalPoint.x, originalPoint.y, originalPoint.z});
//                        points2write.add(tempPoint2);
//                        points2write.add(new double[]{newPoint.x, newPoint.y, newPoint.z});
                        points2writeTemp[i] = new double[]{newPoint.x, newPoint.y, newPoint.z};
                        i++;
                    }
                } catch (OutOfMemoryError error) {
                    System.out.println(error);
                    System.out.println(i);
                    return;
                }

//                while (!rotatedPointsQueue.isEmpty()) {
//                    System.out.println(rotatedPointsQueue.remove());
//                }

//                if (tempCount == 3) {//temp
//                    double[][] pointListDoubleArray = points2write.toArray(new double[][]{});
//                    System.out.println("Points to write: " + outputName);
                    JniLibraryHelpers.writePointList(points2writeTemp, INPUT_FILE_NAME, DATA_FOLDER + outputName, (tempCount % 6)+1);
//                }

                // - odstrani iz rotatedPoints tiste ki so dlje
                //       - probaj dat točke v evenFieldController, spremeni tam da se upoštevajo ustrezne koordinate: z se ne sme več ignoritat
//            System.out.println("transformation ended, get next normal");




        System.out.println("end");

    }

    /*input: x,y,z of a vector of direction we want to translate z-axis to*/
    private Transform3D getRotationTransformation(double x, double y, double z) {

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

//        System.out.println("koti " + aroundZToXAngle + " " + aroundYtoZAngle);
        //check if similiar transformation was alredy done
        //todo optimization - remove transformations/normals that are almost identical

        //create transformation matrix based on rotation angles
        //transformation order: translate to origin (0,0,0), rotate by z-axis, rotate by y-axis, then translate back to start
        Transform3D rotationZ = new Transform3D();
        Transform3D rotationY = new Transform3D();
        Transform3D transformation = new Transform3D();

        rotationZ.rotZ(aroundZToXAngle);
        rotationY.rotY(aroundYtoZAngle);

        transformation.mul(translationCenter);
        transformation.mul(rotationZ);
        transformation.mul(rotationY);
        transformation.mul(translationBack);

        return transformation;
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
