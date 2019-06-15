package com.slemenik.lidar.reconstruction.mountains;



import delaunay_triangulation.Point_dt;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import delaunay_triangulation.Delaunay_Triangulation;

import static com.slemenik.lidar.reconstruction.buildings.ShpController.getBoundsFromFilename;

public class MountainController {

    public String dmrFileName;

    private double centerX;
    private double centerY;
    private double centerZ;

    private List<Point3d> originalPointList = new ArrayList<>();

    public MountainController(double[][] arr) {
//        dmrFileName = new ArrayList<>();
        setCenterAndPointList(arr);
    }

    private void setCenterAndPointList(double[][] arr){
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

         centerX = (minX + maxX)/2;
         centerY = (minY + maxY)/2;
         centerZ = (minZ + maxZ)/2;
    }

    public void start() {

        Point_dt[] dmrPointList = getDmrFromFile(dmrFileName);

        System.out.println("Start triangulation");
        Delaunay_Triangulation dt = new Delaunay_Triangulation(dmrPointList);
        System.out.println("End triangulation");

        //create normals
        List<Vector3D> normalList = new ArrayList<>();
        dt.trianglesIterator().forEachRemaining(triangleDt -> {
            if (!triangleDt.isHalfplane()) {
                Vector3D normal = getNormalVector(triangleDt.p1(), triangleDt.p2(), triangleDt.p3());
                //todo maybe calculate here directly instead of creating a list and then iterating it
                normalList.add(normal);
            }
        });

        normalList.forEach(normal -> {
            //todo optimization - remove normals that are almost identical
            Transform3D transformation = getRotationTransformation(normal.getX(), normal.getY(), normal.getZ());
            List<Point3d> rotatedPoints = new ArrayList<>();
            originalPointList.forEach(originalPoint -> {
                Point3d newPoint = new Point3d();
                transformation.transform(originalPoint, newPoint);
                rotatedPoints.add(newPoint);
            });

        });

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

        //create transformation matrix based on rotation angles
        //transformation order: translate to origin (0,0,0), rotate by z-axis, rotate by y-axis, then translate back to start
        Transform3D rotationZ = new Transform3D();
        Transform3D rotationY = new Transform3D();
        Transform3D translationCenter = new Transform3D();
        Transform3D translationBack = new Transform3D();
        Transform3D transformation = new Transform3D();

        translationCenter.setTranslation(new Vector3d(centerX, centerY, centerZ));
        translationBack.setTranslation(new Vector3d(-centerX, -centerY, -centerZ));
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
    private Point_dt[] getDmrFromFile(String ascFileName) {
        System.out.println("method getDmrFromFile()");
        List<Point_dt> list = new ArrayList<>();
//        int i = 0;
//        Coordinate center = new Coordinate(410810.150, 137776.920, 2777.130);
//        double radius = 150.0;

        try {
            Scanner scanner = new Scanner(new File(ascFileName));
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] coordinates = line.split(";");
                double x = Double.parseDouble(coordinates[0]);
                double y = Double.parseDouble(coordinates[1]);
                double z = Double.parseDouble(coordinates[2]);
                list.add(new Point_dt(x,y,z));

//                if (center.distance3D(new Coordinate(x,y,z)) < radius) {
//                    temp.add( new Point3d(x,y,z));
//                    temp2.add(new Point_dt(x,y,z));;
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
        System.out.println("method getNormalVector()");
//
//        System.out.println(p1.toString());
//        System.out.println(p2.toString());
//        System.out.println(p3.toString());

        Vector3D U = new Vector3D(p2.x(), p2.y(), p2.z()).subtract(new Vector3D(p1.x(), p1.y(), p1.z()));
        Vector3D V = new Vector3D(p3.x(), p3.y(), p3.z()).subtract(new Vector3D(p1.x(), p1.y(), p1.z()));

        Vector3D normal = U.crossProduct(V);

        if (normal.getZ() < 0) { // if normal points down, change direction of normal
            normal = V.crossProduct(U);
        }

        System.out.println("normal vector: " + normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
        return normal;
//        return getNormalAngle(normal.getX(),normal.getY(), normal.getZ());
//        return rotate(new Vector3d(normal.getX(),normal.getY(), normal.getZ()));
//        System.out.println("end getNormalVector()");
    }

}
