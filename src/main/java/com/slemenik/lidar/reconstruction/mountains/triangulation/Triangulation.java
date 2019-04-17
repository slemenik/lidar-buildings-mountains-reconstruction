package com.slemenik.lidar.reconstruction.mountains.triangulation;

import com.slemenik.lidar.reconstruction.main.Main;
import org.locationtech.jts.geom.Coordinate;

import java.io.*;
import java.util.*;
import java.util.List;

public class Triangulation {

    public static void triangulate(String fileName) {

        System.out.println("Started...");
        List<PointDTO> pointList = getParsedData(fileName);
        System.out.println(pointList.size());
        HashSet<TriangleDTO> triangulation = getTriangulation(pointList);
        makeOBJ(triangulation, true);
        System.out.println("Finished.");
    }

    //1. PointDTO cloud data
    private static List<PointDTO> getParsedData(String fileName) {
        //.asc only - format: x;y;z\n (e.g. ARSO Lidar DMR)
        List<PointDTO> pointList = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(new File(fileName)))) {
            String line;
            int id = 1;

            Coordinate center = new Coordinate(410810.150, 137776.920, 2777.130);
            double radius = 100.0;

            while ((line = br.readLine()) != null) {
                String[] coordinatesString = line.split(";");
                double x = Double.parseDouble(coordinatesString[0]);
                double y = Double.parseDouble(coordinatesString[1]);
                double z = Double.parseDouble(coordinatesString[2]);

                if (center.distance3D(new Coordinate(x,y,z)) < radius) {
                    PointDTO p = new PointDTO(x,y,z, id++);
                    pointList.add(p);
                }
            }
        } catch (Exception e){
            System.out.println("izjema" + e);
        }
        return pointList;

    }

    //3D surface reconstruction.
    private static HashSet<TriangleDTO> getTriangulation(List<PointDTO> pointList){

        //Bowyer–Watson algorithm
        int triangleID = 1;
        List<TriangleDTO> triangulation = new LinkedList<>();
        //super-triangle (http://page.mi.fu-berlin.de/faniry/files/faniry_aims.pdf -> 4.1.)
        double M = getMaximumAbsoluteCoordinate(pointList);
        TriangleDTO superTriangle = new TriangleDTO(  new PointDTO(3*M, 0,0, -1),//-1
                new PointDTO(0,3*M, 0, -2),//-2
                new PointDTO( -3*M, -3*M, 0, -3), -1);//-3
        triangulation.add(superTriangle);
        HashSet<TriangleDTO> solution = new HashSet<>();
        for (PointDTO point : pointList) {
//            System.out.println();
            if (point.getId() % 10000 == 0)System.out.println("tocka" + point);
            HashSet<EdgeDTO> edge1stAppearance = new HashSet<>(); //integer counts num of same edges
            HashSet<EdgeDTO> polygon = new HashSet<>();

            Iterator<TriangleDTO> i = triangulation.iterator();
            while (i.hasNext()) {
                TriangleDTO triangle = i.next();
                if (inCircle(point, triangle.getPoint1(), triangle.getPoint2(), triangle.getPoint3())) {
                    i.remove();
                    solution.remove(triangle);
                    if (edge1stAppearance.contains(triangle.getEdge1())) {//already appeared - will NOT be in polygon
                        //edge is not shared by any other triangles in badTriangles
                        polygon.remove(triangle.getEdge1());
                    } else {//1st appearance
                        edge1stAppearance.add(triangle.getEdge1());
                        polygon.add(triangle.getEdge1());
                    }

                    if (edge1stAppearance.contains(triangle.getEdge2())) {
                        polygon.remove(triangle.getEdge2());
                    } else {
                        edge1stAppearance.add(triangle.getEdge2());
                        polygon.add(triangle.getEdge2());
                    }

                    if (edge1stAppearance.contains(triangle.getEdge3())) {
                        polygon.remove(triangle.getEdge3());
                    } else {
                        edge1stAppearance.add(triangle.getEdge3());
                        polygon.add(triangle.getEdge3());
                    }
                }
            }

            for (EdgeDTO edge : polygon) {
                TriangleDTO newTriangle = new TriangleDTO(point, edge.getPoint1(), edge.getPoint2(), triangleID++);
                triangulation.add(newTriangle);
                if (hasNoSuperTrianglePoint(newTriangle, superTriangle)) {
                    solution.add(newTriangle);//assume it is solution, if not remove later
                }
            }
        }
        return solution;
    }

    //Output and Implementation
    private static void makeOBJ(HashSet<TriangleDTO> triangles, boolean writeToFile) {

        HashMap<Integer, String> vertexMap = new HashMap<>();//id=>"v x y z"
        ArrayList<String> faceList = new ArrayList<>();
        for (TriangleDTO triangle : triangles) {

            if (!vertexMap.containsKey(triangle.getPoint1().getId()))
                vertexMap.put(triangle.getPoint1().getId(),
                        "v " + triangle.getPoint1().getX() + " " + triangle.getPoint1().getY() + " " + triangle.getPoint1().getZ() );
            if (!vertexMap.containsKey(triangle.getPoint2().getId()))
                vertexMap.put(triangle.getPoint2().getId(),
                        "v " + triangle.getPoint2().getX() + " " + triangle.getPoint2().getY() + " " + triangle.getPoint2().getZ() );
            if (!vertexMap.containsKey(triangle.getPoint3().getId()))
                vertexMap.put(triangle.getPoint3().getId(),
                        "v " + triangle.getPoint3().getX() + " " + triangle.getPoint3().getY() + " " + triangle.getPoint3().getZ() );

            String facesString = String.format("f %d %d %d", triangle.getPoint1().getId(), triangle.getPoint2().getId(), triangle.getPoint3().getId());
            if (!faceList.contains(facesString)) {
                faceList.add(facesString);
            }
        }
        PrintWriter writer = null;
        if (writeToFile) {
            try {
                writer = new PrintWriter(Main.DATA_FOLDER + "output.obj", "UTF-8");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        for (Map.Entry<Integer, String> entry : vertexMap.entrySet()) {
            if (writeToFile){
                assert writer != null;
                writer.println(entry.getValue());
            } else{
                System.out.println(entry.getValue());
            }
        }
        System.out.println();
        if (writeToFile) writer.println();

        for (String face : faceList){
            if (writeToFile){
                writer.println(face);
            } else {
                System.out.println(face);
            }
        }
        if (writeToFile) writer.close();
    }

    private static boolean hasNoSuperTrianglePoint(TriangleDTO triangle, TriangleDTO superTriangle){
        return  (!triangle.getPoint1().equals(superTriangle.getPoint1()) &&
                !triangle.getPoint1() .equals( superTriangle.getPoint2()) &&
                !triangle.getPoint1() .equals( superTriangle.getPoint3()) &&
                !triangle.getPoint2() .equals( superTriangle.getPoint1()) &&
                !triangle.getPoint2() .equals( superTriangle.getPoint2()) &&
                !triangle.getPoint2() .equals( superTriangle.getPoint3())&&
                !triangle.getPoint3() .equals( superTriangle.getPoint1()) &&
                !triangle.getPoint3() .equals( superTriangle.getPoint2()) &&
                !triangle.getPoint3() .equals( superTriangle.getPoint3()));
    }

    private static double getMaximumAbsoluteCoordinate(List<PointDTO> pointList) {
        double M = 0.0; //absolute maximum
        for (PointDTO point : pointList) {
            if (Math.abs(point.getX()) > M) M = Math.abs(point.getX());
            if (Math.abs(point.getY()) > M) M = Math.abs(point.getY());
        }
        return M;
    }

    private static boolean inCircle(PointDTO pt, PointDTO v1, PointDTO v2, PointDTO v3) {

        double ax = v1.getX();
        double ay = v1.getY();
        double bx = v2.getX();
        double by = v2.getY();
        double cx = v3.getX();
        double cy = v3.getY();
        double dx = pt.getX();
        double dy = pt.getY();

        double  ax_ = ax-dx;
        double  ay_ = ay-dy;
        double  bx_ = bx-dx;
        double  by_ = by-dy;
        double  cx_ = cx-dx;
        double  cy_ = cy-dy;
        double det=  (
                (ax_*ax_ + ay_*ay_) * (bx_*cy_-cx_*by_) -
                        (bx_*bx_ + by_*by_) * (ax_*cy_-cx_*ay_) +
                        (cx_*cx_ + cy_*cy_) * (ax_*by_-bx_*ay_)
        );

        if (ccw ( ax,  ay,  bx,  by,  cx,  cy)) {
            return (det>0);
        } else {
            return (det<0);
        }

    }

    private static boolean ccw(double ax, double ay, double bx, double by, double cx, double cy) {//counter-clockwise
        return (bx - ax)*(cy - ay)-(cx - ax)*(by - ay) > 0;
    }


}
