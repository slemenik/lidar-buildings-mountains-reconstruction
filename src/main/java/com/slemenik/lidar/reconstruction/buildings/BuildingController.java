package com.slemenik.lidar.reconstruction.buildings;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import org.geotools.feature.FeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.math.Vector2D;
import org.opengis.feature.Feature;
import org.opengis.feature.Property;
import org.opengis.geometry.BoundingBox;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class BuildingController {

    public String inputLazFileName;
    public boolean createTempLazFile;
    public String tempLazFileName;
    public double boundingBoxFactor;
    public double createdPointsSpacing;
    public double distanceFromOriginalPointThreshold;
    public boolean considerExistingPoints;
    public String outputFileName;
    public String shpFileName;

    public List<double[]> points2Insert = new ArrayList<>();

    public void write() {
        double bounds[] = ShpController.getBoundsFromFilename(inputLazFileName);
        write(bounds);
    }

//    public double[][] start() {
//
//    }

    public void write(double[] bounds) {


        FeatureIterator iterator = ShpController.getFeatures(bounds, shpFileName);

        int index = 0;
        while (iterator.hasNext() && index < 2) {//temp
            index++;
            int j =0;
//            while (j++ < 8){
//                iterator.next();
//            }
            Feature feature = iterator.next();

            Property geom = feature.getProperty("the_geom");

            //create temp.laz file from bounds of current building -> call las2las.exe
            if (createTempLazFile) {
                System.out.print("create temp laz file... ");
                BoundingBox boundingBox = feature.getBounds();
                String CMDparams = String.format(Locale.ROOT,
                        "las2las.exe -i %s -o %s -keep_xy %f %f %f %f",
                        inputLazFileName, tempLazFileName,
                        boundingBox.getMinX()- boundingBoxFactor,
                        boundingBox.getMinY()- boundingBoxFactor,
                        boundingBox.getMaxX()+ boundingBoxFactor,
                        boundingBox.getMaxY()+ boundingBoxFactor
                );
                try {
                    Runtime.getRuntime().exec(CMDparams).waitFor();
                } catch (IOException | InterruptedException e) {
                    System.out.println("Error creating temp .laz file: " + e);
                    System.out.print("Reading from input file...");
                    tempLazFileName = inputLazFileName;
                }
                System.out.println("Done");
            } else {
                tempLazFileName = inputLazFileName; //if there is no tempfile, we always read from source
            }


//                int result = JniLibraryHelpers.createTempLaz(
//                        boundingBox.getMinX()-boundingBoxFactor,
//                        boundingBox.getMinY()-boundingBoxFactor,
//                        boundingBox.getMaxX()+boundingBoxFactor,
//                        boundingBox.getMaxY()+boundingBoxFactor
//                );


//                GeometryAttribute sourceGeometry = feature.getDefaultGeometryProperty();
//                System.out.println(feature.getType().getName().toString());
//                System.out.println(feature.getIdentifier().getID());
//                System.out.println(feature.getName());
//                System.out.println(feature.getName());
//                System.out.println(sourceGeometry.getValue());
//                System.out.println(feature.getProperty("VISINA"));
//                for (PropertyDescriptor a : feature.getType().getDescriptors()) {
//                    System.out.println(a.getName());
//                    System.out.println(a.getType());
//                    System.out.println(a.getUserData());
//                    System.out.println(feature.getName());
//
//                }
//                System.out.println(feature.getType().getName().toString());
//                System.out.println(feature.getBounds().getMaxY());
//
            //System.out.println(feature.getDefaultGeometryProperty().getDescriptor().getCoordinateReferenceSystem());
            //System.out.println(feature.getDefaultGeometryProperty().getValue());
//                System.out.println(feature.getDefaultGeometryProperty().getBounds());
//                System.out.println(feature.getBounds());
//                System.out.println(feature.getType());
//                System.out.println(geom.getName());
//                System.out.println(geom.getType().getDescription());
//                System.out.println(geom.getUserData());
//                System.out.println(geom.getDescriptor());

            System.out.println("Stavba"+index);
            MultiPolygon buildingPolygon = (MultiPolygon) geom.getValue();
            Coordinate[] buildingVertices = buildingPolygon.getCoordinates();
            for (int i = 0; i < buildingVertices.length - 1; i++ ) { //for each until the one before last
                Coordinate vertexFrom =  buildingVertices[i];
                Coordinate vertexTo = buildingVertices[i+1];
                createWall(vertexFrom, vertexTo);
            }
            System.out.println();

            // features.add((SimpleFeature)feature); //uncomment if write shp to file

            System.out.println("-------------------------------------------");
        }
        iterator.close();

//            writeShpFile( oldFeatureSource,  features, oldFeatureCollection);

        System.out.println("Konec racunanja.");
    }

    //returns the Coordinate that lies on a line between "start" and "end" and is a "distance" away from start
    public static Coordinate getNextCoordinate(Coordinate start, Coordinate end, double distance) {
        // https://math.stackexchange.com/questions/175896
        Vector2D v0 = new Vector2D(start);
        Vector2D v1 = new Vector2D(end);
        Vector2D v = v1.subtract(v0);

        Vector2D u = v.normalize();
        Vector2D newPoint = v0.add(u.multiply(distance));
        return new Coordinate(newPoint.getX(), newPoint.getY());
    }


    public void createWall(Coordinate startCoordinate, Coordinate endCoordinate) {
        System.out.println("Ustvari zid od " + startCoordinate + " do " + endCoordinate);
        Coordinate currentCoordinate = startCoordinate.copy();
        double wallLength = startCoordinate.distance(endCoordinate);
        while (startCoordinate.distance(currentCoordinate) <= wallLength) {
            createHeightLine(currentCoordinate);
            currentCoordinate = getNextCoordinate(currentCoordinate, endCoordinate, createdPointsSpacing);
        }
        System.out.println("Zid ustvarjen.");
    }

    private void createPoints(double minZ, double maxZ, double x, double y) {
//        System.out.println("Ustvari točke od višine " + minZ + " do " + maxZ);
        double currentZ = minZ + createdPointsSpacing; //we set first Z above minZ, avoiding duplicates points on same level
        while (currentZ < maxZ) {
            points2Insert.add(new double[]{x,y,currentZ});
//            System.out.println(new Coordinate(x, y, currentZ));
            currentZ += createdPointsSpacing;
        }
    }


    private void createHeightLine(Coordinate c) {
//        System.out.println("Ustvari točke na koordinati " + c);
        double[] coordinates = JniLibraryHelpers.getMinMaxHeight(c.x,c.y, distanceFromOriginalPointThreshold, tempLazFileName);
        if (considerExistingPoints){
            createPoints(coordinates[0], coordinates[1], coordinates[2], coordinates[3]);
        } else {
            createPoints(coordinates[0], coordinates[1], c.x, c.y);
        }

//        System.out.println("Točke ustvarjene.");
    }

}
