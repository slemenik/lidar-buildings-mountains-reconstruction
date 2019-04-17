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
import java.io.InterruptedIOException;
import java.util.Locale;

import static com.slemenik.lidar.reconstruction.main.Main.*;

public class BuildingController {


    public static void write() {
        int bounds[] = ShpController.getBoundsFromFilename(INPUT_FILE_NAME);
        write(bounds);
    }

    public static void write(int[] bounds) {


        FeatureIterator iterator = ShpController.getFeatures(bounds);

        int index = 0;
        while (iterator.hasNext() && index < 2) {//temp
            index++;
            Feature feature = iterator.next();

            Property geom = feature.getProperty("the_geom");

            //create temp.laz file from bounds of current building -> call las2las.exe
            if (CREATE_TEMP_FILE) {
                System.out.print("create temp laz file... ");
                BoundingBox boundingBox = feature.getBounds();
                String CMDparams = String.format(Locale.ROOT,
                        "las2las.exe -i %s -o %s -keep_xy %f %f %f %f",
                        INPUT_FILE_NAME, TEMP_FILE_NAME,
                        boundingBox.getMinX()-BOUNDING_BOX_FACTOR,
                        boundingBox.getMinY()-BOUNDING_BOX_FACTOR,
                        boundingBox.getMaxX()+BOUNDING_BOX_FACTOR,
                        boundingBox.getMaxY()+BOUNDING_BOX_FACTOR
                );
                try {
                    Runtime.getRuntime().exec(CMDparams).waitFor();
                } catch (IOException | InterruptedException e) {
                    System.out.println("Error creating temp .laz file: " + e);
                    System.out.print("Reading from input file...");
                    TEMP_FILE_NAME = INPUT_FILE_NAME;
                }
                System.out.println("Done");
            } else {
                TEMP_FILE_NAME = INPUT_FILE_NAME; //if there is no tempfile, we always read from source
            }


//                int result = JniLibraryHelpers.createTempLaz(
//                        boundingBox.getMinX()-BOUNDING_BOX_FACTOR,
//                        boundingBox.getMinY()-BOUNDING_BOX_FACTOR,
//                        boundingBox.getMaxX()+BOUNDING_BOX_FACTOR,
//                        boundingBox.getMaxY()+BOUNDING_BOX_FACTOR
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


    public static void createWall(Coordinate startCoordinate, Coordinate endCoordinate) {
        System.out.println("Ustvari zid od " + startCoordinate + " do " + endCoordinate);
        Coordinate currentCoordinate = startCoordinate.copy();
        double wallLength = startCoordinate.distance(endCoordinate);
        while (startCoordinate.distance(currentCoordinate) <= wallLength) {
            createHeightLine(currentCoordinate);
            currentCoordinate = getNextCoordinate(currentCoordinate, endCoordinate, CREATED_POINTS_SPACING);
        }
        System.out.println("Zid ustvarjen.");
    }

    public static void createPoints(double minZ, double maxZ, double x, double y) {
//        System.out.println("Ustvari točke od višine " + minZ + " do " + maxZ);
        double currentZ = minZ + CREATED_POINTS_SPACING; //we set first Z above minZ, avoiding duplicates points on same level
        while (currentZ < maxZ) {
//            Main.count++;
            if (WRITE_POINTS_INDIVIDUALLY) {
                JniLibraryHelpers.writePoint(x,y,currentZ, INPUT_FILE_NAME, OUTPUT_FILE_NAME);
            } else {
                points2Insert.add(new double[]{x,y,currentZ});
            }

//            System.out.println(new Coordinate(x, y, currentZ));
            currentZ += CREATED_POINTS_SPACING;
        }
    }


    public static void createHeightLine(Coordinate c) {
//        System.out.println("Ustvari točke na koordinati " + c);
        double[] coordinates = JniLibraryHelpers.getMinMaxHeight(c.x,c.y, DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD, TEMP_FILE_NAME);
        if (CONSIDER_EXISTING_POINTS){
            createPoints(coordinates[0], coordinates[1], coordinates[2], coordinates[3]);
        } else {
            createPoints(coordinates[0], coordinates[1], c.x, c.y);
        }

//        System.out.println("Točke ustvarjene.");
    }

}
