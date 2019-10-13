package com.slemenik.lidar.reconstruction.buildings;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import com.slemenik.lidar.reconstruction.main.DTO;
import com.slemenik.lidar.reconstruction.main.HelperClass;
import com.slemenik.lidar.reconstruction.main.Main;
import org.geotools.feature.FeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.math.Vector2D;
import org.opengis.feature.Feature;
import org.opengis.feature.Property;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.geometry.BoundingBox;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//import static com.slemenik.lidar.reconstruction.buildings.ShpController.oldFeatureCollectiontemp;
//import static com.slemenik.lidar.reconstruction.buildings.ShpController.oldFeatureSourcetemp;

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

    private double[] bounds;

    public List<double[]> points2Insert = new ArrayList<>();

    public BuildingController(DTO.LasHeader headerDTO) {
        this.inputLazFileName = Main.INPUT_FILE_NAME;
        this.createTempLazFile = Main.CREATE_TEMP_FILE;
        this.tempLazFileName = Main.TEMP_FILE_NAME;
        this.boundingBoxFactor = Main.BOUNDING_BOX_FACTOR;
        this.createdPointsSpacing = Main.CREATED_POINTS_SPACING;
        this.distanceFromOriginalPointThreshold = Main.DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD;
        this.considerExistingPoints = Main.CONSIDER_EXISTING_POINTS;
        this.outputFileName = Main.OUTPUT_FILE_NAME;
        this.shpFileName = Main.SHP_FILE_NAME;
        this.bounds = new double[]{headerDTO.minX, headerDTO.minY, headerDTO.maxX, headerDTO.maxY };
        //this is not original header minx and maxy but changed one

//        bc.write(TEMP_BOUNDS);
//        return bc.points2Insert;
    }

    public BuildingController(){}

//    public void write() {
//        double bounds[] = ShpController.getBoundsFromFilename(inputLazFileName);
//        write(bounds);
//    }

    public List<double[]> getNewPoints() {
        HelperClass.printLine(" ", "Function BuildingController.getNewPoints()");
        FeatureIterator iterator = ShpController.getFeatures(bounds, shpFileName);
        List<SimpleFeature> features = new ArrayList<>();
        int index = 0;
//        try {
        while (iterator.hasNext()) {

            Feature feature = iterator.next();
            System.out.print("create temp laz file... ");
            BoundingBox boundingBox = feature.getBounds();
            String CMDparams = String.format(Locale.ROOT,
                    Main.LAS_2_LAS_FILE_NAME + " -i %s -o %s -keep_xy %f %f %f %f",
                    inputLazFileName + ".laz", tempLazFileName + ".laz",
                    boundingBox.getMinX()- boundingBoxFactor,
                    boundingBox.getMinY()- boundingBoxFactor,
                    boundingBox.getMaxX()+ boundingBoxFactor,
                    boundingBox.getMaxY()+ boundingBoxFactor
            );
            try {
                Runtime.getRuntime().exec(CMDparams).waitFor();
            } catch (IOException | InterruptedException e) {
                System.out.println("Error creating temp .laz file: " + e + ". Check if las2las.exe exists in specifed folder: " + new File(Main.LAS_2_LAS_FILE_NAME).getAbsolutePath());
                System.out.print("Reading from input file...");
                tempLazFileName = inputLazFileName;
            }
            System.out.println("Done");
            Property geom = feature.getProperty(feature.getDefaultGeometryProperty().getName());
            System.out.println("Stavba"+index++);
            MultiPolygon buildingPolygon = (MultiPolygon) geom.getValue();
            Coordinate[] buildingVertices = buildingPolygon.getCoordinates();
            for (int i = 0; i < buildingVertices.length - 1; i++ ) { //for each until the one before last
                Coordinate vertexFrom =  buildingVertices[i];
                Coordinate vertexTo = buildingVertices[i+1];
                createWall(vertexFrom, vertexTo, null);
            }
             features.add((SimpleFeature)feature); //uncomment if write shp to file

            System.out.println();
            System.out.println("-------------------------------------------");
        }
        iterator.close();
        System.out.println("END BuildingController.getNewPoints()");
//                    ShpController.writeShpFile( oldFeatureSourcetemp,  features, oldFeatureCollectiontemp, "tempppp.shp");

        return this.points2Insert;
    }

//    public void write(double[] bounds) {
//
//
//        FeatureIterator iterator = ShpController.getFeatures(bounds, shpFileName);
//
//        int index = 0;
//        while (iterator.hasNext() && index < 2) {//temp
//            index++;
//            int j =0;
////            while (j++ < 8){
////                iterator.next();
////            }
//            Feature feature = iterator.next();
//
//            Property geom = feature.getProperty("the_geom");
//
//            //create temp.laz file from bounds of current building -> call las2las.exe
//            if (createTempLazFile) {
//                System.out.print("create temp laz file... ");
//                BoundingBox boundingBox = feature.getBounds();
//                String CMDparams = String.format(Locale.ROOT,
//                        "las2las.exe -i %s -o %s -keep_xy %f %f %f %f",
//                        inputLazFileName, tempLazFileName,
//                        boundingBox.getMinX()- boundingBoxFactor,
//                        boundingBox.getMinY()- boundingBoxFactor,
//                        boundingBox.getMaxX()+ boundingBoxFactor,
//                        boundingBox.getMaxY()+ boundingBoxFactor
//                );
//                try {
//                    Runtime.getRuntime().exec(CMDparams).waitFor();
//                } catch (IOException | InterruptedException e) {
//                    System.out.println("Error creating temp .laz file: " + e);
//                    System.out.print("Reading from input file...");
//                    tempLazFileName = inputLazFileName;
//                }
//                System.out.println("Done");
//            } else {
//                tempLazFileName = inputLazFileName; //if there is no tempfile, we always read from source
//            }
//
//
////                int result = JniLibraryHelpers.createTempLaz(
////                        boundingBox.getMinX()-boundingBoxFactor,
////                        boundingBox.getMinY()-boundingBoxFactor,
////                        boundingBox.getMaxX()+boundingBoxFactor,
////                        boundingBox.getMaxY()+boundingBoxFactor
////                );
//
//
////                GeometryAttribute sourceGeometry = feature.getDefaultGeometryProperty();
////                System.out.println(feature.getType().getName().toString());
////                System.out.println(feature.getIdentifier().getID());
////                System.out.println(feature.getName());
////                System.out.println(feature.getName());
////                System.out.println(sourceGeometry.getValue());
////                System.out.println(feature.getProperty("VISINA"));
////                for (PropertyDescriptor a : feature.getType().getDescriptors()) {
////                    System.out.println(a.getName());
////                    System.out.println(a.getType());
////                    System.out.println(a.getUserData());
////                    System.out.println(feature.getName());
////
////                }
////                System.out.println(feature.getType().getName().toString());
////                System.out.println(feature.getBounds().getMaxY());
////
//            //System.out.println(feature.getDefaultGeometryProperty().getDescriptor().getCoordinateReferenceSystem());
//            //System.out.println(feature.getDefaultGeometryProperty().getValue());
////                System.out.println(feature.getDefaultGeometryProperty().getBounds());
////                System.out.println(feature.getBounds());
////                System.out.println(feature.getType());
////                System.out.println(geom.getName());
////                System.out.println(geom.getType().getDescription());
////                System.out.println(geom.getUserData());
////                System.out.println(geom.getDescriptor());
//
//            System.out.println("Stavba"+index);
//            MultiPolygon buildingPolygon = (MultiPolygon) geom.getValue();
//            Coordinate[] buildingVertices = buildingPolygon.getCoordinates();
//            for (int i = 0; i < buildingVertices.length - 1; i++ ) { //for each until the one before last
//                Coordinate vertexFrom =  buildingVertices[i];
//                Coordinate vertexTo = buildingVertices[i+1];
//                createWall(vertexFrom, vertexTo, null);
//            }
//            System.out.println();
//
//            // features.add((SimpleFeature)feature); //uncomment if write shp to file
//
//            System.out.println("-------------------------------------------");
//        }
//        iterator.close();
//
////            writeShpFile( oldFeatureSource,  features, oldFeatureCollection);
//
//        System.out.println("Konec racunanja.");
//    }

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


    public void createWall(Coordinate startCoordinate, Coordinate endCoordinate, Feature feature) {
        System.out.println("Ustvari zid od " + startCoordinate + " do " + endCoordinate);
        Coordinate currentCoordinate = startCoordinate.copy();
        double wallLength = startCoordinate.distance(endCoordinate);
        while (startCoordinate.distance(currentCoordinate) <= wallLength) {
            createHeightLine(currentCoordinate, feature);
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

    private void createHeightLine(Coordinate c, Feature feature) {
//        System.out.println("Ustvari točke na koordinati " + c);
        double[] coordinates;
        if (feature != null) {
            BoundingBox boundingBox = feature.getBounds();
            double minX = boundingBox.getMinX()- boundingBoxFactor;
            double minY = boundingBox.getMinY()- boundingBoxFactor;
            double maxX = boundingBox.getMaxX()+ boundingBoxFactor;
            double maxY = boundingBox.getMaxY()+ boundingBoxFactor;
            coordinates = JniLibraryHelpers.getMinMaxHeight(c.x,c.y, distanceFromOriginalPointThreshold, inputLazFileName, minX, minY, maxX, maxY);
        } else {
           coordinates = JniLibraryHelpers.getMinMaxHeight(c.x,c.y, distanceFromOriginalPointThreshold, tempLazFileName);
        }
        if (considerExistingPoints){
            createPoints(coordinates[0], coordinates[1], coordinates[2], coordinates[3]);
        } else {
            createPoints(coordinates[0], coordinates[1], c.x, c.y);
        }

//        System.out.println("Točke ustvarjene.");
    }

}
