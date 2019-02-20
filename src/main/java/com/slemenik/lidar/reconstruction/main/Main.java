package com.slemenik.lidar.reconstruction.main;

import com.github.mreutegg.laszip4j.LASPoint;
import com.github.mreutegg.laszip4j.LASReader;
import org.geotools.data.*;
import org.geotools.data.collection.ListFeatureCollection;
import org.geotools.data.shapefile.ShapefileDataStore;
import org.geotools.data.shapefile.ShapefileDataStoreFactory;
import org.geotools.data.simple.SimpleFeatureCollection;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.data.simple.SimpleFeatureStore;
import org.geotools.factory.CommonFactoryFinder;
import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.math.Vector2D;
import org.opengis.feature.Feature;
import org.opengis.feature.Property;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.simple.SimpleFeatureType;
import org.opengis.feature.type.FeatureType;
import org.opengis.filter.Filter;
import org.opengis.filter.FilterFactory;

import java.io.File;
import java.io.Serializable;
import java.util.*;

public class Main {

    private static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 1.0;
    private static final double CREATED_POINTS_SPACING = 0.2;

    private static int count = 0;

    public static void main(String[] args) {
        System.out.println("start");
        write();
        System.out.println();
        System.out.println("end");
    }

    public static void write () {
        try {

            FeatureSource oldFeatureSource = getFeatureSource();

            String typeName = "BU_STAVBE_P";

            FeatureType schema = oldFeatureSource.getSchema();
            String name = schema.getGeometryDescriptor().getLocalName();

            System.out.println(name);
            System.out.println();
            String srs = oldFeatureSource.getBounds().getCoordinateReferenceSystem().toString();

            FilterFactory ff = CommonFactoryFinder.getFilterFactory( null );
            Filter filter = ff.bbox("the_geom", 462000.0, 100000.0, 463000, 101000, srs);
//            ff.property( "the_geom"), ff.literal( 12 )
//            Filter filter = CQL.toFilter(text.getText());

            Query query = new Query(typeName, filter, new String[] {name});
            FeatureCollection oldFeatureCollection = oldFeatureSource.getFeatures(query);

//            System.out.println(oldFeatureSource.getFeatures(query).size());
//            System.out.println(oldFeatureSource.getFeatures().size());

            List<SimpleFeature> features = new ArrayList<>();

            FeatureIterator iterator = oldFeatureCollection.features();
//            System.out.println(collection.size());
            int index = 0;
            while (iterator.hasNext() && index++ < 2) {
                Feature feature = iterator.next();

                Property geom = feature.getProperty("the_geom");

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
//                System.out.println(feature.getDefaultGeometryProperty().getDescriptor().getCoordinateReferenceSystem());
//                System.out.println(feature.getDefaultGeometryProperty().getValue());
//                System.out.println(feature.getDefaultGeometryProperty().getBounds());
//                System.out.println(feature.getBounds());
//                System.out.println(feature.getType());
//                System.out.println(geom.getName());
//                System.out.println(geom.getType());
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

                features.add((SimpleFeature)feature);

                System.out.println("-------------------------------------------");
            }
            iterator.close();

//            writeShpFile( oldFeatureSource,  features, oldFeatureCollection);


        } catch (Exception e){
            System.out.println(e);
        }
        System.out.println();
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

    public static void writeShpFile(FeatureSource oldFeatureSource, List<SimpleFeature> features, FeatureCollection oldFeatureCollection) {
        try {
            /*
             * Get an output file name and create the new shapefile
             */
            File newFile = new File("./data/test.shp");

            ShapefileDataStoreFactory dataStoreFactory = new ShapefileDataStoreFactory();

            Map<String, Serializable> params = new HashMap<>();
            params.put("url", newFile.toURI().toURL());
            params.put("create spatial index", Boolean.TRUE);

            ShapefileDataStore newDataStore =
                    (ShapefileDataStore) dataStoreFactory.createNewDataStore(params);

            SimpleFeatureType TYPE = (SimpleFeatureType) oldFeatureSource.getSchema();

            /*
             * TYPE is used as a template to describe the file contents
             */
            newDataStore.createSchema(TYPE);

            /*
             * Write the features to the shapefile
             */
            Transaction transaction = new DefaultTransaction("create");

            String typeName = newDataStore.getTypeNames()[0];
            System.out.println("typeName: " + typeName);
            SimpleFeatureSource featureSource = newDataStore.getFeatureSource(typeName);
            SimpleFeatureType SHAPE_TYPE = featureSource.getSchema();
            /*
             * The Shapefile format has a couple limitations:
             * - "the_geom" is always first, and used for the geometry attribute name
             * - "the_geom" must be of type Point, MultiPoint, MuiltiLineString, MultiPolygon
             * - Attribute names are limited in length
             * - Not all data types are supported (example Timestamp represented as Date)
             *
             * Each data store has different limitations so check the resulting SimpleFeatureType.
             */
            System.out.println("SHAPE:" + SHAPE_TYPE);

            if (featureSource instanceof SimpleFeatureStore) {
                SimpleFeatureStore featureStore = (SimpleFeatureStore) featureSource;
                /*
                 * SimpleFeatureStore has a method to add features from a
                 * SimpleFeatureCollection object, so we use the ListFeatureCollection
                 * class to wrap our list of features.
                 */
                SimpleFeatureCollection collection = new ListFeatureCollection(TYPE, features);
                featureStore.setTransaction(transaction);
                try {
                    featureStore.addFeatures(oldFeatureCollection/*collection*/);
                    transaction.commit();
                } catch (Exception problem) {
                    problem.printStackTrace();
                    transaction.rollback();
                } finally {
                    transaction.close();
                }
                System.exit(0); // success!
            } else {
                System.out.println(typeName + " does not support read/write access");
                System.exit(1);
            }
        } catch (Exception e) {System.out.println(e);}
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
            Coordinate pointToCreate = new Coordinate(x, y, currentZ);
//            Main.count++;
//            System.out.println("št. " + Main.count);
//            JNI.writePoint(pointToCreate); // todo
//            System.out.println(pointToCreate);
//            System.out.println("Točka kao ustvarjen: " + pointToCreate);
            currentZ += CREATED_POINTS_SPACING;
        }
    }

    public static Iterable<LASPoint> getLasPoints() { //todo; use Native Library
        LASReader reader = new LASReader(new File("./data/462_100_grad.laz"));
        return reader.getPoints();
    }

    public static void createHeightLine(Coordinate c) {
//        System.out.println("Ustvari točke na koordinati " + c);
        double maxHeight = 0.0;
        double minHeight = Double.MAX_VALUE;

        PriorityQueue<Coordinate> queue = new PriorityQueue<>((o1, o2) -> {
            double d1 = o1.distance(c);
            double d2 = o2.distance(c);
            return Double.compare(d1,d2);
        });

        for (LASPoint p : getLasPoints()) {
            double lasX = p.getX() / 100.0;
            double lasY = p.getY() / 100.0;
            double lasZ = p.getZ() / 100.0;

            Coordinate c1 = new Coordinate(lasX, lasY);

            // calculate max and min z coordinate
            if (c.equals2D(c1, DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD)) {
                if (lasZ > maxHeight) maxHeight = lasZ;
                if (lasZ < minHeight) minHeight = lasZ;
            }
            queue.add(c1);

        }
        Coordinate closest = queue.peek();
        createPoints(minHeight, maxHeight, closest.x, closest.y);
//        System.out.println("Točke ustvarjene.");
    }

    //todo - ko iščeš min in max, poglej kaj se zgodi če dobiš premalo točk - samo ena ali celo nič - povečaj toleracno

    public static FeatureSource getFeatureSource() {
        File file = new File("./data/BU_STAVBE_P.shp");
        FeatureSource featureSource = null;
        try {
            Map<String, String> connect = new HashMap();
            connect.put("url", file.toURI().toString());

            DataStore dataStore = DataStoreFinder.getDataStore(connect);
            String[] typeNames = dataStore.getTypeNames();
            String typeName = typeNames[0];

            System.out.println("Reading content " + typeName);

            featureSource = dataStore.getFeatureSource(typeName);

        } catch (Throwable e) {
            System.out.println(e);
        }
        return featureSource;
    }
}
