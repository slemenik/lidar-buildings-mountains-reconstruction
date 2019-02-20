package com.slemenik.lidar.reconstruction.main;

import com.github.mreutegg.laszip4j.LASPoint;
import com.github.mreutegg.laszip4j.LASReader;
import com.github.mreutegg.laszip4j.laszip.LASpoint;
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
import org.geotools.feature.simple.SimpleFeatureTypeBuilder;
import org.geotools.feature.simple.SimpleFeatureTypeImpl;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.opengis.feature.Feature;
import org.opengis.feature.GeometryAttribute;
import org.opengis.feature.Property;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.simple.SimpleFeatureType;
import org.opengis.feature.type.FeatureType;
import org.opengis.feature.type.PropertyDescriptor;
import org.opengis.filter.Filter;
import org.opengis.filter.FilterFactory;

import java.io.File;
import java.io.Serializable;
import java.util.*;


public class Main {

    public static void main(String[] args) {
//    shp();

        System.out.println("start");

        Coordinate a = new Coordinate(1.0,1.0);
        Coordinate b = new Coordinate(1.0,2.0);
//        System.out.println(a.equals2D(b, 1.0));

write();
//las();
    }

    public static void write () {
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

            FeatureSource oldFeatureSource = shp();

            String typeName2 = "BU_STAVBE_P";

            FeatureType schema = oldFeatureSource.getSchema();
            String name = schema.getGeometryDescriptor().getLocalName();

            System.out.println(name);
            System.out.println();
            String srs = oldFeatureSource.getBounds().getCoordinateReferenceSystem().toString();

            FilterFactory ff = CommonFactoryFinder.getFilterFactory( null );
            Filter filter = ff.bbox("the_geom", 462000.0, 100000.0, 463000, 101000, srs);
//            ff.property( "the_geom"), ff.literal( 12 )
//            Filter filter = CQL.toFilter(text.getText());

            Query query = new Query(typeName2, filter, new String[] {name});
            FeatureCollection oldFeatureCollection = oldFeatureSource.getFeatures(query);



//            System.out.println(oldFeatureSource.getFeatures(query).size());
//            System.out.println(oldFeatureSource.getFeatures().size());

            List<SimpleFeature> features = new ArrayList<>();

            FeatureIterator iterator = oldFeatureCollection.features();
            int i = 0;
            while (iterator.hasNext() && i++ < 2) {
                Feature feature = iterator.next();

                Property geom = feature.getProperty("the_geom");
//                System.out.println(feature.getDefaultGeometryProperty().getDescriptor().getCoordinateReferenceSystem());
//                System.out.println(feature.getDefaultGeometryProperty().getValue());
//                System.out.println(feature.getDefaultGeometryProperty().getBounds());
//                System.out.println(feature.getBounds());
//                System.out.println(feature.getType());
//                System.out.println(geom.getName());
//                System.out.println(geom.getType());
//                System.out.println(geom.getUserData());
//                System.out.println(geom.getDescriptor());
                System.out.println("Feature"+i);
                MultiPolygon a = (MultiPolygon) geom.getValue();
//                System.out.println(a.getBoundary());

//                System.out.println(a.);
                for (Coordinate coordinate : a.getCoordinates()) {

                    System.out.println("iščem ujemanje s koodrinato: " + coordinate);
                    findPointsWithXY(coordinate, 1.0);
//                    coordinate.equals2D()
                }
                System.out.println();
//                (geom.);


                features.add((SimpleFeature)feature);

                System.out.println("-------------------------------------------");
            }

            if (true) return;

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
            System.out.println("typeName: "  + typeName);
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

        } catch (Exception e){System.out.println(e);}
        System.out.println();
    }

    public static void createPoints(double minZ, double maxZ, double x, double y) {
//        LASPoint p = LASPgetX();
    }

    public static void findPointsWithXY(Coordinate c, double allowedDistance ) {
        LASReader reader = new LASReader(new File("./data/462_100_grad.laz"));

        double maxVisina = 0.0;
        double minVisina = Double.MAX_VALUE;

        PriorityQueue queue = new PriorityQueue<Coordinate>((o1, o2) -> {
            double d1 = o1.distance(c);
            double d2 = o2.distance(c);

            return d1 > d2 ? 1 : -1;
        });

        for (LASPoint p : reader.getPoints()) {
            double lasX = p.getX()/100.0;
            double lasY = p.getY()/100.0;
            double lasZ = p.getZ()/100.0;



            Coordinate c1 = new Coordinate(lasX, lasY);
//            System.out.println(c1);
//            System.out.println(c);

            if (c.equals2D(c1, allowedDistance)) {
                if (lasZ > maxVisina) maxVisina = lasZ;
                if (lasZ < minVisina) minVisina = lasZ;
            }
//                System.out.print("ujema se točka: ");
//                System.out.print(lasX + ", ");
//                System.out.print(lasY + "\n");

//            }
                queue.add(c1);



//            if (x + allowedDistance)

        }
        //najdi najbližjo
        System.out.println("najbližja: " + queue.peek());
        System.out.println("maxVisina: " + maxVisina);
        System.out.println("minVisina: " + minVisina);
//        Collections.sort(bestCandidates, new Comparator<Coordinate>() {
//
//            @Override
//            public int compare(Coordinate fruit1, Coordinate fruit2) {
//
//
//
//                //descending order
//                //return fruitName2.compareTo(fruitName1);
//            }
//        });

    }

    public static void las() {
        LASReader reader = new LASReader(new File("./data/462_100_grad.laz"));
        for (LASPoint p : reader.getPoints()) {
            // read something from point
            System.out.println(p.getX());
            System.out.println(p.getX()/100.0);
//            System.out.println(p.getY());
//            System.out.println(p.getZ());
            p.getClassification();
            if (
            p.getX() <= 462258 /*&& p.getX()-2 <= 462258*/) {
                System.out.println(p.getY());
            }
        }

        Iterable<LASPoint> a = reader.getPoints();
    }

    public static FeatureSource shp() {
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
//            FeatureCollection collection = featureSource.getFeatures();


//            FeatureIterator iterator = collection.features();
//            System.out.println(collection.size());
//
//
          /*  try {
                int i = 0;
                while (iterator.hasNext() && i++ < 20) {
                    Feature feature = iterator.next();
                    GeometryAttribute sourceGeometry = feature.getDefaultGeometryProperty();
//                    System.out.println(feature.getType().getName().toString());
//                    System.out.println(feature.getIdentifier().getID());
//                    System.out.println(feature.getName());
                    System.out.println(feature.getName());
                    System.out.println(sourceGeometry.getValue());
                    System.out.println(feature.getProperty("VISINA"));
//                    for (PropertyDescriptor a : feature.getType().getDescriptors()) {
//                        System.out.println(a.getName());
//                        System.out.println(a.getType());
//                        System.out.println(a.getUserData());
////                        System.out.println(feature.getName());
//
//                    }
//                    System.out.println(feature.getType().getName().toString());
//                    System.out.println(feature.getBounds().getMaxY());
                }
            } finally {
                iterator.close();
            }
*/
        } catch (Throwable e) {System.out.println(e);}
        return featureSource;
    }
}
