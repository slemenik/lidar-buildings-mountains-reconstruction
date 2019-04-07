package com.slemenik.lidar.reconstruction.main;

import com.github.mreutegg.laszip4j.LASPoint;
import com.github.mreutegg.laszip4j.LASReader;
import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import org.apache.commons.io.FilenameUtils;
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
import org.opengis.geometry.BoundingBox;

import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.*;
import java.util.concurrent.TimeUnit;

public class Main {

//    private static final String INPUT_FILE_NAME = ".\\data\\410_137_triglav.laz";
    private static final String INPUT_FILE_NAME = ".\\data\\462_100_grad.laz";
    private static final String OUTPUT_FILE_NAME =".\\data\\out.laz";
    private static String TEMP_FILE_NAME = ".\\data\\temp.laz";

    private static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 1; //manjše je bolj natančno za detajle, ne prekrije celega
    private static final double CREATED_POINTS_SPACING = 0.5;//2.0;//0.2;
    private static final boolean WRITE_POINTS_INDIVIDUALLY = false;
    private static final boolean CONSIDER_EXISTING_POINTS = false;
    private static final double BOUNDING_BOX_FACTOR = 2.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    private static final boolean CREATE_TEMP_FILE = true;
    private static final boolean PROCESS_WHOLE_FILE = true;

    private static int count = 0;
    private static List<double[]> points2Insert = new ArrayList<>();

    public static String getHTML(String urlToRead) throws Exception {
        StringBuilder result = new StringBuilder();
        URL url = new URL(urlToRead);
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        conn.setRequestMethod("GET");
        BufferedReader rd = new BufferedReader(new InputStreamReader(conn.getInputStream()));
        String line;
        while ((line = rd.readLine()) != null) {
            result.append(line);
            //System.out.println(result);
        }
        rd.close();
        return result.toString();
    }

    public static void main(String[] args) {
        System.out.println("start");
        long startTime = System.nanoTime();
        write();
        System.out.println("Konec racunanja.");


//        System.out.println(f.getAbsoluteFile());

        if (!points2Insert.isEmpty()) {
            System.out.println("zacetek pisanja... ");
            double[][] pointListDoubleArray = points2Insert.toArray(new double[][]{});
            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME);
            System.out.println(returnValue);
        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Sekunde izvajanja: " + time);
        System.out.println("end");
    }

    public static int[] getLowerBoundsFromFilename(String filename){
        String fileName = FilenameUtils.getBaseName(filename);
        int bboxX = 0;
        int bboxY = 0;
        for (String str : fileName.split("_")) {
            try {
                int num = Integer.parseInt(str);
                if (bboxX == 0) {
                    bboxX = num;
                } else if (bboxY == 0) {
                    bboxY = num;
                    break;
                }
            } catch (NumberFormatException e) {

            }
        }
        return new int[]{bboxX*1000, bboxY*1000};
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
            Filter filter;

            if (PROCESS_WHOLE_FILE) {
                int bounds[] = getLowerBoundsFromFilename(INPUT_FILE_NAME);
                filter = ff.bbox("the_geom", bounds[0], bounds[1], bounds[0]+1000, bounds[1]+1000, srs);
            } else {
                filter = ff.bbox("the_geom", 462735, 97883, 462829 , 98000, srs); //lj grad, temp
            }


//

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
            while (iterator.hasNext() && index > -1) {//temp
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
                    Runtime.getRuntime().exec(CMDparams).waitFor();
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

    //todo - ko iščeš min in max, poglej kaj se zgodi če dobiš premalo točk - samo ena ali celo nič - povečaj toleracno
    //todo - kaj pa tam ko odstopa od shapefajla - zelo, povečaš toleranco?
    //todo - zapiši še ostale atribute npr. klasifikacijo, index return itd - poskusi tudi user data - vse private static final paramtere s katerimi si kreiral trenutni file
    //todo - tam kjer so v steni še vseeno luknje - mogočeč dinamično večanje radija pri MinMAXHeight, ali pa upoštevanje sosdenjih točk gleda na klasifikacijo
    // npr. minHegiht za tla, Max za streho
    //todo napiš source za laslib v githubu od cpp
    //todo - odstrani nepotrebne knjižnice (za branje las) iz import in iz pom
    //todo zbriši temp file ko končas
    //todo preimenuj dll
    //todo glej robne točke oz robne stavbe - kaj je z njimi
    //todo raziskuj kaj se zgodi in kako zajezit napako, ko se bbox ne ujema z dejanskim fajlom in se temp ne ustvari
    //todo related zgoraj - kaj če se ne zgeneraira temp file - že iz prve, torej je prazen ali pa se upošteva še prejšnji
    // mogoče dodaj še da se lahko iz dveh laz fajlov skupej naredi naenkrat rezultat

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
