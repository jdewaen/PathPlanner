package pathplanner.common;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;



public abstract class ObstacleImporter {
    
    static final double DEGREE_LAT_SIZE = 111131.745;
    
    public static void importFromFile(World2D world, String filename, Pos2D offset){
        BufferedReader reader = null;
        
        
        double degreeLongSize = Math.cos( Math.PI * offset.y / 180) * DEGREE_LAT_SIZE; 
        try{
            reader = new BufferedReader(new FileReader(filename));
            String line;
            while((line = reader.readLine()) != null){
                String[] fields = line.split(",");
                
                double latMin = (Double.valueOf(fields[0]) - offset.y) * DEGREE_LAT_SIZE;
                double lonMin = (Double.valueOf(fields[1]) - offset.x) * degreeLongSize;
                double latMax = (Double.valueOf(fields[2]) - offset.y) * DEGREE_LAT_SIZE;
                double lonMax = (Double.valueOf(fields[3]) - offset.x) * degreeLongSize;
                @SuppressWarnings("unused")
                double height = Double.valueOf(fields[4]);
                
                Obstacle2DB obs = new Obstacle2DB(new Pos2D(lonMin, latMin), 
                        new Pos2D(lonMax, latMax), Double.valueOf(fields[4]));
                
                if(world.isInside(obs)){
                    world.addObstacle(obs);
                }
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    
    public static void convertToKML(String inputname, String outputname){
        BufferedReader reader = null;
        BufferedWriter writer = null;
        try{
            reader = new BufferedReader(new FileReader(inputname));
            writer = new BufferedWriter(new FileWriter(outputname));
            String line;
            int i = 0;
            
            writer.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"); writer.newLine();
            writer.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\">"); writer.newLine();
            writer.write("<Document>"); writer.newLine();

            while((line = reader.readLine()) != null){
                writer.write("<Placemark>"); writer.newLine();
                writer.write(String.format(Locale.ROOT, "<name>%1$d</name>",i)); writer.newLine();
                writer.write("<Polygon>"); writer.newLine();
                writer.write("<extrude>1</extrude>"); writer.newLine();
                writer.write("<altitudeMode>absolute</altitudeMode>"); writer.newLine();
                writer.write("<outerBoundaryIs>"); writer.newLine();
                writer.write("<LinearRing>"); writer.newLine();
                writer.write("<coordinates>"); writer.newLine();

                String[] fields = line.split(",");
                double latMin = Double.valueOf(fields[0]);
                double lonMin = Double.valueOf(fields[1]);
                double latMax = Double.valueOf(fields[2]);
                double lonMax = Double.valueOf(fields[3]);
                double height = Double.valueOf(fields[4]);
                
                writer.write(String.format(Locale.ROOT, "%1$.8f,%2$.8f,%3$.8f", lonMin, latMin, height)); writer.newLine();
                writer.write(String.format(Locale.ROOT, "%1$.8f,%2$.8f,%3$.8f", lonMin, latMax, height)); writer.newLine();
                writer.write(String.format(Locale.ROOT, "%1$.8f,%2$.8f,%3$.8f", lonMax, latMax, height)); writer.newLine();
                writer.write(String.format(Locale.ROOT, "%1$.8f,%2$.8f,%3$.8f", lonMax, latMin, height)); writer.newLine();
                writer.write(String.format(Locale.ROOT, "%1$.8f,%2$.8f,%3$.8f", lonMin, latMin, height)); writer.newLine();

                writer.write("</coordinates>"); writer.newLine();
                writer.write("</LinearRing>"); writer.newLine();
                writer.write("</outerBoundaryIs>"); writer.newLine();
                writer.write("</Polygon>"); writer.newLine();
                writer.write("</Placemark>"); writer.newLine();

                i++;
            }
            writer.write("</Document>"); writer.newLine();
            writer.write("</kml>"); writer.newLine();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            
            if (writer != null) {
                try {
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    
    public static void importMultipleKML(World2D world, List<String> files, Pos2D offset, boolean clockwise){
        files.stream().forEach(file -> importFromKML(world, file, offset, clockwise));
    }
    
    public static void importFromKML(World2D world, String filename, Pos2D offset, boolean clockwise){
        double degreeLongSize = Math.cos( Math.PI * offset.y / 180) * DEGREE_LAT_SIZE; 
        try {

            SAXParserFactory factory = SAXParserFactory.newInstance();
            SAXParser saxParser = factory.newSAXParser();

            DefaultHandler handler = new DefaultHandler() {

            boolean placemark = false;
            boolean polygon = false;
            boolean outer = false;
            boolean ring = false;
            boolean coords = false;
            StringBuffer data = new StringBuffer();

            public void startElement(String uri, String localName,String qName,
                        Attributes attributes) throws SAXException {
                
                if (qName.equalsIgnoreCase("kml:Placemark")) {
                    placemark = true;
                }
                if (qName.equalsIgnoreCase("kml:Polygon")) {
                    polygon = true;
                }
                if (qName.equalsIgnoreCase("kml:outerBoundaryIs")) {
                    outer = true;
                }
                if (qName.equalsIgnoreCase("kml:LinearRing")) {
                    ring = true;
                }
                if (qName.equalsIgnoreCase("kml:coordinates")) {
                    coords = true;
                    data = new StringBuffer();
                }
                
//                System.out.println("Start Element :" + qName);

            }

            public void endElement(String uri, String localName,
                String qName) throws SAXException {

                
                if (qName.equalsIgnoreCase("kml:Placemark")) {
                    placemark = false;
                }
                if (qName.equalsIgnoreCase("kml:Polygon")) {
                    polygon = false;
                }
                if (qName.equalsIgnoreCase("kml:outerBoundaryIs")) {
                    outer = false;
                }
                if (qName.equalsIgnoreCase("kml:LinearRing")) {
                    ring = false;
                }
                if (qName.equalsIgnoreCase("kml:coordinates")) {
                    coords = false;
                    if(placemark && polygon && outer && ring){
                        String[] coordsArray = data.toString().split(" ");
                        List<Pos2D> points = new ArrayList<Pos2D>(); 
                        for(int i = 0; i < coordsArray.length - 1; i++){
                            String[] pointString = coordsArray[i].split(",");
                            double lng = (Double.valueOf(pointString[0]) - offset.x) * degreeLongSize;
                            double lat = (Double.valueOf(pointString[1]) - offset.y) * DEGREE_LAT_SIZE;
                            points.add(new Pos2D(lng, lat));
                        }
                        if(clockwise){
                            Collections.reverse(points);
                        }
                        world.addObstacle(new Obstacle2DB(points));

                    }
                }
                
//                System.out.println("End Element :" + qName);

            }

            public void characters(char ch[], int start, int length) throws SAXException {
                
                if(placemark && polygon && outer && ring && coords){
                    data.append(ch, start, length);
                }
            }

             };

               saxParser.parse(filename, handler);

             } catch (Exception e) {
               e.printStackTrace();
             }
    }

    
}
