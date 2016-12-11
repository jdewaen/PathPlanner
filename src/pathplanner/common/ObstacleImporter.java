package pathplanner.common;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;


public abstract class ObstacleImporter {
    
    public static void importFromFile(World2D world, String filename){
        BufferedReader reader = null;
        try{
            reader = new BufferedReader(new FileReader(filename));
            String line;
            while((line = reader.readLine()) != null){
                String[] fields = line.split(",");
                Obstacle2D obs = new Obstacle2D(new Pos2D(Double.valueOf(fields[1]), Double.valueOf(fields[0])), 
                        new Pos2D(Double.valueOf(fields[3]), Double.valueOf(fields[2])), Double.valueOf(fields[4]));
                world.addRegion(obs);
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
}
