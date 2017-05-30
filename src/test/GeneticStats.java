package test;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class GeneticStats {
    public final List<List<Double>> times;
    public final List<List<Double>> areas;
    public static final NumberFormat formatter = new DecimalFormat("#0.00");
    
    public GeneticStats(){
        this.times = new ArrayList<List<Double>>();
        this.areas = new ArrayList<List<Double>>();
    }
    
    public GeneticStats(List<List<Double>> times, List<List<Double>> areas){
        this.times = times;
        this.areas = areas;
    }
    
    public GeneticStats add(GeneticStats other){
        List<List<Double>> newTimes = new ArrayList<List<Double>>();
        newTimes.addAll(times);
        newTimes.addAll(other.times);
        
        List<List<Double>> newAreas = new ArrayList<List<Double>>();
        newAreas.addAll(areas);
        newAreas.addAll(other.areas);
        
        return new GeneticStats(newTimes, newAreas);
    }
    
    public static GeneticStats fromSingle(List<Double> times, List<Double> areas){
        return new GeneticStats(Arrays.asList(times), Arrays.asList(areas));
    }
    
    public static List<Double> getMeans(List<List<Double>> values){
        int valuesPerList = values.get(0).size();
        List<Double> result = new ArrayList<Double>(valuesPerList);
        for(int i = 0; i < valuesPerList; i++){
            double mean = 0;
            for(int j = 0; j < values.size(); j++){
                mean += values.get(j).get(i);
            }
            result.add(mean / values.size());
        }
        return result;
    }
    
    
    public String compareResults(GeneticStats ref){
        
        List<Double> thisTimes = getMeans(times);
        List<Double> thisAreas = getMeans(areas);
        
        List<Double> otherTimes = getMeans(ref.times);
        List<Double> otherAreas = getMeans(ref.areas);
        
        List<Double> absTimeDiffs = new ArrayList<Double>(thisTimes.size());
        List<Double> relTimeDiffs = new ArrayList<Double>(thisTimes.size());
        
        List<Double> absAreaDiffs = new ArrayList<Double>(thisAreas.size());
        List<Double> relAreaDiffs = new ArrayList<Double>(thisAreas.size());
        
//        double highestAbsTimeDiff = Double.NEGATIVE_INFINITY;
//        double lowestAbsTimeDiff = Double.POSITIVE_INFINITY;
//
//        double highestRelTimeDiff = Double.NEGATIVE_INFINITY;
//        double lowestRelTimeDiff = Double.POSITIVE_INFINITY;
//        
//        double highestAbsAreaDiff = Double.NEGATIVE_INFINITY;
//        double lowestAbsAreaDiff = Double.POSITIVE_INFINITY;
//        
//        double highestRelAreaDiff = Double.NEGATIVE_INFINITY;
//        double lowestRelAreaDiff = Double.POSITIVE_INFINITY;
        
        for(int i = 0; i < times.size(); i++){
            double absTimeDiff = thisTimes.get(i) - otherTimes.get(i);
            double relTimeDiff = absTimeDiff / otherTimes.get(i);
            
            double absAreaDiff = thisAreas.get(i) - otherAreas.get(i);
            double relAreaDiff = absAreaDiff / otherAreas.get(i);
            
            absTimeDiffs.add(absTimeDiff);
            relTimeDiffs.add(relTimeDiff);
            
            absAreaDiffs.add(absAreaDiff);
            relAreaDiffs.add(relAreaDiff);
//            
//            if(absTimeDiff > highestAbsTimeDiff) highestAbsTimeDiff = absTimeDiff;
//            if(absTimeDiff < lowestAbsTimeDiff) lowestAbsTimeDiff = absTimeDiff;
//            
//            if(relTimeDiff > highestRelTimeDiff) highestRelTimeDiff = relTimeDiff;
//            if(relTimeDiff < lowestRelTimeDiff) lowestRelTimeDiff = relTimeDiff;
//            
//            if(absAreaDiff > highestAbsTimeDiff) highestAbsTimeDiff = absAreaDiff;
//            if(absAreaDiff < lowestAbsTimeDiff) lowestAbsTimeDiff = absAreaDiff;
//            
//            if(relAreaDiff > highestRelAreaDiff) highestRelAreaDiff = relAreaDiff;
//            if(relAreaDiff < lowestRelAreaDiff) lowestRelAreaDiff = relAreaDiff;
        }
        
        StringBuffer buf = new StringBuffer();
        buf.append("Time Totals:    " 
                + " This=" + formatter.format(sum(thisTimes)) + "s" 
                + " Ref=" + formatter.format(sum(otherTimes)) + "s" 
                + System.lineSeparator());
        
        buf.append("Time Means:     " 
                + " This=" + formatter.format(meanValue(thisTimes)) + "s" 
                + " Ref=" + formatter.format(meanValue(otherTimes)) + "s" 
                + System.lineSeparator());
        
        buf.append("Area Totals:    " 
                + " This=" + formatter.format(sum(thisAreas)) + "" 
                + " Ref=" + formatter.format(sum(otherAreas)) + "" 
                + System.lineSeparator());   
        
        buf.append("Area Means:     " 
                + " This=" + formatter.format(meanValue(thisAreas)) + "" 
                + " Ref=" + formatter.format(meanValue(otherAreas)) + "" 
                + System.lineSeparator());   
        buf.append("AbsTimeDiff:    " 
                + " Mean=" + formatter.format(meanValue(absTimeDiffs)) + "s" 
                + " Min=" + formatter.format(minValue(absTimeDiffs)) + "s" 
                + " Max=" + formatter.format(maxValue(absTimeDiffs)) + "s"
                + " Std=" + formatter.format(std(absTimeDiffs)) + "s"
                + System.lineSeparator());
        
        buf.append("RelTimeDiff:    " 
                + " Mean=" + formatter.format(meanValue(relTimeDiffs)) + "" 
                + " Min=" + formatter.format(minValue(relTimeDiffs)) + "" 
                + " Max=" + formatter.format(maxValue(relTimeDiffs)) + ""
                + " Std=" + formatter.format(std(relTimeDiffs)) + ""
                + System.lineSeparator());
        
        buf.append("AbsAreaDiff:    " 
                + " Mean=" + formatter.format(meanValue(absAreaDiffs)) + "" 
                + " Min=" + formatter.format(minValue(absAreaDiffs)) + "" 
                + " Max=" + formatter.format(maxValue(absAreaDiffs)) + ""
                + " Std=" + formatter.format(std(absAreaDiffs)) + ""
                + System.lineSeparator());
        
        buf.append("RelAreaDiff:    " 
                + " Mean=" + formatter.format(meanValue(relAreaDiffs)) + "" 
                + " Min=" + formatter.format(minValue(relAreaDiffs)) + "" 
                + " Max=" + formatter.format(maxValue(relAreaDiffs)) + ""
                + " Std=" + formatter.format(std(relAreaDiffs)) + ""
                );
        
        return buf.toString();
        
        
    }
    
    private double minValue(List<Double> input){
        double min = Double.MAX_VALUE;
        for(Double cur : input){
            if(cur < min) min = cur;
        }
        return min;
    }
    
    private double maxValue(List<Double> input){
        double max = -Double.MAX_VALUE;
        for(Double cur : input){
            if(cur > max) max = cur;
        }
        return max;
    }
    
    private double meanValue(List<Double> input){
        double mean = 0;
        for(Double cur : input){
            mean += cur;
        }
        return mean / input.size();
    }
    
    private double std(List<Double> input){
        double mean = meanValue(input);
        double variance = 0;
        for(Double cur : input){
            variance += Math.pow(cur - mean, 2);
        }
        return Math.sqrt(variance / (input.size() - 1));
    }
    
    private double sum(List<Double> input){
        double sum = 0;
        for(Double cur : input){
            sum += cur;
        }
        return sum;
    }
    

}
