package test;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import pathplanner.common.StatisticsTracker;


public class MultiStatTracker {
    public final LinkedList<StatisticsTracker> trackers = new LinkedList<StatisticsTracker>();
    NumberFormat              formatter        = new DecimalFormat("#0.00");

    
    @Override
    public String toString(){
        
        if(trackers.isEmpty()) return "NO DATA";
        
        LinkedList<Double> prePathTimes = new LinkedList<Double>();
        LinkedList<Double> cornerTimes = new LinkedList<Double>();
        LinkedList<Double> pathSegmentTimes = new LinkedList<Double>();
        LinkedList<Double> scenSegmentTimes = new LinkedList<Double>();
        LinkedList<Double> geneticTimes = new LinkedList<Double>();
        LinkedList<Double> setupTimes = new LinkedList<Double>();
        LinkedList<Double> solveTimes = new LinkedList<Double>();
        LinkedList<Double> totalTimes = new LinkedList<Double>();
        LinkedList<Double> scores = new LinkedList<Double>();
        
        for(StatisticsTracker tracker : trackers){
            prePathTimes.add(StatisticsTracker.toSeconds(tracker.prePathTime));
            cornerTimes.add(StatisticsTracker.toSeconds(tracker.cornerTime));
            pathSegmentTimes.add(StatisticsTracker.toSeconds(tracker.pathSegmentTime));
            scenSegmentTimes.add(StatisticsTracker.toSeconds(tracker.scenSegmentTime));
            geneticTimes.add(StatisticsTracker.toSeconds(tracker.geneticTimes.stream().reduce((long) 0, (a, b) -> a + b )));
            setupTimes.add(StatisticsTracker.toSeconds(tracker.setupTime.stream().reduce((long) 0, (a, b) -> a + b )));
            solveTimes.add(StatisticsTracker.toSeconds(tracker.solveTime.stream().reduce((long) 0, (a, b) -> a + b )));
            totalTimes.add(StatisticsTracker.toSeconds(tracker.totalTime));
            scores.add(tracker.score);
        }
        
        
        
        StringBuffer buf = new StringBuffer();
        buf.append("PrePath:    " 
                + " Mean=" + formatter.format(meanValue(prePathTimes)) + "s" 
                + " Min=" + formatter.format(minValue(prePathTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(prePathTimes)) + "s"
                + System.lineSeparator());
        buf.append("Corner:     " 
                + " Mean=" + formatter.format(meanValue(cornerTimes)) + "s" 
                + " Min=" + formatter.format(minValue(cornerTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(cornerTimes)) + "s"
                + System.lineSeparator());
        buf.append("PathSegment:" 
                + " Mean=" + formatter.format(meanValue(pathSegmentTimes)) + "s" 
                + " Min=" + formatter.format(minValue(pathSegmentTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(pathSegmentTimes)) + "s"
                + System.lineSeparator());
        buf.append("ScenSegment:" 
                + " Mean=" + formatter.format(meanValue(scenSegmentTimes)) + "s" 
                + " Min=" + formatter.format(minValue(scenSegmentTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(scenSegmentTimes)) + "s"
                + System.lineSeparator());
        buf.append("Genetic:    " 
                + " Mean=" + formatter.format(meanValue(geneticTimes)) + "s" 
                + " Min=" + formatter.format(minValue(geneticTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(geneticTimes)) + "s"
                + System.lineSeparator());
        buf.append("Setup:      " 
                + " Mean=" + formatter.format(meanValue(setupTimes)) + "s" 
                + " Min=" + formatter.format(minValue(setupTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(setupTimes)) + "s"
                + System.lineSeparator());
        buf.append("Solve:      " 
                + " Mean=" + formatter.format(meanValue(solveTimes)) + "s" 
                + " Min=" + formatter.format(minValue(solveTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(solveTimes)) + "s"
                + System.lineSeparator());
        buf.append("TOTAL:      " 
                + " Mean=" + formatter.format(meanValue(totalTimes)) + "s" 
                + " Min=" + formatter.format(minValue(totalTimes)) + "s" 
                + " Max=" + formatter.format(maxValue(totalTimes)) + "s"
                + System.lineSeparator());
        buf.append("SCORE:      " 
                + " Mean=" + formatter.format(meanValue(scores)) + "s" 
                + " Min=" + formatter.format(minValue(scores)) + "s" 
                + " Max=" + formatter.format(maxValue(scores)) + "s"
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

}
