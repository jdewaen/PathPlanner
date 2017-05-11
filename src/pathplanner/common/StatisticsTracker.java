package pathplanner.common;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;


public class StatisticsTracker {
    public long totalTime = 0;
    public long prePathTime = 0;
    public long cornerTime = 0;
    public long pathSegmentTime = 0;
    public long scenSegmentTime = 0;
    public LinkedList<Long> geneticTimes = new LinkedList<Long>();
    public LinkedList<Long> setupTime = new LinkedList<Long>();
    public LinkedList<Long> solveTime = new LinkedList<Long>();
    NumberFormat              formatter        = new DecimalFormat("#0.00");
    public double score = 0;
    
    public long startTimer(){
        return System.currentTimeMillis();
    }
    
    public long stopTimer(long startTime){
        return System.currentTimeMillis() - startTime;
    }
    
    @Override
    public String toString(){
        long genetic = geneticTimes.stream().reduce((long) 0, (a, b) -> a + b );
        long setup = setupTime.stream().reduce((long) 0, (a, b) -> a + b );
        long solve = solveTime.stream().reduce((long) 0, (a, b) -> a + b );
        
        StringBuffer buf = new StringBuffer();
        buf.append("PrePath:    " + formatter.format(toSeconds(prePathTime)) + "s" + System.lineSeparator());
        buf.append("Corner:     " + formatter.format(toSeconds(cornerTime)) + "s" + System.lineSeparator());
        buf.append("PathSegment:" + formatter.format(toSeconds(pathSegmentTime)) + "s" + System.lineSeparator());
        buf.append("ScenSegment:" + formatter.format(toSeconds(scenSegmentTime)) + "s" + System.lineSeparator());
        buf.append("Genetic:    " + formatter.format(toSeconds(genetic)) + "s" + System.lineSeparator());
        buf.append("Setup:      " + formatter.format(toSeconds(setup)) + "s" + System.lineSeparator());
        buf.append("Solve:      " + formatter.format(toSeconds(solve)) + "s" + System.lineSeparator());
        buf.append("TOTAL:      " + formatter.format(toSeconds(totalTime)) + "s" + System.lineSeparator());
        buf.append("SCORE:      " + formatter.format(score) + "s");
        
        return buf.toString();
    }
    
    public static double toSeconds(long input){
        return ((double) input) / 1000;
    }
}
