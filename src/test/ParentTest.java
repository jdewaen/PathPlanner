package test;

import java.util.function.Supplier;

import org.junit.Test;

import pathplanner.PlannerResult;


public abstract class ParentTest {

    public boolean measurePerformance(String name, int runs, Supplier<PlannerResult> func) {
        System.out.print("RUNNING " + name + "...");
        MultiStatTracker stats = new MultiStatTracker();
        for(int i = 0; i < runs; i++){
            PlannerResult result = func.get();
            if(result.failed){
                System.out.println("");
                System.out.println("FAIL");
//                return false;
                i--;
                continue;
            }
            stats.trackers.add(result.stats);
            System.out.print(" " + i);
        }
        System.out.println("");
        System.out.println(stats);
        System.out.println("----------------");
        return true;
    }
}
