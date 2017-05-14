package test;

import java.util.function.Supplier;

import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;


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
    
    public PlannerResult solve(Supplier<ScenarioFactory> factProvider){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }
}
