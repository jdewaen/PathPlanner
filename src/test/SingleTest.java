package test;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.common.Scenario;


public class SingleTest {
    public static final int RUNS = 5;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {}

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}

    @Test
    public void measurePerformance() {
        System.out.println("RUNNING SINGLE BLOCK...");
        MultiStatTracker stats = new MultiStatTracker();
        for(int i = 0; i < RUNS; i++){
            PlannerResult result = solveSingle();
            stats.trackers.add(result.stats);
        }
        System.out.println("----------------");
        System.out.println("SINGLE BLOCK");
        System.out.println(stats);
        System.out.println("----------------");
    }
    
    
    private PlannerResult solveSingle(){
        Scenario scenario = Scenarios.singleBlock();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
