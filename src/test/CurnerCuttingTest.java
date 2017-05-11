package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;


public class CurnerCuttingTest extends ParentTest{
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
    public void leuvenAllowed(){
        if(!measurePerformance("LEUVEN ALLOW CUT", RUNS, ()->solveCuttingAllowed(Scenarios::leuvenSmall))) fail();
    }
    
    @Test
    public void leuvenBlocked(){
        if(!measurePerformance("LEUVEN BLOCK CUT", RUNS, ()->solveCuttingBlocked(Scenarios::leuvenSmall))) fail();
    }
    
    @Test
    public void benchmarkAllowed(){
        if(!measurePerformance("BENCHMARK ALLOW CUT", RUNS, ()->solveCuttingAllowed(Scenarios::benchmarkLarge))) fail();
    }
    
    @Test
    public void benchmarkBlocked(){
        if(!measurePerformance("BENCHMARK BLOCK CUT", RUNS, ()->solveCuttingBlocked(Scenarios::benchmarkLarge))) fail();
    }
    
    @Test
    public void SFAllowed(){
        if(!measurePerformance("SF ALLOW CUT", RUNS, ()->solveCuttingAllowed(Scenarios::sanFranciscoSmall))) fail();
    }
    
    @Test
    public void SFBlocked(){
        if(!measurePerformance("SF BLOCK CUT", RUNS, ()->solveCuttingBlocked(Scenarios::sanFranciscoSmall))) fail();
    }
    
    private PlannerResult solveCuttingAllowed(Supplier<ScenarioFactory> factProvider){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.preventCornerCutting = false;
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }
    
    private PlannerResult solveCuttingBlocked(Supplier<ScenarioFactory> factProvider){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.preventCornerCutting = true;
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
