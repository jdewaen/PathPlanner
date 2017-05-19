package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.NaivePathPlanner;
import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;


public class ConvexityTest extends ParentTest{
    public static final int RUNS = 5;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {}

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    
    @Ignore
    @Test
    public void straightNaive27(){
        if(!measurePerformance("STRAIGHT NAIVE 27", RUNS, ()->solveNaive(Scenarios::flatBenchmarkSmall, 27))) fail();
    }
    
//    @Ignore
    @Test
    public void straightNaive30(){
        if(!measurePerformance("STRAIGHT NAIVE 30", RUNS, ()->solveNaive(Scenarios::flatBenchmarkSmall, 30))) fail();
    }
    
    @Ignore
    @Test
    public void straightNaive37(){
        if(!measurePerformance("STRAIGHT NAIVE 37", RUNS, ()->solveNaive(Scenarios::flatBenchmarkSmall, 37))) fail();
    }
    
//    @Ignore
    @Test
    public void straightRegular(){
        if(!measurePerformance("STRAIGHT REGULAR", RUNS, ()->solve(Scenarios::flatBenchmarkSmall))) fail();
    }
    
    @Ignore
    @Test
    public void diagNaive27(){
        if(!measurePerformance("DIAG NAIVE 27", RUNS, ()->solveNaive(Scenarios::flatBenchmarkDiagSmall, 27))) fail();
    }
    
//    @Ignore
    @Test
    public void diagNaive30(){
        if(!measurePerformance("DIAG NAIVE 30", RUNS, ()->solveNaive(Scenarios::flatBenchmarkDiagSmall, 30))) fail();
    }
    
    @Ignore
    @Test
    public void diagNaive37(){
        if(!measurePerformance("DIAG NAIVE 37", RUNS, ()->solveNaive(Scenarios::flatBenchmarkDiagSmall, 37))) fail();
    }
    
//    @Ignore
    @Test
    public void diagRegular(){
        if(!measurePerformance("DIAG REGULAR", RUNS, ()->solve(Scenarios::flatBenchmarkDiagSmall))) fail();
    }
    
//    @Ignore
    @Test
    public void updownNaive30(){
        if(!measurePerformance("UP/DOWN NAIVE 30", RUNS, ()->solveNaive(Scenarios::benchmarkSmall, 30))) fail();
    }
    
//    @Ignore
    @Test
    public void updownRegular(){
        if(!measurePerformance("UP/DOWN REGULAR", RUNS, ()->solve(Scenarios::benchmarkSmall))) fail();
    }

    
    private PlannerResult solveNaive(Supplier<ScenarioFactory> factProvider, double maxTime){
        Scenario scenario = factProvider.get().build();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.timeLimit = 15*60;
        solverConfigFact.absMIPgap = 1;
//        solverConfigFact.verbose = true;
        NaivePathPlanner planner = new NaivePathPlanner(solverConfigFact.build(), scenario, maxTime);
        PlannerResult result = planner.solve();
        return result;
    }

}
