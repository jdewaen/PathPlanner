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


public class PrelimTest extends ParentTest{
    public static final int RUNS = 1;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {}

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    @Test
    public void updown0(){
        if(!measurePerformance("UP/DOWN 0", RUNS, ()->solveNaive(Scenarios::benchmark0, 30))) fail();
    }
    
    @Test
    public void updown1(){
        if(!measurePerformance("UP/DOWN 1", RUNS, ()->solveNaive(Scenarios::benchmark1, 30))) fail();
    }
    
    @Test
    public void updown2(){
        if(!measurePerformance("UP/DOWN 2", RUNS, ()->solveNaive(Scenarios::benchmark2, 30))) fail();
    }
    
    @Test
    public void updown3(){
        if(!measurePerformance("UP/DOWN 3", RUNS, ()->solveNaive(Scenarios::benchmark3, 30))) fail();
    }
    @Ignore
    @Test
    public void updown4(){
        if(!measurePerformance("UP/DOWN 4", RUNS, ()->solveNaive(Scenarios::benchmark4, 30))) fail();
    }
    
    @Ignore
    @Test
    public void updown5(){
        if(!measurePerformance("UP/DOWN 5", RUNS, ()->solveNaive(Scenarios::benchmarkSmall, 30))) fail();
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
