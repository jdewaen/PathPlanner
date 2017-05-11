package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;


public class AgilityTest extends ParentTest{
    public static final int RUNS = 1;
    boolean runBench = true;
    boolean runLeuven = true;
    boolean runSF = true;

    // 3 cases: more less standard acc
    // 3 cases: more less standard max speed
    // 3 scenarios: benchmark, leuven sf
    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    // BENCHMARK

    @Test
    public void benchmarkLowAccLowSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccLowSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccLowSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkLowAccMedSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccMedSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccMedSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkLowAccHighSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC HIGH SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccHighSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC HIGH SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccHighSpeed(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenLowAccLowSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccLowSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccLowSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenLowAccMedSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccMedSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccMedSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenLowAccHighSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccHighSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccHighSpeed(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoLowAccLowSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccLowSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccLowSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoLowAccMedSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccMedSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccMedSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoLowAccHighSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccHighSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccHighSpeed(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    private PlannerResult solve(Supplier<ScenarioFactory> factProvider){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
