package test;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;


public class ObsTest extends ParentTest{
    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    @Test
    public void benchmarkSmall(){
        printObsCount("BENCHMARK SMALL", Scenarios::benchmarkSmall);
    }
    
    @Ignore
    @Test
    public void benchmarkSmallFlat(){
        printObsCount("BENCHMARK SMALL FLAT", Scenarios::flatBenchmarkSmall);
    }

    @Ignore
    @Test
    public void benchmarkSmallDiag(){
        printObsCount("BENCHMARK SMALL DIAG",Scenarios::flatBenchmarkDiagSmall);
    }

    @Test
    public void benchmarkLarge(){
        printObsCount("BENCHMARK LARGE", Scenarios::benchmarkLarge);
    }
    
    @Test
    public void spiral(){
        printObsCount("SPIRAL", Scenarios::spiral);
    }
        
    @Test
    public void leuvenSmall1(){
        printObsCount("LEUVEN SMALL 1", Scenarios::leuvenSmall);
    }
    
    @Test
    public void leuvenSmall2(){
        printObsCount("LEUVEN SMALL 2", Scenarios::leuvenSmallAlternate);
    }
    
    @Test
    public void leuvenLarge1(){
        printObsCount("LEUVEN LARGE 1", Scenarios::leuvenLarge);
    }

    @Test
    public void sanFranciscoSmall1(){
        printObsCount("SF SMALL 1", Scenarios::sanFranciscoSmall);
    }
    
    @Test
    public void sanFranciscoSmall2(){
        printObsCount("SF SMALL 2", Scenarios::sanFranciscoSmallAlternate);
    }
    
    @Test
    public void sanFranciscoLarge1(){
        printObsCount("SF LARGE 1", Scenarios::sanFranciscoLarge);
    }
    
    
    private void printObsCount(String text, Supplier<ScenarioFactory> factProvider){
        Scenario scen = factProvider.get().build();
        int count = (int) scen.world.getObstacles().stream()
                .count();
        System.out.println(text + ": " + count);
    }

}
