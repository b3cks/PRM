package test;

import java.util.List;
import java.util.ArrayList;
import java.util.Random;

import search.*;
import search.algorithms.*;
import search.heuristics.*;
import visualiser.Visualiser;




public class Main {
	

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		long seed = (new Random()).nextLong();
		//System.out.println("Seed: " + seed);
		StateTools.setSeed(seed);
		
		List<ASVState> states = new ArrayList<ASVState>();
		List<Obstacle> obs = new ArrayList<Obstacle>();
		
		//Obstacle myObs1 = new Obstacle(0.4,0.3,0.6,0.05);
		//Obstacle myObs2 = new Obstacle(0,0.3,0.3,0.05);
		Obstacle myObs1 = new Obstacle("0.375 0.000 0.625 0.000 0.625 0.400 0.375 0.400");
		Obstacle myObs2 = new Obstacle("0.375 0.520 0.625 0.520 0.625 1.000 0.375 1.000");
		Obstacle myObs3 = new Obstacle(0.0,0.50,0.8,0.05);
		Obstacle myObs4 = new Obstacle(0.4,0.7,0.6,0.05);
		Obstacle myObs5 = new Obstacle(0,0.7,0.3,0.05);
		obs.add(myObs1);
		obs.add(myObs2);
		/*obs.add(myObs3);
		obs.add(myObs4);
		obs.add(myObs5);*/
		
		int asvCount = 3;
		
		ASVState initialState;
		ASVState goalState;
		
		
		/*System.out.println(goalState.getASVPositions());
		System.out.println(goalState.angles);
		
		System.out.println(StateTools.checkTurningDirection(goalState));
		System.out.println(StateTools.checkTurningDirection(initialState));
		System.out.println(goalState.booms);*/
		
		//Visualiser.displaySampleSpace(new ArrayList<ASVState>(states), new ArrayList<Obstacle>(obs));
		
		//-----------------------------------------------------------------------------------------//
		
		
		/*while(true){
		initialState = StateTools.createRandomStateWithASVPos(asvCount, true,
				0.15, 0.15);
		goalState = StateTools.createRandomStateWithASVPos(asvCount, true,
				0.85, 0.85);
		if (StateTools.isValidState(initialState, obs) && StateTools.isValidState(goalState, obs)) break;
		}*/
		
		initialState = new ASVState(3,"0.185 0.240 0.150 0.180 0.220 0.180 0.069 0.070");
		goalState = new ASVState(3,"0.838 0.153 0.815 0.200 0.792 0.153 0.052 0.052");
		//initialState = new ASVState(7,"0.185 0.281 0.122 0.250 0.106 0.182 0.150 0.127 0.220 0.127 0.264 0.182 0.248 0.250 0.070 0.070 0.070 0.070 0.070 0.070");
		//goalState = new ASVState(7,"0.887 0.071 0.880 0.122 0.857 0.169 0.815 0.200 0.773 0.169 0.750 0.122 0.743 0.071 0.051 0.052 0.052 0.052 0.052 0.051");
		
		System.out.println(initialState.getASVPositions());
		System.out.println(goalState.getASVPositions());
		
		/*System.out.println(initialState.angles);
		System.out.println(initialState.booms);
		System.out.println(goalState.angles);
		System.out.println(goalState.booms);*/
		//System.out.println(goalState.angles);
		
		states.add(initialState);
		states.add(goalState);
		
		System.out.println(StateTools.checkTurningDirection(goalState));
		System.out.println(StateTools.checkTurningDirection(initialState));
		
		Visualiser.displaySampleSpace(new ArrayList<ASVState>(states), new ArrayList<Obstacle>(obs));
		
		long startTime = System.currentTimeMillis();
		
		//states.addAll(StateTools.samplingAroundObs(asvCount, true, obs));
		//ASVState myState1 = StateTools.createRandomState(3,true);
		//ASVState myState2 = StateTools.createRandomState(3, StateTools.checkTurningDirection(myState1));
		//System.out.println(StateTools.isValidState(myState1, obs));
		//System.out.println(StateTools.isValidState(myState2, obs));
		
		Heuristic heuristic;
		heuristic = new DistanceHeuristic(goalState);
		AbstractSearchAlgorithm algo;
		algo = new AStarSearch(initialState, goalState,heuristic);
		
		
		//Visualiser.displaySampleSpace(new ArrayList<ASVState>(states), new ArrayList<Obstacle>(obs));
		
		
		int noOfObs = obs.size();
		//states.addAll(StateTools.samplingInsidePassage(, true, obs));
		
		//Visualiser.displaySampleSpace(new ArrayList<ASVState>(states), new ArrayList<Obstacle>(obs));
		
		sampling(states,asvCount,StateTools.checkTurningDirection(goalState),obs,noOfObs);
		Visualiser.displaySampleSpace(new ArrayList<ASVState>(states), new ArrayList<Obstacle>(obs));
		System.out.println("Graph size: " + states.size());
		connectStates(states,obs,0);
		
		
		int noOfTrail = 1;
		
		while(true){
			algo.verboseSearch();
			if (algo.goalFound()) {
				List<ASVState> path = new ArrayList<ASVState>();
				for (State s : algo.getGoalPath()) {
					path.add((ASVState) s);
				}
				
				Visualiser.displaySampleSpace(new ArrayList<ASVState>(path),
						new ArrayList<Obstacle>(obs));
				
				List<ASVState> realPath = new ArrayList<ASVState>();
				
				
				long startInter = System.currentTimeMillis();
				for (int i = 0; i < path.size()-1; i++){
					realPath.addAll(StateTools.interpolate(path.get(i), path.get(i+1)));
				}
				long endInter = System.currentTimeMillis();
				System.out.println("Interpolating time: " + (endInter - startInter));
				
				Visualiser.displaySampleSpace(new ArrayList<ASVState>(realPath),
						new ArrayList<Obstacle>(obs));
				
				break;
			}
			else{
				noOfTrail++;
				System.out.println("Extending... " + noOfTrail);
				System.out.println("Graph size: " + states.size());
				int size = states.size();
				sampling(states,asvCount,StateTools.checkTurningDirection(goalState),obs,noOfObs);
				connectStates(states,obs,size);
			}
		}
		
		long endTime = System.currentTimeMillis();
		System.out.println("Total time: " + (endTime-startTime));
	}
	
	public static void connectStates(List<ASVState> states, List<Obstacle> obs, int startIndex) {
		for (int i = 0; i < states.size(); i++){
			ASVState s1 = states.get(i);
			int scaler = s1.getASVCount();
			if (i >= startIndex){
				for (int j = i + 1; j < states.size(); j++){
					ASVState s2 = states.get(j);
					double cost = StateTools.getCost(s1, s2);
					if (cost < 0.15*scaler){
						if (StateTools.hasDirectPath(s1, s2, obs)){
							s1.addSuccessor(s2, cost);
							s2.addSuccessor(s1, cost);
						}
					}
				}
			}
			else{
				for (int j = startIndex; j < states.size(); j++){
					ASVState s2 = states.get(j);
					double cost = StateTools.getCost(s1, s2);
					if (cost < 0.15*scaler){
						if (StateTools.hasDirectPath(s1, s2, obs)){
							s1.addSuccessor(s2, cost);
							s2.addSuccessor(s1, cost);
						}
					}
				}
			}
		}
	}
	
	public static void sampling(List<ASVState> states, int asvCount, boolean turning, List<Obstacle> obs, int noOfObs){
		int count = 0;
		int scaler = asvCount;
		for (int i = 0; i < 200*scaler; i++){
			ASVState state = StateTools.createRandomState(asvCount, turning);
			if (StateTools.isValidState(state, obs)){
				states.add(state);
				count++;
			}	
			if (count == 50*scaler) break;
		}
		states.addAll(StateTools.samplingAroundObs(asvCount, turning, obs));
		states.addAll(StateTools.samplingOnBoundary(asvCount, turning, obs));
	}
}
