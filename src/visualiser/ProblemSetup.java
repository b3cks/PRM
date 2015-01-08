package visualiser;


import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import test.Obstacle;
import test.ASVState;

public class ProblemSetup {
	private boolean problemLoaded = false;
	private boolean solutionLoaded = false;
	
	private int asvCount;
	private ASVState initialState;
	private ASVState goalState;
	private List<Obstacle> obstacles;
	private List<ASVState> samples;
	private List<ASVState> path;
	
	public void loadProblem(String filename) throws IOException {
		problemLoaded = false;
		solutionLoaded = false;
		BufferedReader input = new BufferedReader(new FileReader(filename));
		try {
			asvCount = Integer.valueOf(input.readLine().trim());
			initialState = new ASVState(asvCount, input.readLine().trim());
			goalState = new ASVState(asvCount, input.readLine().trim());
			
			int numObstacles = Integer.valueOf(input.readLine().trim());
			obstacles = new ArrayList<Obstacle>();
			for (int i = 0; i < numObstacles; i++) {
				obstacles.add(new Obstacle(input.readLine().trim()));
			}
			input.close();
			problemLoaded = true;
		} catch (NumberFormatException e) {
			throw new IOException("Invalid number format.");
		} catch (IndexOutOfBoundsException e) {
			throw new IOException("Invalid format; not enough tokens in a line.");
		}
	}
	
	public void loadCSpace(List<ASVState> states,List<Obstacle> obs){
		this.samples = states;
		this.obstacles = obs;
		problemLoaded = true;
	}
	
	public void loadSolution(String filename) throws IOException {
		if (!problemLoaded) {
			return;
		}
		solutionLoaded = false;
		BufferedReader input = new BufferedReader(new FileReader(filename));
		try {
			String[] tokens = input.readLine().trim().split("\\s+");
			int pathLength = Integer.valueOf(tokens[0]);
			path = new ArrayList<ASVState>();
			for (int i = 0; i < pathLength; i++) {
				ASVState s = new ASVState(asvCount, input.readLine().trim());
				path.add(s);
			}
			input.close();
			solutionLoaded = true;
		} catch (NumberFormatException e) {
			throw new IOException("Invalid number format.");
		} catch (IndexOutOfBoundsException e) {
			throw new IOException("Invalid format; not enough tokens in a line.");
		}
	}
	
	public int getASVCount() {
		return asvCount;
	}
	
	public ASVState getInitialState() {
		return initialState;
	}
	
	public ASVState getGoalState() {
		return goalState;
	}
	
	public List<ASVState> getPath() {
		return new ArrayList<ASVState>(path);
	}
	
	public List<ASVState> getSampleStates() {
		return new ArrayList<ASVState>(samples);
	}
	
	public List<Obstacle> getObstacles() {
		return new ArrayList<Obstacle>(obstacles);
	}
	
	public boolean problemLoaded() {
		return problemLoaded;
	}
	
	public boolean solutionLoaded() {
		return solutionLoaded;
	}
}
