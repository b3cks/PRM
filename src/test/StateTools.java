package test;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class StateTools {
	
	/** The maximum distance any ASV can travel between two states */
	public static final double MAX_STEP = 0.001;
	/** The minimum allowable boom length */
	public static final double MIN_BOOM_LENGTH = 0.05;
	/** The maximum allowable boom length */
	public static final double MAX_BOOM_LENGTH = 0.075;
	/** The default value for maximum error */
	public static final double DEFAULT_MAX_ERROR = 1e-5;
	/** The random number generator */
	public static final Random random = new Random();
	public static final Rectangle2D BOUNDS = new Rectangle2D.Double(0, 0, 1, 1);
	public static final double maxError = 0;

	/** Sets the seed for Random Number Generator */
	public static void setSeed(long seed) {
		random.setSeed(seed);
	}

	/**
	 * Generates a random RAS 
	 */
	public static ASVState createRandomState(int noOfASV, boolean clockwise) {
		List<Double> booms = new ArrayList<Double>();
		List<Double> angles = new ArrayList<Double>();
		
		double angle1 = (random.nextDouble() - 0.5) * Math.PI * 2;
		double boom1 = 0.05 + random.nextDouble()*0.025;
		
		booms.add(boom1);
		angles.add(angle1);
		
		double angleMean = (Math.PI * 2 / noOfASV)*0.75;
		
		if (!clockwise){
			for (int i = 0; i < noOfASV-2; i++){
				angles.add(angles.get(i) + StateTools.getGaussian(angleMean));
				booms.add(0.05 + random.nextDouble()*0.025);
			}
		}
		else{
			for (int i = 0; i < noOfASV-2; i++){
				angles.add(angles.get(i) - StateTools.getGaussian(angleMean));
				booms.add(0.05 + random.nextDouble()*0.025);
			}
		}
		
		return new ASVState(booms, angles, new Point2D.Double(random.nextDouble(),random.nextDouble()),noOfASV);
	}
	
	
	public static double getGaussian(double mean){
		double number = 0;
		while(true){
			number = mean + random.nextGaussian() * Math.PI*1.75;
			if (number > 0 && number < Math.PI)
				break;
		}
		return number;
	}
	
	/**
	 * 
	 * @param s
	 * @return true if counter-clockwise, false if clockwise
	 */
	public static boolean checkTurningDirection(ASVState s){
		List<Double> angles = s.angles;
		if (angles.get(0) - angles.get(1) > 0)
			return true;
		else
			return false;
	}
	
	public static double maxDistance(ASVState s1, ASVState s2){
		List<Point2D> points1 = s1.getASVPositions();
		List<Point2D> points2 = s2.getASVPositions();
		double max = points1.get(0).distance(points2.get(0));
		for (int i = 1; i < s1.getASVCount(); i++){
			if (points1.get(i).distance(points2.get(i)) > max)
				max = points1.get(i).distance(points2.get(i));
		}
		return max;
	}

	
	
	/**
	 * Remember to add goal state in the end!
	 * @param s1
	 * @param s2
	 * @return
	 */
	public static List<ASVState> interpolate(ASVState s1, ASVState s2){
		if (StateTools.maxDistance(s1, s2) > 0.001){
			double x = (s1.midASV.getX() + s2.midASV.getX())/2;
			double y = (s1.midASV.getY() + s2.midASV.getY())/2;
			List<Double> booms = new ArrayList<Double>();
			List<Double> angles = new ArrayList<Double>();
			for (int j = 0; j < s1.getASVCount() - 1; j++){
				booms.add((s1.booms.get(j) + s2.booms.get(j))/2);
				angles.add((s1.angles.get(j) + s2.angles.get(j))/2);
			}
			ASVState newState = new ASVState(booms,angles,new Point2D.Double(x,y),s1.getASVCount());
			List<ASVState> path = new ArrayList<ASVState>();
			path.addAll(StateTools.interpolate(s1, newState));
			path.addAll(StateTools.interpolate(newState, s2));
			return path;
		}
		else {
			List<ASVState> path = new ArrayList<ASVState>();
			path.add(s1);
			return path;
		}	
	}
	
	/**
	 * recursive works?
	 * @param s1
	 * @param s2
	 * @param obs
	 * @return
	 */
	public static boolean hasDirectPath(ASVState s1, ASVState s2, List<Obstacle> obs){
		if (StateTools.maxDistance(s1, s2) > 0.001){
			double x = (s1.midASV.getX() + s2.midASV.getX())/2;
			double y = (s1.midASV.getY() + s2.midASV.getY())/2;
			List<Double> booms = new ArrayList<Double>();
			List<Double> angles = new ArrayList<Double>();
			for (int j = 0; j < s1.getASVCount() - 1; j++){
				booms.add((s1.booms.get(j) + s2.booms.get(j))/2);
				angles.add((s1.angles.get(j) + s2.angles.get(j))/2);
			}
			ASVState newState = new ASVState(booms,angles,new Point2D.Double(x,y),s1.getASVCount());
			if (!StateTools.isValidState(newState, obs))
				return false;
			else
				return StateTools.hasDirectPath(s1, newState, obs) && StateTools.hasDirectPath(newState, s2, obs);
			
		}
		else
			return true;
	}
	
	public static double getCost(ASVState s1, ASVState s2){
		List<Point2D> points1 = s1.getASVPositions();
		List<Point2D> points2 = s2.getASVPositions();
		double cost = 0.0;
		for (int i = 0; i < s1.getASVCount(); i++){
			cost += points1.get(i).distance(points2.get(i));
		}
		return cost;
	}
	
	public static double getPathCost(List<ASVState> path){
		double cost = 0.0;
		for (int i = 0; i < path.size()-1; i++){
			cost += StateTools.getCost(path.get(i), path.get(i+1));
		}
		return cost;
	}
	
	/**
	 * using createRandomState and ASVState.setASVPos methods
	 * @param noOfASV
	 * @param clockwise
	 * @param x
	 * @param y
	 * @return ASVState having 1st ASV with coordinate of (x,y)
	 */
	public static ASVState createRandomStateWithASVPos(int noOfASV, boolean clockwise, double x, double y) {
		List<Double> booms = new ArrayList<Double>();
		List<Double> angles = new ArrayList<Double>();
		
		double angle1 = (random.nextDouble() - 0.5) * Math.PI * 2;
		double boom1 = 0.05 + random.nextDouble()*0.025;
		
		booms.add(boom1);
		angles.add(angle1);
		
		double angleMean = (Math.PI * 2 / noOfASV)*0.75;
		
		if (!clockwise){
			for (int i = 0; i < noOfASV-2; i++){
				angles.add(angles.get(i) + StateTools.getGaussian(angleMean));
				booms.add(0.05 + random.nextDouble()*0.025);
			}
		}
		else{
			for (int i = 0; i < noOfASV-2; i++){
				angles.add(angles.get(i) - StateTools.getGaussian(angleMean));
				booms.add(0.05 + random.nextDouble()*0.025);
			}
		}
		return new ASVState(booms, angles, new Point2D.Double(x,y),noOfASV);
	}
	
	/**
	 * create 500 states for each pair of obstacles
	 * @param n
	 * @param clockwise
	 * @param obs
	 * @return list of valid ASVState
	 */
	public static List<ASVState> samplingInsidePassage(int n, boolean clockwise, List<Obstacle> obs){
		List<ASVState> states = new ArrayList<ASVState>();
		int size = obs.size();
		int stop = 1;
		if (size > 0)
			stop = 250*n/size;
		
		for (int i = 0; i < size; i++){
			for (int j = i; j < size; j++){
				int noOfValid = 0;
				for (int k = 0; k < 500*n/size; k++){
					double x1 = obs.get(i).getRect().getX() + random.nextDouble()*obs.get(i).getRect().getWidth();
					double x2 = obs.get(j).getRect().getX() + random.nextDouble()*obs.get(j).getRect().getWidth();
					double y1 = obs.get(i).getRect().getY() + random.nextDouble()*obs.get(i).getRect().getHeight();
					double y2 = obs.get(j).getRect().getY() + random.nextDouble()*obs.get(j).getRect().getHeight();
					double xmid = (x1 + x2)/2 + (random.nextDouble() - 0.5)*0.005;
					double ymid = (y1 + y2)/2 + (random.nextDouble() - 0.5)*0.005;
					ASVState state = StateTools.createRandomStateWithASVPos(n, clockwise, xmid, ymid);
					if (StateTools.isValidState(state, obs)){
						states.add(state);
						noOfValid++;
					}
					if (noOfValid == stop) break;
				}
			}
		}
		return states;
	}
	
	public static List<ASVState> samplingAroundObs(int asvCount, boolean clockwise, List<Obstacle> obs){
		List<ASVState> states = new ArrayList<ASVState>();
		int noOfObs = obs.size();
		for (int i = 0; i < noOfObs; i++){
			double x = obs.get(i).getRect().getX();
			double y = obs.get(i).getRect().getY();
			double w = obs.get(i).getRect().getWidth();
			//double h = obs.get(i).getRect().getHeight();
			int stop = 0;
			for (int j = 0; j < 100*w*asvCount; j++){
				double x1 = x + random.nextDouble()*w;
				double y1 = y - 0.0001 - random.nextDouble()*0.001;
				ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x1, y1);
				if (StateTools.isValidState(state, obs)){
					states.add(state);
					stop++;
				}
				if (stop == 25*w*asvCount) break;
			}
		}
		for (int i = 0; i < noOfObs; i++){
			double x = obs.get(i).getRect().getX();
			double y = obs.get(i).getRect().getY();
			double w = obs.get(i).getRect().getWidth();
			double h = obs.get(i).getRect().getHeight();
			int stop = 0;
			for (int j = 0; j < 200*h*asvCount; j++){
				double x1 = x + w + 0.0001 + random.nextDouble()*0.001;
				double y1 = y + random.nextDouble()*h;
				ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x1, y1);
				if (StateTools.isValidState(state, obs)){
					states.add(state);
					stop++;
				}
				if (stop == 50*h*asvCount) break;
			}
		}
		for (int i = 0; i < noOfObs; i++){
			double x = obs.get(i).getRect().getX();
			double y = obs.get(i).getRect().getY();
			//double w = obs.get(i).getRect().getWidth();
			double h = obs.get(i).getRect().getHeight();
			int stop = 0;
			for (int j = 0; j < 200*h*asvCount; j++){
				double x1 = x - 0.0001 - random.nextDouble()*0.001;
				double y1 = y + random.nextDouble()*h;
				ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x1, y1);
				if (StateTools.isValidState(state, obs)){
					states.add(state);
					stop++;
				}
				if (stop == 50*h*asvCount) break;
			}
		}
		for (int i = 0; i < noOfObs; i++){
			double x = obs.get(i).getRect().getX();
			double y = obs.get(i).getRect().getY();
			double w = obs.get(i).getRect().getWidth();
			double h = obs.get(i).getRect().getHeight();
			int stop = 0;
			for (int j = 0; j < 200*w*asvCount; j++){
				double x1 = x + random.nextDouble()*w;
				double y1 = (y + h + 0.0001) + random.nextDouble()*0.001;
				ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x1, y1);
				if (StateTools.isValidState(state, obs)){
					states.add(state);
					stop++;
				}
				if (stop == 50*h*asvCount) break;
			}
		}
		return states;
	}
	
	public static List<ASVState> samplingOnBoundary(int asvCount, boolean clockwise, List<Obstacle> obs){
		List<ASVState> states = new ArrayList<ASVState>();
		double x;
		double y;
		int stop = 75*asvCount;
		int noOfValid = 0;
		for (int i = 0; i < 200*asvCount; i++){
			y = 0.0001;
			x = random.nextDouble();
			ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x, y);
			if (StateTools.isValidState(state, obs)){
				states.add(state);
				noOfValid++;
			}
			if (noOfValid == stop) break;
		}
		noOfValid = 0;
		for (int i = 0; i < 200*asvCount; i++){
			y = 0.0009;
			x = random.nextDouble();
			ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x, y);
			if (StateTools.isValidState(state, obs)){
				states.add(state);
				noOfValid++;
			}
			if (noOfValid == stop) break;
		}
		noOfValid = 0;
		for (int i = 0; i < 200*asvCount; i++){
			y = random.nextDouble();
			x = 0.0001;
			ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x, y);
			if (StateTools.isValidState(state, obs)){
				states.add(state);
				noOfValid++;
			}
			if (noOfValid == stop) break;
		}
		noOfValid = 0;
		for (int i = 0; i < 200*asvCount; i++){
			y = random.nextDouble();
			x = 0.0009;
			ASVState state = StateTools.createRandomStateWithASVPos(asvCount, clockwise, x, y);
			if (StateTools.isValidState(state, obs)){
				states.add(state);
				noOfValid++;
			}
			if (noOfValid == stop) break;
		}
		return states;
	}
	
	public static boolean isValidState(ASVState s, List<Obstacle> obstacles) {

		List<Point2D> asvPos = s.getASVPositions();
		int size = asvPos.size();
		
		// Check bound
		if (!StateTools.fitsBounds(new ArrayList<Point2D>(asvPos)))
			return false;
		
		// Check collision
		if (hasCollision(new ArrayList<Point2D>(asvPos),new ArrayList<Obstacle>(obstacles)))
			return false;

		if (!isConvex(new ArrayList<Point2D>(asvPos)))
			return false;
		
		// Checking area
		if (!hasEnoughArea(new ArrayList<Point2D>(asvPos),size))
			return false;
		
		return true;
	}
	
	public static boolean hasValidBoomLengths(List<Point2D> points) {
		for (int i = 1; i < points.size(); i++) {
			Point2D p0 = points.get(i - 1);
			Point2D p1 = points.get(i);
			double boomLength = p0.distance(p1);
			if (boomLength < MIN_BOOM_LENGTH - maxError) {
				return false;
			} else if (boomLength > MAX_BOOM_LENGTH + maxError) {
				return false;
			}
		}
		return true;
	}
	
	public static boolean isConvex(List<Point2D> asvPos) {
		List<Point2D> points = asvPos;
		if (points.size() == 3) return true;
		points.add(points.get(0));
		points.add(points.get(1));

		double requiredSign = 0;
		double totalTurned = 0;
		Point2D p0 = points.get(0);
		Point2D p1 = points.get(1);
		double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
		for (int i = 2; i < points.size(); i++) {
			Point2D p2 = points.get(i);
			double nextAngle = Math.atan2(p2.getY() - p1.getY(),
					p2.getX() - p1.getX());
			double turningAngle = normaliseAngle(nextAngle - angle);

			if (turningAngle == Math.PI) {
				return false;
			}

			totalTurned += Math.abs(turningAngle);
			if (totalTurned > 3 * Math.PI) {
				return false;
			}

			double turnSign;
			if (turningAngle < -maxError) {
				turnSign = -1;
			} else if (turningAngle > maxError) {
				turnSign = 1;
			} else {
				turnSign = 0;
			}

			if (turnSign * requiredSign < 0) {
				return false;
			} else if (turnSign != 0) {
				requiredSign = turnSign;
			}

			p0 = p1;
			p1 = p2;
			angle = nextAngle;
		}
		return true;
	}
	
	/**
	 * Returns whether the given configuration has sufficient area.
	 * 
	 * @param the postion of ASV 
	 * @return whether the given configuration has sufficient area.
	 */
	public static boolean hasEnoughArea(List<Point2D> points,int size) {
		double total = 0;
		points.add(points.get(0));
		points.add(points.get(1));
		for (int i = 1; i < points.size() - 1; i++) {
			total += points.get(i).getX()
					* (points.get(i + 1).getY() - points.get(i - 1).getY());
		}
		double area = Math.abs(total) / 2;
		return (area >= getMinimumArea(size));
	}
	
	/**
	 * Returns the minimum area required for the given number of ASVs.
	 * 
	 * @param asvCount
	 *            the number of ASVs
	 * @return the minimum area required.
	 */
	public static final double getMinimumArea(int asvCount) {
		double radius = 0.007 * (asvCount - 1);
		return Math.PI * radius * radius;
	}

	
	/**
	 * Creates a new Rectangle2D that is grown by delta in each direction
	 * compared to the given Rectangle2D.
	 * 
	 * @param rect
	 *            the Rectangle2D to expand.
	 * @param delta
	 *            the amount to expand by.
	 * @return a Rectangle2D expanded by delta in each direction.
	 */
	
	public static Rectangle2D grow(Rectangle2D rect, double delta) {
		return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
				rect.getWidth() + delta * 2, rect.getHeight() + delta * 2);
	}
	
	/**
	 * Returns whether the given config collides with the given obstacle.
	 * 
	 * @param cfg
	 *            the configuration to test.
	 * @param o
	 *            the obstacle to test against.
	 * @return whether the given config collides with the given obstacle.
	 */
	public static boolean hasCollision(List<Point2D> points, List<Obstacle> obs) {
		for(Obstacle o:obs){
			Rectangle2D lenientRect = grow(o.getRect(), -maxError);
			for (int i = 1; i < points.size(); i++) {
				if (new Line2D.Double(points.get(i - 1), points.get(i))
						.intersects(lenientRect)) {
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * Returns whether the given state is out of bound or not.
	 */
	static boolean checkBound(ASVState state) {
		List<Point2D> asvPos = state.getASVPositions();
		for (int i = 0; i < asvPos.size(); i++) {
			if (!BOUNDS.contains(asvPos.get(i)))
				return false;
		}
		return true;

	}
	
	/**
	 * Returns whether the given configuration fits wholly within the bounds.
	 * 
	 * @param cfg
	 *            the configuration to test.
	 * @return whether the given configuration fits wholly within the bounds.
	 */
	public static boolean fitsBounds(List<Point2D> ASVPos) {
		for (Point2D p : ASVPos) {
			if (!BOUNDS.contains(p)) {
				return false;
			}
		}
		return true;
	}

	public static double normaliseAngle(double angle) {
		while (angle <= -Math.PI) {
			angle += 2 * Math.PI;
		}
		while (angle > Math.PI) {
			angle -= 2 * Math.PI;
		}
		return angle;
	}
	
}