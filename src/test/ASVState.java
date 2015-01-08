package test;

import java.util.ArrayList;
import java.util.List;
import java.awt.geom.Point2D;

import search.StateWithMap;


public class ASVState extends StateWithMap {
	private int noOfAsv;
	public Point2D firstASV;
	public Point2D midASV;
	public List<Double> booms ;
	public List<Double> angles;
	
	/*ASVState(int n, List<Double> booms, List<Double> angles, Point2D firstasv){
		int mid = (int) Math.ceil(noOfAsv/2-1);
		double x = firstasv.getX();
		double y = firstasv.getY();
		for (int i = 0; i <= mid; i++){
			x = x + booms.get(i)*Math.cos(angles.get(i));
			y = y + booms.get(i)*Math.sin(angles.get(i));
		}
		Point2D midPos = new Point2D.Double(x, y);
		this.midASV = midPos;
		this.noOfAsv = n;
		this.booms = booms;
		this.angles = angles;
		this.firstASV = firstasv;
	}*/
	
	/**
	 * create from mid asv
	 * @param booms
	 * @param angles
	 * @param asv
	 * @param n
	 */
	ASVState(List<Double> booms, List<Double> angles, Point2D midasv, int n){
		this.booms = booms;
		this.noOfAsv = n;
		this.angles = angles;
		this.midASV = midasv;
		int mid = (int) Math.ceil(n/2-1);
		double x = midasv.getX();
		double y = midasv.getY();
		for (int i = mid; i >= 0; i--){
			x = x - booms.get(i)*Math.cos(angles.get(i));
			y = y - booms.get(i)*Math.sin(angles.get(i));
		}
		Point2D firstPos = new Point2D.Double(x, y);
		this.firstASV = firstPos;
	}
	
	public List<Point2D> getASVPositions(){
		List<Point2D> points = new ArrayList<Point2D>();
		points.add(this.firstASV);
		for (int i = 0; i < noOfAsv - 1; i++){
			points.add(new Point2D.Double(points.get(i).getX() + booms.get(i)*Math.cos(angles.get(i)),
					points.get(i).getY() + booms.get(i)*Math.sin(angles.get(i))));
		}
		return points;
	}
	
	public ASVState(double[] coords) {
		// Calculate boom length
		this.firstASV = new Point2D.Double(coords[0], coords[1]);
		this.noOfAsv = coords.length;
		booms = new ArrayList<Double>();
		double x1, y1, x2, y2;
		for (int i = 0; i < coords.length - 3; i++) {
			x1 = coords[i];
			y1 = coords[i + 1];
			x2 = coords[i + 2];
			y2 = coords[i + 3];
			booms.add(Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2)));
		}
		//Calculate first angel between two first asv and horizontal line
		double signAngel;
		x1 = coords[1];
		y1 = coords[2];
		x2 = coords[3];
		y2 = coords[4];
		signAngel = Math.atan2(1, 0) - Math.atan2(x2 - x1, y2 - y1);
		
		
		// Calculate the angle
		
		for (int i = 0; i < coords.length - 3; i++) {
			x1 = coords[i];
			y1 = coords[i + 1];
			x2 = coords[i + 2];
			y2 = coords[i + 3];
			signAngel = Math.atan2(1, 0) - Math.atan2(x2 - x1, y2 - y1);
			if (signAngel > Math.PI)
				signAngel -= 2 * Math.PI;
			else if (signAngel < -(Math.PI))
				signAngel += 2 * Math.PI;
			angles.add(signAngel);
		}

	}

	public ASVState(int asvCount, String str) {
		
		List<Double> booms = new ArrayList<Double>();
		List<Double> angles = new ArrayList<Double>();
		List<Point2D> points = new ArrayList<Point2D>();
		
		this.noOfAsv = asvCount;
		String[] tokens = str.trim().split("\\s+");
		this.firstASV = new Point2D.Double(Double.valueOf(tokens[0]), Double.valueOf(tokens[1]));
		
		for (int i = 0; i < asvCount*2; i = i + 2){
			points.add(new Point2D.Double(Double.valueOf(tokens[i]), Double.valueOf(tokens[i+1])));
		}
		
		// initializing
		
		Point2D point1 = points.get(0);
		Point2D point2 = points.get(1);
		Point2D point3 = points.get(2);
		double x1 = point1.getX();
		double y1 = point1.getY();
		double x2 = point2.getX();
		double y2 = point2.getY();
		double x3 = point3.getX();
		double y3 = point3.getY();
		
		double angle1 = Math.atan2(y2-y1,x2-x1);
		double angle2 = Math.atan2(y3-y2,x3-x2);

		System.out.println(angle1);
		System.out.println(angle2);
		
		if (Math.abs(angle1 - angle2) > Math.PI){
			if (angle1 < 0){
				angle1 = Math.PI*2 + angle1;
			}
			if (angle2 < 0){
				angle2 = Math.PI*2 + angle2;
			}
		}
		
		angles.add(angle1);
		angles.add(angle2);
		booms.add(point1.distance(point2));
		booms.add(point2.distance(point3));
		
		boolean clockwise;
		
		if (angle2 - angle1 > 0)
			clockwise = false;
		else
			clockwise = true;
		
		if (asvCount > 3){
			for (int i = 2; i < asvCount-1; i++){
				point1 = points.get(i);
				point2 = points.get(i+1);
				x1 = point1.getX();
				y1 = point1.getY();
				x2 = point2.getX();
				y2 = point2.getY();
				booms.add(point1.distance(point2));
				angle1 = Math.atan2(y2-y1,x2-x1);
				if (clockwise){										//decrease
					if (angles.get(angles.size()-1) > angle1){
						angles.add(angle1);
					}
					else 
						angles.add(-Math.PI*2+angle1);
				}
				else {												//increase
					if (angles.get(angles.size()-1) < angle1){
						angles.add(angle1);
					}
					else {
						angles.add(Math.PI*2+angle1);
					}
				}
			}
		}
		this.angles = angles;
		this.booms = booms;
		int mid = (int) Math.ceil(noOfAsv/2-1);
		double x = this.firstASV.getX();
		double y = this.firstASV.getY();
		for (int i = 0; i <= mid; i++){
			x = x + booms.get(i)*Math.cos(angles.get(i));
			y = y + booms.get(i)*Math.sin(angles.get(i));
		}
		Point2D midPos = new Point2D.Double(x, y);
		this.midASV = midPos;
	}

	public String toString() {
		StringBuilder sb = new StringBuilder();
		List<Point2D> asvPositions = new ArrayList<Point2D>();
		asvPositions.add(this.firstASV);
		for (int i = 0; i < noOfAsv - 1; i++) {
			asvPositions.add(new Point2D.Double(asvPositions.get(i).getX()
					+ booms.get(i) * Math.cos(angles.get(i)), asvPositions.get(
					i).getY()
					+ booms.get(i) * Math.sin(angles.get(i))));
		}
		for (Point2D point : asvPositions) {
			if (sb.length() > 0) {
				sb.append(" ");
			}
			sb.append(point.getX());
			sb.append(" ");
			sb.append(point.getY());
		}
		return sb.toString();
	}

	public int getASVCount() {
		return noOfAsv;
	}

}
