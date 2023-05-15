package org.firstinspires.ftc.teamcode.AStar;


import static java.util.Collections.reverse;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.Vector2d;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.PriorityQueue;

@RequiresApi(api = Build.VERSION_CODES.N)
public class AStar {
	
	final int REASONABLE_INFINITY = 10000;

	Vector2d goal;
	Vector2d initial;
	HashMap<Vector2d, Vector2d> cameFrom = new HashMap<>();
	HashMap<Vector2d, Double> gScore = new HashMap<>();
	HashMap<Vector2d, Double> fScore = new HashMap<>();
	PriorityQueue<Vector2d> openSet = new PriorityQueue<>((point, t1) -> -Double.compare(getFScore(t1), getFScore(point)));

	public ArrayList<Circle> obstacles = new ArrayList<>();

	Vector2d[][] field = {
			{new Vector2d(-72, -60), new Vector2d(-48, -60), new Vector2d(-24, -60), new Vector2d(0, -60), new Vector2d(24, -60), new Vector2d(48, -60), new Vector2d(72, -60)},
			{new Vector2d(-72, -36), new Vector2d(-48, -36), new Vector2d(-24, -36), new Vector2d(0, -36), new Vector2d(24, -36), new Vector2d(48, -36), new Vector2d(72, -36)},
			{new Vector2d(-72, -12), new Vector2d(-48, -12), new Vector2d(-24, -12), new Vector2d(0, -12), new Vector2d(24, -12), new Vector2d(48, -12), new Vector2d(72, -12)},
			{new Vector2d(-72, 12), new Vector2d(-48, 12), new Vector2d(-24, 12), new Vector2d(0, 12), new Vector2d(24, 12), new Vector2d(48, 12), new Vector2d(72, 12)},
			{new Vector2d(-72, 36), new Vector2d(-48, 36), new Vector2d(-24, 36), new Vector2d(0, 36), new Vector2d(24, 36), new Vector2d(48, 36), new Vector2d(72, 36)},
			{new Vector2d(-72, 60), new Vector2d(-48, 60), new Vector2d(-24, 60), new Vector2d(0, 60), new Vector2d(24, 60), new Vector2d(48, 60), new Vector2d(72, 60)}
	};


	public AStar(Vector2d goal, Vector2d initial) {
		this.goal = goal;
		this.initial = initial;
	}


	public ArrayList<Vector2d> getNeighbors(Vector2d point) {

		int [][] directions = {
				{1,0},
				{-1,0},
				{0,1},
				{0,-1},
				{1,1},
				{1,-1},
				{-1,-1},
				{-1,1}
		};

		int indexX = (int)point.x + (field.length / 2);
		int indexY = (int)point.y + (field.length / 2);
		ArrayList<Vector2d> neighbors = new ArrayList<>();

		for (int[] direction : directions) {
			int newX = indexX + direction[0] ;
			int newY = indexY + direction[1] ;
			if (!(newX < 0 || newX > field.length  || newY < 0 || newY > field[0].length) ) {
					neighbors.add(new Vector2d(newX,newY));
			}
		}

		if (distanceTo(point,goal) < 12) {
			neighbors.add(new Vector2d(goal.x,goal.y));
		}
		if (distanceTo(point,initial) < 12) {
			neighbors.add(new Vector2d(initial.x,initial.y));
		}

		return neighbors;
	}





	/**
	 * straight line distance heuristic function
	 * @param n node we are calculating the weight of
	 * @return weight of n relative to the goal
	 */
	public double H(Vector2d n) {
		//return n.distanceTo(goal);
		return Math.abs(n.x - goal.x) + Math.abs(n.y - goal.y);
	}

	public ArrayList<Vector2d> computeAStar() {
		gScore.put(initial,0.0);
		fScore.put(initial, H(initial));
		openSet.add(initial);


		while (!openSet.isEmpty()) {

			Vector2d current = openSet.peek();
			assert current != null;
			openSet.remove(current);

			if (current.equals(goal)) {
				return reconstructPath(cameFrom, current);
			}


			ArrayList<Vector2d> neighbors = getNeighbors(current);

			System.out.println(neighbors.size());

			for (Vector2d neighbor: neighbors) {
				double tentative_gScore = getGScore(current) + distanceTo(current,neighbor);  //current.distanceTo(neighbor);
				if (tentative_gScore < getGScore(neighbor)) {
					cameFrom.put(neighbor,current);
					gScore.put(neighbor, tentative_gScore);
					fScore.put(neighbor, tentative_gScore + H(neighbor));
					if (!openSet.contains(neighbor)) {
						openSet.add(neighbor);
					}
				}
			}

		}

		return null;

	}

	protected double getGScore(Vector2d c){
		if (gScore.containsKey(c)) {
			return Objects.requireNonNull(gScore.get(c));
		}
		return REASONABLE_INFINITY;
	}

	protected double getFScore(Vector2d c){
		if (fScore.containsKey(c)) {
			return Objects.requireNonNull(fScore.get(c));
		}
		return REASONABLE_INFINITY;
	}


	protected ArrayList<Vector2d> reconstructPath(HashMap<Vector2d, Vector2d> cameFrom, Vector2d current) {
		ArrayList<Vector2d> reversedPath = new ArrayList<>();
		reversedPath.add(current);
		Vector2d c = new Vector2d(current.x,current.y);
		while (cameFrom.containsKey(c)) {
			c = new Vector2d(Objects.requireNonNull(cameFrom.get(c)).x, Objects.requireNonNull(cameFrom.get(c)).y);
			reversedPath.add(c);
		}
		// now the path is the correct direction
		reverse(reversedPath);
		return reversedPath;
	}

	public static double distanceTo(Vector2d current, Vector2d other) {
		return current.minus(other).norm();
	}


}
