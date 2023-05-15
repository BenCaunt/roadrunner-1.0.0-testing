package org.firstinspires.ftc.teamcode.AStar;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.Vector2d;


import java.util.ArrayList;

public class Main {


	@RequiresApi(api = Build.VERSION_CODES.N)
	public static void main(String[] args) {

		AStar astar = new AStar(new Vector2d(36,48), new Vector2d(0,0));

		ArrayList<Vector2d> path = astar.computeAStar();
		if (path.isEmpty()) return;
		for (Vector2d n: path) {
			System.out.println(n);
		}
		System.out.println("path length" + path.size());
		System.out.println("last coordinate " + path.get(path.size() -1));
	}

}
