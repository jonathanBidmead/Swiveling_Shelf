# Swivelling_Shelf

This project simlates a shelf in a cabinet which is able to translate and rotate along an arbitrary path. It uses this simulation to approximate an optimal dimension of shelf by ensuring that the outside edge of the shelf is always touching the cabinet in at least one place at every simulation time-step along its travel.

# Example Demonstration

In this example, the shelf is swiveling around a pivot in the bottom left corner of the cabinet, and simultaneously rotating from 0 degrees through to 70 degrees as it moves through that swivel point (as defined by the bezier curve in orange).

![](https://github.com/jonathanBidmead/Swiveling_Shelf/blob/main/rotating_shelf1.gif)

# How It Works

The algorithm which generates this shelf works in two discrete stages: Stage One is Raycasting, and Stage Two is Normal Vector Expansion (NVE).

A successful simulation requires a balance of both sections. Raycasting by itself will work, and for the case of a path with zero rotation performs as well as NVE. However, it is unable to create concavities in the shelf geometry. For that, a secondary stage like NVE is necessary. 

However, NVE requires an intial set of points to iterate off of. In fact, if too few angles are checked in the Raycasting stage (to save simulation time), the NVE tends to become unstable and place points that don't make physical sense.

## Stage One: Raycasting

In this first stage, a draft outline of the shelf is created by casting rays from the line of travel and checking their intersections with the cabinet walls.

1.	Loop through the angles from 0 to 2*pi. For each of these angles, loop through all points on the translational line. For each, generate a vector at the current angle (plus any angle from the rotational path) and with a large magnitude. Get the nearest intersection with the cabinet walls.
3.	Once all the points on the line are looped through, record the shortest distance to a wall, and the point on the line where that occurred.
4.	Once all the shortest lengths are found for each angle, the initial outline of the shelf is formed. Draw vectors starting from the start of the translational path with angles and magnitudes as per the minimum lengths determined previously.

## Stage Two: Normal Vector Expansion

In some cases, raycasting generates obviously sub-optimal shelf designs. Its largest shortcoming is that it cannot place multiple points along the same angle starting from the initial point. This means that it is impossible to generate concave features in the shelf by that method alone.

Normal Vector Expansion builds on the initial outline created by raycasting, and is able to form these more difficult features. It works by looking at each pair of adjacent points, finding the normal vector at their midpoint, and computing the length to its nearest intersection with a wall.

1.	Loop through each pair of adjacent points in the shelf outline from stage one. If the distance between these points is above a threshold value, get the midpoint between them.
2.	Generate a new point by computing the intersection of the normal vector at the midpoint to each wall. Repeat along the whole path and record the smallest value.
3.	Continue this process until the distance between all adjacent points is below the threshold, or the iteration limit is reached.
4.	In the case where the midpoint is outside of the cabinet, reverse the direction of the normal vector.

