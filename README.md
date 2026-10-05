# Simplified-FSAI-Path-Planning-Task

This program is designed to help a car navigate through yellow and blue cones similar to FSAI.

---

## Overview & Core Execution Flow

The path generation follows a simple step-by-step sequence:

1. The program gets the midpoints to follow (will be discussed later).
2. The program then loops every midpoint and plots the path towards it from the previous point of the car.
3. If there are no points the car will move forward for 7 metres (it will plot a virtual point 7 paths ahead of it so that it follows it).
4. All the path points are then added to a `Path2D` list which is then returned by the function.

```python
def generatePath(self) -> Path2D:
    midpoints = self.get_midpoints()

    path = []

    if midpoints:
        for midpoint in midpoints:
            self.follow_point(midpoint, path)
    else:
        target = (self.car_pose.x + 7 * math.cos(self.car_pose.yaw), self.car_pose.y + 7 * math.sin(self.car_pose.yaw))
        self.follow_point(target, path)
    return path

```

---

## How the Midpoints are Generated

Now this has 2 main cases:

### Case 1: All Cones Have Corresponding Opposite-Colored Cones

In this case the program separates cones into pairs and calculates the midpoint between the 2 cones and follows it.

### Case 2: Uneven Number of Cones (Lonely Cones)

This case is more complex than the previous because there are many assumptions to be made and it's also separated into 2 cases:

* **a. Only one side/color of cones is available:**
In this case the program uses an assumption that the track width is 2 metres and uses it to plot a point shifted from the cone in a certain direction by half the track width.
* **b. Both sides are available but one side has more cones than the other:**
In this case the program uses an assumption that the distance between 2 cones is the same until stated otherwise (because some scenarios stated otherwise). So what the program will do is that it will take the distance between any random yellow-blue cone pair and plot a point shifted from the lonely cone in a certain direction by using half the distance between 2 cones.

```python
def get_midpoints(self):
    cone_pairs = self.get_all_cone_pairs()
    midpoints = []
    lonely_cones = self.sort_cones(self.get_lonely_cones())
    for cone_pair in cone_pairs:
        midpoints.append(((cone_pair[0].x + cone_pair[1].x) / 2, (cone_pair[0].y + cone_pair[1].y) / 2))

    if len(self.cones) == len(lonely_cones):
        midpoints.extend(self.get_lonely_midpoints(lonely_cones))
    else: 
        pair = self.get_any_cone_pair()
        scenario_track_width = ((pair[0].x - pair[1].x)**2 + (pair[0].y - pair[1].y)**2)**0.5
        midpoints.extend(self.get_lonely_midpoints(lonely_cones, scenario_track_width))
                
    return self.sort_midpoints(midpoints)

```

---

## Shifting Direction Logic for Lonely Cones

Now as I said earlier in the second case where the number of cones is uneven the point is shifted from the cone towards a certain direction, but how do we know that direction?

The solution implemented here is simple and works in these scenarios, but it would not work efficiently in all real life cases.

Basically we start with the point of the cone and its coordinates $x$ and $y$, then we shift them by using `shift_values[0]` and `shift_values[1]` where shift values 0 and 1 are the shift values for $x$ and $y$ respectively.

To get these values:

* We check for any adjacent point near the cone (one with the same $x$ or the same $y$), even if the point is the car itself.
* If they're on the same $x$ axis, the point is shifted to the right if blue and to the left if yellow.
* If both cones have the same $y$ value, then the point is shifted downwards if blue and upwards if yellow.
* Other than that, if there aren't adjacent cones, we resort to the default case which just shifts the point upwards for yellow cones and downwards for blue cones and that's it.

Of course, this doesn't work for all real life scenarios, but as for the virtual scenarios in this task they do work well.

```python
def get_shift_values(self, cone, cone_list, track_width):
    shift_values = [0, 0]
    cone_index = cone_list.index(cone)
    adjacent_point = (0, 0)
    if len(cone_list) == 1:
        adjacent_point = (self.car_pose.x, self.car_pose.y) 
    elif cone_index + 1 < len(cone_list):
        adjacent_point = (cone_list[cone_index + 1].x, cone_list[cone_index + 1].y)
    elif cone_index - 1 >= 0:
        adjacent_point = (cone_list[cone_index - 1].x, cone_list[cone_index - 1].y)

    if cone.color == blue:
        if cone.y == adjacent_point[1]:
            shift_values[1] = -track_width / 2
        elif cone.x == adjacent_point[0]:
            shift_values[0] = track_width / 2
        else:
            shift_values = [0, -track_width / 2]
    else:
        if cone.y == adjacent_point[1]:
            shift_values[1] = track_width / 2
        elif cone.x == adjacent_point[0]:
            shift_values[0] = -track_width / 2
        else:
            shift_values = [0, track_width / 2]
    return shift_values

```

---

## Cone and Midpoint Sorting Strategy

Another thing to consider is how to sort the cones because sometimes if we just use the shortest distance between the cone and the car then it could cause some errors, so we will focus on the main axis first which in this case is the $x$ axis—so the closest cones are the ones with the lowest $x$ distance from the car. If 2 cones have the same distance from the car, they are compared by using the shortest distance from the cone to the car in $x$ and $y$.

Of course this, just like the shifting values for lonely cones, won't work everytime, but it works well for the scenarios.

```python
def distance_to_car(self, x, y):
    return ((x - self.car_pose.x, y - self.car_pose.y, ((x - self.car_pose.x)**2 + (y - self.car_pose.y)**2)**0.5)) 

def sort_cones(self, cone_list):
    for i in range(len(cone_list) - 1):
        for j in range(len(cone_list) - 1):
            if self.distance_to_car(cone_list[j].x, cone_list[j].y)[0] > self.distance_to_car(cone_list[j+1].x, cone_list[j+1].y)[0]:
                cone_list[j], cone_list[j+1] = cone_list[j+1], cone_list[j]
            elif (self.distance_to_car(cone_list[j].x, cone_list[j].y)[0] == self.distance_to_car(cone_list[j+1].x, cone_list[j+1].y)[0]) and (self.distance_to_car(cone_list[j].x, cone_list[j].y)[2] > self.distance_to_car(cone_list[j+1].x, cone_list[j+1].y)[2]):
                cone_list[j], cone_list[j+1] = cone_list[j+1], cone_list[j]
    return cone_list

```

> **Note on Future Development:**
> Both fixes for these for future development would be to include the heading of the car and heading between the cones to identify both the shifting values and how the cones are sorted.

---

## Waypoint Following & Step Discretization

The program adds the path points simply by moving in increments of 0.05m between steps towards one point which could cause some overshooting in the paths like for the endpoint to be shifted slightly up if the path distance isn't divisible by 0.05m, but a simple solution would be to smooth the path or use smaller increments so that even if there is a little bit of overshoot it won't affect the path significantly (it already doesn't affect it significantly at this step value).

```python
def follow_point(self, point, path):
    sp = (self.car_pose.x, self.car_pose.y)

    if path: 
        sp = path[-1]

    target_angle = math.atan2(point[1] - sp[1], point[0] - sp[0])
    target_dist = math.hypot(abs(sp[1] - point[1]), abs(sp[0] - point[0]))

    step = 0.05
    max_steps = 10 / step
    taken_steps = 0
    cp = list(sp)
    while taken_steps < max_steps and taken_steps < target_dist / step:
        cp[0] += step * math.cos(target_angle)
        cp[1] += step * math.sin(target_angle)
        path.append((cp[0], cp[1]))
        taken_steps += 1

    dx = path[-1][0] - path[-2][0]
    dy = path[-1][1] - path[-2][1]
    target_angle = math.atan2(dy, dx)

```
