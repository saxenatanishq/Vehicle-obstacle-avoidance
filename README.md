# Obstacle Avoidance vehicle control

## Parameters

I have used 3 parameters here, which I have declared at the very starting of the code:

- **Velocity of Car** - In my code, I have kept the velocity of the car constant most of the time. Only the velocity of the car becomes zero when the distance between the car and the wall is less than some specific limit (wall margin)
- **Wall Margin** - When the car comes in close contact to the wall, the velocity of the car becomes zero momentarily and the car changes the direction.
- **L** - In F1tenth algorithm, L has a much more significance (there it is called Lookahead distance), but here in my algorithm, the only purpose L serves is that when the car is inside the circle of radius L of a waypoint, then it is considered that the car has reached that point. This is because it is practically not possible for the car to reach exactly at the waypoint, instead it gets near the waypoint. In this code, I have kept L = 0.5
- **Gain - Gain** is a proportional factor that determines how strongly an output responds to an input. Basically, the steering angle is multiplied by a factor called gain in the end. Higher gain makes the turns more sharp and lower gain makes it small

## Algorithm Overview

Since in the subtask 1 we do not have many obstacles in the car’s path, so I have used a relatively simple version of “**Path Pursuit**” algorithm. At any point of time, there is a **waypoint** which the car follows. As soon as the car reaches that point, it moves to the next waypoint (I have made this cyclic, so the simulation never ends by itself). At every frame (frame rate is 30hz), the car finds the angle at which it has to turn to reach the waypoint by using the formula:

<div align="center">
  <img src="https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/image.png?raw=true" width="500px">
</div>
Although this formula is not the actual path pursuit algorithm, but this one also works.

Also, I have multiplied the steering angle with the gain (=2) to make the turns quick.

## Using the LIDAR Data

I have used the Lidar’s Data to dodge a wall in case any wall comes in between. Although this method is not very good in many ways but works (I will use better methods in the next subtask). So the algorithm is that it considers the 180 degrees view in front of the car and finds the minimum of all the points to find the closest point towards which the car is moving. If the minimum distant point is closer than a certain limit (wall margin), then the car stops and rotates. Also, if the minimum distant point is in the left side of car, then car rotates towards right and vice-versa. The speed at which the car rotates is kept 10 rad/s. 

I have used this relatively simple mechanism to avoid obstacles, as the path followed by the car in subtask 1 does not actually has any obstacles in between. Actually, my code never came across any such case when the wall came in between, so to check if the algorithm is correct, I changed the points, and it was working fine.

## Problems Faced

### Omit counter

In some frames, there was some problem with the Lidar data as it returned “null” instead of correct data, in that case, whole frame is omitted and the omit counter in increased. For some reason, thsi happens in the starting of the simulation. In my case, it happened in 3-4 frames.

### Lidar Data Array


Conventionally, the lidar data is received in the form of an array where the mid-point of the array means the front of the car. But here, the array was arranged such that the array went full 360 degrees starting from front and ending in the front only, that too anticlockwise. It took me some time to identify what was the array returning

[Task2subtask1.mp4](https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/Task2subtask1.mp4)



# Designing an algorithm to dodge obstacles

## Problems in the standard algorithm

At first, I tried to implement the F1TENTH Follow the gap algorithm, but there were some problems with that algorithm

1. F1TENTH algorithm is made for straight path which is easy to follow and the Lidar data to be processed is only in front of the car. However, in our case the path is much more bigger as there is open field for the car to roam with obstacles in all directions (that too moving)

<div align="center">
  <div style="display: inline-block; margin: 0 10px;">
    <img src="https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/image3.png?raw=true" width="250px">
    <p style="margin-top: 10px; font-style: italic; color: #666;">F1/TENTH Follow the Gap Algorithm Path</p>
  </div>
  <div style="display: inline-block; margin: 0 10px;">
    <img src="https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/image%201.png?raw=true" width="250px">
    <p style="margin-top: 10px; font-style: italic; color: #666;">Our Path</p>
  </div>
</div>
1. F1Tenth algorithm does not have anything as a “target point”, it just goes forward following the path it has. But in our case, there are some target points which the car has to follow and the target points keep on changing as the car reaches there. This demands us to implement something so that we can bias the car to go towards a specefic target point instead of just finding the largest gap

## My solution

I have used the F1/TENTH algorithm as the basis of my solution, but I have made many additions to that algorithm.  First, I’ll explain what I mean with F1Tenth FTG algorithm:

1. First it collects the Lidar data in the form of an array and finds the minimum of those values.
2. **Make a bubble around minimum distant point**
It creates a bubble of radius **Rb** around the smallest point and makes all the values around it zero in the array. The radius Rb in our case is the width of the car (although we were not given the width, but I have taken it to be 0.5). The main purpose to do this is that if the car gets closer to that minimum distant point, then the length of the zero region in the Lidar data will increase, which will then force the car to take some other path.
3. **How to implement the above point?**
To implement the above point, I have used simple trigonometry

After this, F1Tenth algorithm says to simply follow the largest gap among all the gaps. But, I have made some changes here:

1. **Identify all the gaps (or obstacles):**
Then, it finds all the gaps by finding continuous lidar points. For example, if the lidar array is:
`[3.1, 3.2, 3.1, 3.3, 8.1, 8.2, 8.0, 8.1, 5.1, 5.4, 5.2]`
Then the gaps will be
`[3.1, 3.2, 3.1, 3.3]` , `[8.1, 8.2, 8.0, 8.1]` , `[5.1, 5.4, 5.2]`
This grouping is done as, the array pointer starts from zero and goes to the end of the array. If there is large gap between two consecutive values in the Lidar data, then it starts a new gap. This value which determines if the gap is large enough or not is a parameter named `“distance_threshold”` 
2. Then, for all the gaps it finds three parameters to tackle the three problems:

    1. Gap width - The gap should be wide enough for the car to go through, so the widest gap is preferred. Gap width is proportional to (end - start). Since proportionality is meaningless in comparisons, so I have assumed that the gap is simply the difference between the end and the start index of the respective gaps. 
    So, `gap_width = end-start`
    2. Gap Distance - Since the algorithm doesn't know if something is a gap or an obstacle, it can think that the obstacle is the widest gap and go towards it. It can also cause the car to think that the wall is the widest gap. So, if the distance from the gap is large, then the car should go there. Car should go towards the farthest gap. Algorithm assumes that the distance from the midpoint of the gap is the distance from that gap. 
    Here, `gap_distance = lidar_ranges[mid]`
    3. Angle - The car should prefer that gap which is closer to the target. So the angle between the target and the midpoint of the gap should be small. Smaller the "angle", car should prefer that gap. Here, 
    `angleGW = abs(angleCG - angleCW)`  where,
    `angleCW = math.atan2(dy, dx) - yaw` Angle between Car and Waypoint
    `angleCG = math.pi - (mid/number_of_readings)*(2*math.pi)` Angle between Car and Gap Midpoint
3. **Defining the potential:**
Now the problem is to make a value named “Potential” which can be used to determine where the car should go. So, the formula of potential I came up with is,

<div align="center">
  <img src="https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/Screenshot%202025-03-25%20104750.jpg?raw=true" width="900px">
</div>

Note that here `gap_width_weght` and `gap_distance_weight` and `angle_weight` are variables which are defined in the starting. They are used to vary the importance of any parameter to find the potential
4. **Which path to follow?**
After doing all this, finally the car follows the path which has the maximum potential.

## Radius of consideration

Even after this, there is a small problem with this algorithm. If there is an obstacle just behind the waypoint, then the potential of the waypoint will decrease and the car will noever go towards the waypoint. The path might look something like this:

<div align="center">
  <img src="https://github.com/saxenatanishq/Vehicle-obstacle-avoidance/blob/main/Photos%20and%20Videos/image%202.png?raw=true" width="500px">
</div>

To tackle this problem, the algorithm will consider only the obstacles which are in a radius of the distance between the car and the waypoint. So, the algorithm basically scales down all the Lidar Readings which are more than the distance between car and waypoint.

`lidar_ranges = np.clip(lidar_ranges, 0, distance)`

## Parameters

I have defined a total of 9 parameters:

- **Velocity of Car** - In my code, I have kept the velocity of the car constant most of the time. Only the velocity of the car becomes negative when the distance between the car and the wall is less than some specific limit (wall margin)
- **Wall Margin** - When the car comes in close contact to the wall, the velocity of the car becomes negative momentarily and the car changes the direction.
- **L** -  The only purpose L serves is that when the car is inside the circle of radius L of a waypoint, then it is considered that the car has reached that point. This is because it is practically not possible for the car to reach exactly at the waypoint, instead it gets near the waypoint.
- Radius of car - as mentioned above as well, it is the radius Rb or radius of bubble
- distance threshold - It is the threshold value that helps to differentiate gaps
- angle_weight
- gap_width_weight
- gap_distance_weight
- **Gain - Gain** is a proportional factor that determines how strongly an output responds to an input. Basically, the steering angle is multiplied by a factor called gain in the end. Higher gain makes the turns more sharp and lower gain makes it small

## Problems Faced

1. After some trials with this simulation, I understood that it was giving different visuals even when the code was kept the same. Also, when I used screen recorder app, the simulation was faulty, however when I turned off the screen recording, the simulation was relatively better. This shows that simulation depends on the state of CPU. So, I have skipped the testing part for now. By, “Testing part”, I mean trying out the code for different values of the variables and then coming up with the best variables values for the simulation to be smooth.
2. Also, because the code was not working that well because of the CPU load in my laptop, I optimized the using ChatGPT. It reduced the frame rate and instead of considering the LIDAR data at every frame, it did it once every 5 frames.





# Normalizing noisy LIDAR data

## Naive Approach Drawbacks

In order to filter out random fluctuations in the LIDAR data, we ca use this function to take the average of the neighboring data points and replace every point with its Neighbours’ average.

```python
def smooth_lidar_data(raw_data, window_size=5):
    smoothed_data = []
    n = len(raw_data)
    
    for i in range(n):
        window_start = max(0, i - window_size // 2)
        window_end = min(n, i + window_size // 2 + 1)
        window_values = [raw_data[j] for j in range(window_start, window_end)]
        # Filter out invalid readings (e.g., inf, NaN)
        valid_values = [v for v in window_values if not math.isinf(v) and not math.isnan(v)]
        if valid_values:
            smoothed_data.append(round((sum(valid_values) / len(valid_values)),2))
        else:
            smoothed_data.append(raw_data[i])  # Keep original if no valid neighbors
    
    return smoothed_data
```

1. But this solution possesses a problem that when there is actual jump in the data from some value to other, the algorithm will make that jump smooth as well. This can cause some issues in identifying the gaps part, because since the data is now smooth, actual gaps can’t ne found.
2. Second problem with this algorithm was that because of one random data in between the array, it deviated the neighboring data as well

For example, if the array is - 

```python
array = [3.1,3.2,3.1,3.2,3.3,3.3,**8.1**,3.3,3.2,3.1,3.2,3.1,7.7,7.8,7.7,7.9,7.8,7.8,7.8,7.8]

print(smooth_lidar_data(array))
# Output:
# [3.13,3.15,3.18,3.22,4.2,4.24,4.24,4.2,4.18,3.18,4.06,4.98,5.9,6.84,7.78,7.8,7.8,7.82,7.8]
```

As shown above, the point 8.1 caused the neighboring data to increase from 3 to 4. This can cause the car to misunderstand the obstacle and go in the wrong diresction. Also, when the array was put in the algorithm to find the gaps (`distance threshold = 1`), initially it could find 4 gaps that is

```python
number of gaps : 4
[3.1, 3.2, 3.1, 3.2, 3.3, 3.3]
[8.1]
[3.3, 3.2, 3.1, 3.2, 3.1]
[7.7, 7.8, 7.7, 7.9, 7.8, 7.8, 7.8]
```

But when the array was smoothed using this, it returned only one gap, that is:

```python
number of gaps : 1
[3.13, 3.15, 3.18, 3.22, 4.2, 4.24, 4.24, 4.2, 4.18, 3.18, 4.06, 4.98, 5.9, 6.84, 7.78, 7.8, 7.8, 7.82, 7.8]
```

So, this processing is totally wrong as we know that the actual number of gaps are 2.

## Better Approach

Instead of replacing each reading with its neighbors’ average, we can replace only those readings which are suspicious. To find the suspicious readings we can look at the left and right of each reading to see if it is close to its neighbors’ average or not. If it is not close, then we need to replace that one reading with the average of its neighbors. Also, when we are considering the neighbors, we will consider both of them separately. This is because in case of actual gap change, the reading will match with the average of either left or right but not both.

```python
def smooth_lidar_data(raw_data, window_size=5):
    smoothed_data = []
    n = len(raw_data)
    
    for i in range(n):
        window_start = max(0, i - window_size // 2)
        window_end = min(n, i + window_size // 2 + 1)
        left_values = [raw_data[j] for j in range(window_start, i)]
        right_values = [raw_data[j] for j in range(i+1, window_end)]
        
        if len(left_values) == 0: # if i == 0
            left_average = raw_data[i]
        else:
            left_average = round((sum(left_values)/len(left_values)),2)
        
        if len(right_values) == 0: # if i is in the ending
            right_average = raw_data[i]
        else:
            right_average = round((sum(right_values)/len(right_values)),2)
        
        if (abs(raw_data[i] - left_average) < tolerance) or (abs(raw_data[i] - right_average) < tolerance):
            smoothed_data.append(raw_data[i])
        else:
            smoothed_data.append(round((left_average+right_average)/2,2))   

    return smoothed_data

```

This code works fine for the above example as shown:

```python
array = [3.1,3.2,3.1,3.2,3.3,3.3,8.1,3.3,3.2,3.1,3.2,3.1,7.7,7.8,7.7,7.9,7.8,7.8,7.8,7.8]
new_array = smooth_lidar_data(array)
print(new_array)

# Output - 
# [3.1, 3.2, 3.1, 3.2, 3.3, 3.3, 3.27, 3.3, 3.2, 3.1, 3.2, 3.1, 7.7, 7.8, 7.7, 7.9, 7.8, 7.8, 7.8, 7.8]
```

After smoothing this array, when it was passed in the algorithm to find the gaps, it returned 2 gaps. which we know is the correct answer as shown:

```python
number of gaps : 2
[3.1, 3.2, 3.1, 3.2, 3.3, 3.3, 3.27, 3.3, 3.2, 3.1, 3.2, 3.1]
[7.7, 7.8, 7.7, 7.9, 7.8, 7.8, 7.8, 7.8]
```

So this approach works fine with real data as tested above and thus it can be used in our case to deal with noisy data points from LIDAR reading
