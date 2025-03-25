tolerance = 0.5

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

array = [3.1,3.2,3.1,3.2,3.3,3.3,8.1,3.3,3.2,3.1,3.2,3.1,7.7,7.8,7.7,7.9,7.8,7.8,7.8,7.8]
new_array = smooth_lidar_data(array)
print(new_array)

# Output - 
# [3.1, 3.2, 3.1, 3.2, 3.3, 3.3, 3.27, 3.3, 3.2, 3.1, 3.2, 3.1, 7.7, 7.8, 7.7, 7.9, 7.8, 7.8, 7.8, 7.8]