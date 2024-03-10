import matplotlib.pyplot as plt

from typing import List

# Initialize empty arrays for num1 and num2
num1_array = []

# Open and read the file
with open('data.txt', 'r') as file:
    for line in file:
        # Split the line on colon and strip spaces
        l = line.split(':')
        a = l[1].strip()
        print(a)
        num1_array.append(float(a))
        # num2_array.append(int(num2.strip()))


def data_filtering_program(data: List[float]) -> List[float]:
    out_data = data[:]
    # for i in range(1, len(data)):
    #     if abs(data[i] - data[i-1]) > 3:
    #         out_data[i] = out_data[i-1]
    for i in range(15, len(data)):
        out_data[i] = median(data[i - 15:i])
    return out_data


def median(data: List[float]) -> float:
    data = sorted(data)
    if len(data) % 2 == 1:
        return (data[len(data) // 2] + data[len(data) // 2 + 1]) / 2
    else:
        return data[len(data) // 2]


num2_array = data_filtering_program(num1_array)
# num1_array = data_filtering_program(num1_array)
# num1_array = data_filtering_program(num1_array)
# num1_array = data_filtering_program(num1_array)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(num1_array, marker='o', linestyle='-', color='b')
plt.plot(num2_array, marker='o', linestyle='-', color='r')
plt.title('Plot of num1 vs num2')
plt.xlabel('num1')
plt.ylabel('num2')
plt.grid(True)
plt.show()
