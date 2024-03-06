import matplotlib.pyplot as plt

# Initialize empty arrays for num1 and num2
num1_array = []
num2_array = []

# Open and read the file
with open('data2.txt', 'r') as file:
    for line in file:
        # Split the line on colon and strip spaces
        l = line.split(':')
        a = l[1].strip()
        print(a)
        num1_array.append(int(a))
        # num2_array.append(int(num2.strip()))

# Plotting
plt.figure(figsize=(10, 6)) # Set the figure size (optional)
plt.plot(num1_array, marker='o', linestyle='-', color='b') # You can customize this
plt.title('Plot of num1 vs num2')
plt.xlabel('num1')
plt.ylabel('num2')
plt.grid(True)
plt.show()

