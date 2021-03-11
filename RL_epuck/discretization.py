import numpy as np

distances = np.arange(4,30.1,0.1).tolist() # distances 4 to 30 incremented 0.1
f_dists = [round(x,3) for x in distances]
print(f_dists)
states = 6 # 6 states -> dangerously close, very close, close, good, far, very far

def distances_discretization(distances, states):
    stepsize = int(np.floor(len(distances)/states))
    distances.sort()
    threshold = []
    print(len(distances))
    for i in range(0, states):
        print((i+1)*stepsize)
        threshold.append(distances[(i+1)*stepsize])
    return threshold

thresh = distances_discretization(f_dists, states)
print(thresh)   
# Result is uniform, not ideal (I think)
# [8.3, 12.6, 16.9, 21.2, 25.5, 29.8]


# Senor range from 4 to 30 cm
# distances = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
# 6 states -> dangerously close, very close, close, good, far, very far

# Distances Discretization
# Dangerously close: <5
# Very close: 6 to 8
# Close: 9 to 11
# Good: 12 to 16
# Far: 17 to 21
# Very Far: 22 to 30

# Robot actions discretization
# Actions -> Speed applied to wheel's motors
# Epuch has max speed of 6.28
# Default speed defined to 2

# Front -> Same speed in both wheels                (2,2)
# Light Left -> slightly less speed in Left Wheel   (1.8,2)
# Light Right -> slightly less speed in Right Wheel (2,1.8)
# Left -> 0 speed in Left wheel                     (0,2)
# Right -> 0 speed in Right wheel                   (2,0)
# Rotate Left -> negative speed in Left Wheel       (-2,2)
# Rotate Right -> negative speed in Right Wheel     (2,-2)
# Back -> negative equal speed in both wheels       (-2,-2)

# 6 distance states
# 8 action states
# Total of 48 states