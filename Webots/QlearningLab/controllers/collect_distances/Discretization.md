
# States discretization

## Distances

Sharp GP2Y0A41SK0F range from 4 to 30 cm
distances = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
6 states -> dangerously close, very close, close, good, far, very far

**Distances Discretization**
- Dangerously close: <5.99
- Very close: 6 to 7.99
- Close: 8 to 9.99
- Good: 10 to 15.99
- Far: 16 to 21.99
- Very Far: 22 to 30


## Robot actions discretization
Epuck Max speed: 6.28
Default speed will be 2
**Actions** -> Speed applied to wheel's motors 
- **Front** -> Same speed in both wheels -> (2,2)
- **Light Left** -> slightly less speed in Left Wheel -> (1.8,2)
- **Light Right** -> slightly less speed in Right Wheel -> (2,1.8)
- **Left** -> 0 speed in Left wheel -> (0,2)
- **Right** -> 0 speed in Right wheel -> (2,0)
- **Rotate Left** -> negative speed in Left Wheel -> (-2,2)
- **Rotate Right** -> negative speed in Right Wheel -> (2,-2)
- **Back** -> negative equal speed in both wheels -> (-2,-2)

# Total
 6 distance states
 8 action states
 Total of 48 states

# Reward
- Difference between error in previous position and error in current position 
	- Previous position was (x-0.1,y+0.2) -> equal to 0.3 error
	- Current position is (x, y+0.2) -> equal to 0.2 error
	- Reward of 0.3-0.2 = 0.1
- -1 if dangerously close
- -0.5 if very close
- -0.2 if close
- -0.1 For every action

