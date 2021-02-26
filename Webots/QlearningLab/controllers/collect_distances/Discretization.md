
# States discretization

## Distances

Sharp GP2Y0A41SK0F range from 4 to 30 cm
distances = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
4 states -> very close, close, good, far

**Distances Discretization**
- Very close:  $<$ 7
- Close: 7 $\le$ d $<$ 13
- Good: 13 $\le$ d $<$ 19.99
- Far:  $>$ 20


## Robot actions discretization
Epuck Max speed: 6.28
Default speed will be 2
**Actions** -> Speed applied to wheel's motors 
- **Front** -> Same speed in both wheels -> (2,2)
- **Light Left** -> slightly less speed in Left Wheel -> (1.8,2)
- **Light Right** -> slightly less speed in Right Wheel -> (2,1.8)
- **Mid Left** -> slightly less speed in Left Wheel -> (1.4, 2)
- **Mid Right** -> slightly less speed in Right Wheel -> (2, 1.4)
- **Left** -> half speed in Left wheel -> (1,2)
- **Right** -> half speed in Right wheel -> (2,1)
- **Hard Left** -> 0 speed in Left wheel -> (0,2)
- **Hard Right** -> 0 speed in Right wheel -> (2,0)

# Total
 4 distance states
 9 action states
**Table Size** = $4^6*9=4096*9=36864$
