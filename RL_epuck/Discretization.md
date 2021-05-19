
# States discretization

## Distances

Sharp GP2Y0A41SK0F range from 4 to 30 cm
distances = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
4 states -> very close, close, good, far

**Distances Discretization**
- Very close:  $<$ 6
- Close: 6 $\le$ d $<$ 10
- OK: 10 $\le$ d $<$ 15
- Good: 15 $\le$ d $<$ 20
- Very Good: 20 $\le$ d $<$ 25
- Far:  $>$ 25


## Robot actions discretization
Epuck Max speed: 6.28
Default speed will be 2
**Actions** -> Speed applied to wheel's motors 
- **Front** -> Same speed in both wheels -> (2,2)
- **Light Left** -> slightly less speed in Left Wheel -> (1.5,2)
- **Light Right** -> slightly less speed in Right Wheel -> (2,1.5)
- **Mid Left** -> slightly less speed in Left Wheel -> (1, 2)
- **Mid Right** -> slightly less speed in Right Wheel -> (2, 1)
- **Left** -> half speed in Left wheel -> (0,2)
- **Right** -> half speed in Right wheel -> (2,0)
- **Hard Left** -> 0 speed in Left wheel -> (-2,2)
- **Hard Right** -> 0 speed in Right wheel -> (2,-2)

# Total
 4 distance states
 9 action states
**Table Size** = $4^6*9=4096*9=36864$
