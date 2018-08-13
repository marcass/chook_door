# chook_door
Use a photosesor controlled flap to stop chickens shitting in laying box at night

## Notes
The opening threshold is needs to be lower than the closing threshold, so in order to calculate when to open or close I am using slope calculation
### Slope calculation

* Use FastRunningMedian.h to populate an array (75 members give an array over ~10min of meausrements)
* Compare current value to median value over last 10min
   * If current reading is < median slope is decreasing (moving towards darkness)
      * Higher threshold for closing well before dark
   * If current reading is > median slope is increasing (moving towards daylight)
      * Lower threshold so door opens at dawn
