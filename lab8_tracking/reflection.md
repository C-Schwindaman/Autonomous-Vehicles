1. Parameter tuning process and rationale

* r_pos: I started with the default r_pos of 0.5. I found this made the tracks very jittery and stick too closely to the noisy detections. I increased it to 1.0 to trust the motion model more, which resulted in smoother track paths in RViz.
* q: A low q value made my tracks very stable but slow to react to turns. I increased q to 0.75 to allow the track's velocity and yaw rate to change more quickly, which seemed to follow the turning cars better.
* update_threshold: The default of 10 seemed to work well. A lower value like 3 caused several ghost tracks to appear and disappear quickly. I kept it at 10 to ensure only persistent objects were confirmed.
* coast_threshold_confirmed: I increased this value from 15 to 25. This allowed the tracker to coast tracks for a longer period, successfully tracking a car even after it was hidden by another car for a few seconds.

2. Observed trade-offs between precision and recall

* Precision: To improve precision, I used a high update_threshold of 10. This meant the tracker would not create confirmed tracks for random, noisy detections, reducing the number of false positives.
* Recall: To improve recall, I increased the coast_threshold_confirmed to 25. This ensured that even if the LiDAR missed a car for a few frames (due to it being hidden or out of range), the track would not be dropped, allowing it to be recalled when the detection reappeared.
* Result: My final parameters favor recall slightly, as I felt it was more important to maintain tracks when hidden than to eliminate every possible false positive, which seemed rare.

3. Performance analysis and failure modes

* Overall Performance: The tracker performed well, smoothly following most cars as seen in RViz. The track boxes stayed locked onto the detection boxes.
* Failure Modes:
    * LiDAR Gaps: As expected, the tracker failed when cars entered the LiDAR's rear blind spot. Since no detections were received, the tracks would coast for a few frames and then be deleted.
    * High-Speed Turns: Initially, my tracks would lag behind cars in sharp turns. This was a failure of the constant speed model, which I partially fixed by increasing the process noise, q.
    * Associations: When two cars passed very close, one track briefly jumped to the wrong car, indicating a potential wrong association. This was rare with my final parameters.

4. Whether sensor fusion was attempted

* Sensor fusion was not attempted for this submission. The results are based on the LiDAR-only implementation.