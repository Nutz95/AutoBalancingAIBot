# Tuning Journal - AutoBalancingBot

## System Constants
- **Loop Frequency**: 500Hz (2ms)
- **Encoder Resolution**: 51,200 ticks/rev (Divided by 10 in logic)
- **Hardware**: MKS Servo 42D (RS485), BMI160 (Rigid mount)

---

## Evolution Log

| Version | Kp (Pitch) | Kg (Gyro) | Kd (Dist) | Ks (Speed) | Result | Observation |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **v40** | 0.08 | 0.02 | 0.0001 | 0.0 | **Unstable** | Soft balance, Adaptive Trim was too fast (0.001), robot "chased" itself. |
| **v41** | 0.12 | 0.02 | 0.0003 | 0.0 | **Runaway** | First use of Scale/10. $K_d$ was 30x too high, saturated terms at 15.0. |
| **v42** | 0.12 | 0.02 | 0.00001 | 0.0 | **Runaway** | Position anchor too soft to stop the drift, but term saturation was fixed. |
| **v43** | 0.10 | 0.04 | 5e-6 | 0.0001 | **Oscillation** | Violent 5.4Hz wobble. $K_s$ (Speed damping) too high for noisy derivatives. |
| **v44** | 0.15 | 0.05 | 1e-5 | 2e-5 | **Oscillation** | Constant hum/wobble. High resolution + loop latency = phase lag issues. |
| **v45** | 0.12 | 0.03 | 0.0 | 0.0 | **Suicide** | **Filter Sign Error**. Robot dove into the floor in 80ms. |
| **v47** | 0.08 | 0.02 | 5e-5 | 0.0 | **Wobbly** | Found physical bias at **13.7°**. Still oscillating (6Hz) and running away. |
| **v48.1** | 0.10 | 0.04 | 0.0001 | 0.0 | **Wobbly** | 1000Hz IMU / 500Hz Balancer. Better, but still 5.4Hz oscillation. |
| **v48.2** | 0.10 | 0.04 | 0.0001 | 0.0 | **Wobbly** | Full 1000Hz Sync. RS485 holds well. Oscillation slowed to 2.2Hz. |
| **v49** | 0.05 | 0.02 | 5e-5 | 0.0 | **Wobbly** | Halved gains to exit saturation. **Bug found**: Adaptive Trim sign was inverted, causing runaway. |
| **v50** | 0.04 | 0.015 | 2e-5 | 0.0 | **Saturated** | Correct sign. 15Hz oscillation (Phase Lag limit). Command hits +/- 1.0. Trim too slow. |
| **v51** | 0.035 | 0.012 | 1e-5 | 0.0 | **Worse** | Fell faster (0.8s). Gains too weak. Trim frozen by 15° safety limit. |
| **v52** | 0.042 | 0.008 | 5e-6 | 0.0 | **Wobbly** | Trim s'emballe pendant la chute (10°). Kg trop bas = manque de tenue. |
| **v53** | 0.045 | 0.012 | 5e-6 | 0.0 | **TBD** | Brushing trim (max 7° error). Remontée Kg pour amortir. |

---

## Technical Analysis: The 12-14Hz Mystery
The 12-14Hz (approx 70-80ms period) is the **Phase Lag limit** of the system.
- **Madgwick Cost**: We discovered the Madgwick filter was likely adding 10-20ms of group delay to the pitch estimation.
- **System Latency**: By switching to `COMPLEMENTARY1D`, we eliminate the filter lag. In v45, the filter was fighting itself; v46 fixes the gyro sign.

## Action Plan (Convergence Strategy)
To stop "shooting in the dark", we follow this sequence:

1. **Step 1: Pure Balance (Alpha stage - current V45)**
   - Target: Robot balances without vibrating, even if it moves forward/backward.
   - Using: `IMU_FILTER_COMPLEMENTARY` for zero-lag response.
   
2. **Step 2: Static COG Correction**
   - Once Step 1 is stable, find the **fixed** `pitch_offset` that stops the drift. 
   - *Do not use Adaptive Trim yet.*

3. **Step 3: Add Damping ($K_s$)**
   - Slowly add $K_s$ to resist forward movement (Viscous friction).

4. **Step 4: Enable Position Anchor ($K_d$) & Trim**
   - Last step to lock the robot to a single tile.
