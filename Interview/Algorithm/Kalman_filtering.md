### Kalman filtering
**Time Update("Predict")**

1. Project the state ahead
$$ \hat x_k^- = A\hat x_{k-1} + Bu_{k-1}$$
2. Project the error covariance ahead
$$P_k^- = AP_{k-1}A^T + Q $$

**Measurement Update("Correct")**

3. Compute the Kalman gain
$$K_k = P_k^-H^T(HP_k^-H^T + R)^{-1}$$
4. Update estimate with measurement $z_k$ 
$$\hat x_k = \hat x_k^- + K_k({z_k - H\hat x_k^-})$$
4. Update the error covariance
$$P_k = (I - K_kH)P_k^-$$

**Kalman Filter Algorithm Reference Terms**


|Variable|Introduction|Typy||
|--------|--------:|:--------:|:--------:|
|x|state variable|n x 1 column vector|Output|
|P|state covariance matrix|n x n matrix|Output|
|z|measurement|m x 1 column vector|Input|
|A|state transition matrix|n x n matrix|State-transition Model|
|B|control matrix|n x l matrix|Control-input model|
|H|state-to-measurement matrix|m x n matrix|Observation Model|
|R|measurement covariance matrix|m x m matrix|Input|
|Q|process noise covarivance matrix|n x n matrix|System Model|
|K|Kalman Gain|n x m|Internal|
|z-Hx|measurement residual|||

It implements the algorithm directly as found in [An Introduction to the Kalman Filter](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

More reference:
[Zhamemo](https://github.com/Zhamemo/Ubuntu/tree/master/Interview/Algorithm)
[Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter)
[Kalman Filter Explained Simply](https://thekalmanfilter.com/kalman-filter-explained-simply/)
[How a Kalman filter works in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)



