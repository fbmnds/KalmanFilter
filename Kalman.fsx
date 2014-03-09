#load "FSharpChart.fsx"
open MSDN.FSharp.Charting

type KalmanMatrix<[<Measure>]'x,[<Measure>]'t> = 
    { P_00: float<'x>
      P_01: float<'x/'t>
      P_10: float<'x/'t>
      P_11: float<'x/('t*'t)> }


type KalmanFilterState<[<Measure>]'x,[<Measure>]'t> = 
    { Qx: float<'x/'t>
      Qxdot: float<'x/('t*'t*'t)>
      R: float<'x>
      x: float<'x>
      xdot: float<'x/'t> 
      dt: float<'t>
      P: KalmanMatrix<'x,'t> } 

let P0 = { P_00 = 0.<_>; P_01 = 0.<_>; P_10 = 0.<_>; P_11 = 0.<_> }

let setOrDefault x defaultValue = match x with | Some x -> x | _ -> defaultValue

let setKalmanFilter<[<Measure>]'x,[<Measure>]'t> qx qxdot r x xdot dt P= 
    let kf : KalmanFilterState<'x,'t> = 
        { Qx    = setOrDefault qx    0.<_>;
          Qxdot = setOrDefault qxdot 0.<_>;
          R     = setOrDefault r     0.<_>;
          x     = setOrDefault x     0.<_>;
          xdot  = setOrDefault xdot  0.<_>; 
          dt    = setOrDefault dt    0.<_>;
          P     = setOrDefault P     P0 }
    kf 


let applyKalmanFilter<[<Measure>]'x,[<Measure>]'t> (kf: KalmanFilterState<'x,'t>) (xRaw: float<'x>) (xdotRaw:float<'x/'t>) = 
    // KasBot V2 - Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    //* Step 1 */
    let mutable xdotHat = xdotRaw - kf.xdot
    let mutable xHat = kf.x + kf.dt * xdotHat

    // Update estimation error covariance - Project the error covariance ahead
    //* Step 2 */
    let mutable P_00 = kf.P.P_00 + kf.dt * (kf.dt*kf.P.P_11 - kf.P.P_01 - kf.P.P_10 + kf.Qx)
    let mutable P_01 = kf.P.P_01 - kf.dt * kf.P.P_11
    let mutable P_10 = kf.P.P_10 - kf.dt * kf.P.P_11
    let mutable P_11 = kf.P.P_11 + kf.Qxdot * kf.dt

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    //* Step 4 */
    let s = P_00 + kf.R
    //* Step 5 */
    let k_0 = P_00 / s
    let k_1 = P_10 / s

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    //* Step 3 */
    let y = xRaw - xHat
    //* Step 6 */
    xHat <- xHat + k_0 * y
    xdotHat <- kf.xdot + k_1 * y

    // Calculate estimation error covariance - Update the error covariance
    //* Step 7 */
    P_00 <- P_00 * (1. - k_0)
    P_01 <- P_01 * (1. - k_0)
    P_10 <- P_10 - k_1 * P_00
    P_11 <- P_11 - k_1 * P_01
    
    let kfNew = 
        { Qx    = kf.Qx
          Qxdot = kf.Qxdot
          R     = kf.R
          x     = xHat
          xdot  = xdotHat
          dt    = kf.dt
          P     = { P_00 = P_00; P_01 = P_01; P_10 = P_10; P_11 = P_11 } }
    kfNew


[<Measure>] type m;;
[<Measure>] type s;;

let kf = 
    { Qx    = 0.001<m/s>
      Qxdot = 0.003<m/(s*s*s)>
      R     = 0.03<m>
      x     = 0.<m>
      xdot  = 0.<m/s>
      dt    = 1.<s>
      P     = P0 }


let rnd = System.Random()
let rand() = rnd.NextDouble()

let randomTrend1 = [for i in 0. .. 0.1 .. 10. -> i, sin i + rand()]
let randomTrend2 = [for i in 0. .. 0.1 .. 10. -> i, cos i + rand()]

let damp x = if x > 5. then 5. else if x < -5. then -5. else x

let fillRandomKF kf (x: (float * float) list) (xdot: (float * float) list) = 
    let A = Array.init 101 (fun i -> kf)
    for i in 1 .. 100 do
        A.[i] <- applyKalmanFilter<m,s> A.[i-1] (1.0<m>*(snd x.[i])) (1.0<m/s>*(snd xdot.[i]))
    A

let randomKF = fillRandomKF kf randomTrend1 randomTrend2

let randomKF_x = [for i in 0 .. 100 -> (float i)/10., float randomKF.[i].x]
let randomKF_xdot = [for i in 0 .. 100 -> (float i)/10., float randomKF.[i].xdot]

type Chart = FSharpChart
Chart.Combine [ Chart.Line randomTrend1; Chart.Line randomTrend2; Chart.Line randomKF_x; Chart.Line randomKF_xdot ]
Chart.Combine [ Chart.Line randomKF_x; Chart.Line randomKF_xdot ]
Chart.Combine [ Chart.Line randomTrend1; Chart.Line randomKF_x ]
Chart.Combine [ Chart.Line randomTrend2; Chart.Line randomKF_xdot ]




