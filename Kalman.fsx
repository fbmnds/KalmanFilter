

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
    let mutable xdot = xdotRaw - kf.xdot
    let mutable x = kf.x + kf.dt * kf.xdot
    let mutable P_00 = kf.P.P_00 + kf.dt * (kf.dt*kf.P.P_11 - kf.P.P_01 - kf.P.P_10 + kf.Qx)
    let mutable P_01 = kf.P.P_01 - kf.dt * kf.P.P_11
    let mutable P_10 = kf.P.P_10 - kf.dt * kf.P.P_11
    let mutable P_11 = kf.P.P_11 + kf.Qxdot * kf.dt
    let s = kf.P.P_00 + kf.R
    let k_0 = kf.P.P_00 / s
    let k_1 = kf.P.P_10 / s
    let y = xRaw - kf.x
    x <- x + k_0 * y
    xdot <- xdot + k_1 * y
    P_00 <- P_00 * (1. - k_0)
    P_01 <- P_01 * (1. - k_0)
    P_10 <- P_10 - k_1 * kf.P.P_00
    P_11 <- P_11 - k_1 * kf.P.P_01
    let kfNew = 
        { Qx    = kf.Qx
          Qxdot = kf.Qxdot
          R     = kf.R
          x     = x
          xdot  = xdot
          dt    = kf.dt
          P     = { P_00 = P_00; P_01 = P_01; P_10 = P_10; P_11 = P_11 } }
    kfNew


[<Measure>] type m;;
[<Measure>] type s;;

let kf = 
    { Qx    = 0.001<m/s>
      Qxdot = 0.003<m/(s*s*s)>
      x     = 0.<m>
      xdot  = 0.<m/s>
      R     = 0.03<m>
      dt    = 1.<s>
      P     = P0 }


let rnd = System.Random()
let rand() = rnd.NextDouble()

let randomTrend1 = [for i in 0. .. 0.1 .. 10. -> i, sin i + rand()]
let randomTrend2 = [for i in 0. .. 0.1 .. 10. -> i, sin i * cos i + rand()]

let damp x = if x > 5. then 5. else if x < -5. then -5. else x

let randomKF = [for i in 0 .. 100 -> applyKalmanFilter<m,s> kf (1.0<m>*(snd randomTrend1.[i])) (1.0<m/s>*(snd randomTrend2.[i])) ]
let randomKF_x = [for i in 0 .. 100 -> (float i)/10., (damp (float randomKF.[i].x))]
let randomKF_xdot = [for i in 0 .. 100 -> (float i)/10., (float randomKF.[i].xdot)]

#load "FSharpChart.fsx"
open MSDN.FSharp.Charting

type Chart = FSharpChart
Chart.Combine [Chart.Line randomTrend1; Chart.Line randomTrend2; Chart.Line randomKF_x; Chart.Line randomKF_xdot ]



