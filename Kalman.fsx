

type KalmanMatrix<[<Measure>]'x,[<Measure>]'t> = 
    { mutable P_00: float<'x>
      mutable P_01: float<'x/'t>
      mutable P_10: float<'x/'t>
      mutable P_11: float<'x/('t*'t)> }


type KalmanFilterState<[<Measure>]'x,[<Measure>]'t> = 
    { mutable Qx: float<'x/'t>
      mutable Qxdot: float<'x/('t*'t*'t)>
      mutable R: float<'x>
      mutable x: float<'x>
      mutable xdot: float<'x/'t> 
      mutable dt: float<'t>
      mutable P: KalmanMatrix<'x,'t> } 


let setKalmanFilter<[<Measure>]'x,[<Measure>]'t> qx qxdot r dt = 
    let kf : KalmanFilterState<'x,'t> = 
        { Qx = match qx with | Some qx -> qx | _ -> 0.0<_>;
          Qxdot = match qxdot with | Some qxdot -> qxdot | _ -> 0.0<_>;
          R = match r with | Some r -> r | _ -> 0.0<_>;
          x = 0.0<_>;
          xdot = 0.0<_>; 
          dt = match dt with | Some dt -> dt | _ -> 0.0<_>;
          P = { P_00 = 0.0<_>; P_01 = 0.0<_>; P_10 = 0.0<_>; P_11 = 0.0<_> } }
    kf 


let applyKalmanFilter<[<Measure>]'x,[<Measure>]'t> (kfPrev: KalmanFilterState<'x,'t>) (xRaw: float<'x>) (xdotRaw:float<'x/'t>) = 
    let kf = kfPrev
    kf.xdot <- xdotRaw - kf.xdot
    kf.x <- kf.x + kf.dt * kf.xdot
    kf.P.P_00 <- kf.P.P_00 + kf.dt * (kf.dt*kf.P.P_11 - kf.P.P_01 - kf.P.P_10 + kf.Qx)
    kf.P.P_01 <- kf.P.P_01 - kf.dt * kf.P.P_11
    kf.P.P_10 <- kf.P.P_10 - kf.dt * kf.P.P_11
    kf.P.P_11 <- kf.P.P_11 + kf.Qxdot * kf.dt
    let s = kf.P.P_00 + kf.R
    let k_0 = kf.P.P_00 / s
    let k_1 = kf.P.P_10 / s
    let y = xRaw - kf.x
    kf.x <- kf.x + k_0 * y
    kf.xdot <- kf.xdot + k_1 * y
    kf.P.P_00 <- kf.P.P_00 * (1.0 - k_0)
    kf.P.P_01 <- kf.P.P_01 * (1.0 - k_0)
    kf.P.P_10 <- kf.P.P_10 - k_1 * kf.P.P_00
    kf.P.P_11 <- kf.P.P_11 - k_1 * kf.P.P_01
    kf


[<Measure>] type m;;
[<Measure>] type s;;

let kf = setKalmanFilter<m,s> (Some 0.001<m/s>) (Some 0.003<m/(s*s*s)>) (Some 0.03<m>) (Some 1.0<s>)
//let newkf = applyKalmanFilter<m,s> kf 0.1<m> 0.02<m/s>

let rnd = System.Random()
let rand() = rnd.NextDouble()

let randomTrend1 = [for i in 0.0 .. 0.1 .. 10.0 -> sin i + rand()]
let randomTrend2 = [for i in 0.0 .. 0.1 .. 10.0 -> sin i * cos i + rand()]

let randomKF = [for i in 1 .. 100 -> applyKalmanFilter<m,s> kf (1.0<m>*randomTrend1.[i]) (1.0<m/s>*randomTrend2.[i]) ]