function VehicleDynamics(states, control)
    la = 1.56 
    lb = 1.64
    m = 2020
    g = 9.81
    Izz = 4095
    h = 0.6
    mu = 0.8
    x = states[1]
    y = states[2]
    v = states[3]
    r = states[4]
    ψ = states[5]
    ux = states[6]
    δf = states[7]
    ax = control[1]
    dδf = control[2]

    Fzf =  m *g * (lb/(la + lb)) - (m*h)/(la+lb) * ax #TODO Front axle load
    Fzr =  m *g * (la/(la + lb)) + (m*h)/(la+lb) * ax #TODO Rear axle load

    αf = δf - atan((v + la * r)/ux) #TODO Front slip angle
    αr = -atan((v-lb*r)/ux) #TODO Rear slip angle

    Fyf = MagicFormula(αf, Fzf, mu) #TODO Front lateral force
    Fyr = MagicFormula(αr, Fzr, mu) #TODO Rear lateral force

    dx          = ux * cos(ψ) - v * sin(ψ)#TODO 
    dy          = ux * sin(ψ) + v * cos(ψ)#TODO 
    dv          = ((Fyf + Fyr)/m) - ux * r#TODO 
    dr          = (Fyf * la - Fyr * lb)/Izz#TODO 
    dψ          =  r #TODO 
    dux         = ax #TODO 
    dδ          = dδf #TODO 
    dstates     = [dx dy dv dr dψ dux dδ]
    return dstates
end


function MagicFormula(alpha, Fz, mu)
    B =     5.68 #TODO Input Q2b value here
    C =    1.817 #TODO Input Q2b value here
    Fy =  mu * Fz * sin(C * atan((B/mu) * alpha))  #TODO Lateral force calculation
    return Fy
end


x0 = [-10.0 -5.0 0.5 0.1 0.1 10.0 0.1] # This is the initial state
ctrl = [1 0.1]  # One step control action
dstates = round.(VehicleDynamics(x0, ctrl); digits = 3) # Calculate states derivative
println("The states derivative is: ", dstates)