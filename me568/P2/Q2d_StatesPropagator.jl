# include("Q2c_VehicleDynamics.jl")

function Propagation(states, control, δT)
    #TODO Calculate states derivative using function
    #  VehicleDynamics(args).
    dstates =  VehicleDynamics(states, control)

    #TODO Calculate next Step
    # This is Explicit ForwardEuler
    statesNext = states + dstates .* δT
    return statesNext
end
