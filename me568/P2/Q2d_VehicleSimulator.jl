using Interpolations
using Plots
include("Q2c_VehicleDynamics.jl")
include("Q2d_StatesPropagator.jl")
x0 = [-10.0 -5.0 0.0 0.0 0.0 10.0 0.0]
ctrl = [1 0.1]
dstates = VehicleDynamics(x0, ctrl)


tc      = [0, 4, 8, 12]  # Key time step for control input
dδfc    = [0, 0.02, -0.05, 0.02] # Key value for steering rate
axc     = [0, 1.0, -2.0, 1.0] # Key value for acceleration
dt1      = 0.2  # Simulation dt
t1       = 0:dt1:tc[end]
Interpolatedδf = interpolate((tc ,), dδfc, Gridded(Constant{Next}())) # Interpolations
Interpolateax = interpolate((tc ,), axc, Gridded(Constant{Next}()))
dδf1    = Interpolatedδf.(t1) # Get interpolated steering rate signal
ax1     = Interpolateax.(t1) # Get interpolated acceleration signal

StatesListFE02 = zeros(size(t1, 1), size(x0, 2)) # Initialize states list for 0.2 update time
StatesListFE02[1, :] = x0 # Initial point

control = zeros(2,1) # Init control input

for i = 1:size(StatesListFE02, 1) - 1

    # TODO calculate the next states
    control[1] = ax1[i]
    control[2] = dδf1[i]
    StatesListFE02[i + 1, :] = Propagation(reshape(StatesListFE02[i, :],(1,7)), control, dt1)
end

dt2     = 0.01 # Smaller time step
t2      = 0:dt2:tc[end]
dδf2     = Interpolatedδf.(t2)
ax2      = Interpolateax.(t2)

StatesListFE001 = zeros(size(t2, 1), size(x0, 2)) # Initialize states list for 0.01 update time
StatesListFE001[1, :] = x0 # Initial point

for i = 1:size(StatesListFE001, 1) - 1

    # TODO calculate the next states
    control[1] = ax2[i]
    control[2] = dδf2[i]
    StatesListFE001[i + 1, :] = Propagation(reshape(StatesListFE001[i, :],(1,7)), control, dt2)
end


dt3     = 0.001 # Smaller time step
t3      = 0:dt3:tc[end]
dδf3     = Interpolatedδf.(t3)
ax3      = Interpolateax.(t3)

StatesListFE0001 = zeros(size(t3, 1), size(x0, 2)) # Initialize states list for 0.001 update time
StatesListFE0001[1, :] = x0 # Initial point

for i = 1:size(StatesListFE0001, 1) - 1

    # TODO calculate the next states
    control[1] = ax3[i]
    control[2] = dδf3[i]
    StatesListFE0001[i + 1, :] = Propagation(reshape(StatesListFE0001[i, :],(1,7)), control, dt3)
end



p = plot(size = [600, 600])
plot!(p, StatesListFE02[:, 1], StatesListFE02[:, 2], label = "ForwardEuler " * string(dt1), tickfontsize = 10, xlabel = "X (m)", ylabel = "Y (m)",guidefont=15)
plot!(p, StatesListFE001[:, 1], StatesListFE001[:, 2], label = "ForwardEuler " * string(dt2))
plot!(p, StatesListFE0001[:, 1], StatesListFE0001[:, 2], label = "ForwardEuler " * string(dt3))


