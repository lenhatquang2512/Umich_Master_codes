using LsqFit
using CSV
using DataFrames
############## Here is a small example on how to use LsqFit ###############
# function model(input, par)
#     value = par[1] .* exp.( - input .* par[2])
#     return value
# end
# xdata = range(0, stop=10, length=20)
# ydata = model(xdata, [1.0 2.0]) + 0.01*randn(length(xdata))
# p0 = [0.5, 0.5]
# fit = curve_fit(model, xdata, ydata, p0)
# println("Your fitting value is: ", fit.param)
########## IMPORTANT COMMENT!!!: in function model(args), we add "." in front of mathematical operators to allow broadcasting (similar to Matlab) ##########


function magicFormula(input, par)
    # par = [B, C]
    # input = xdata
    #TODO Fill in the magic formula equation here
    alpha = input[:,1]
    Fz = input[:,2]
    mu = input[:,3]
    B = par[1]
    C = par[2]
    # Fy = mu .* Fz .* sin(C .* atan((B./mu).* alpha))

    Fy = mu .* Fz .* sin.(C .* atan.((B./mu) .* alpha))

    return Fy
end

TireForceDataFrame = CSV.read("TireForce.csv", DataFrame) # Load data in DataFrame mode, we recommend you to open csv to see the structure of data
TireForceMatrix = Matrix(TireForceDataFrame) # Change data format to matrix, it is formatted in the form of [alpha Fz mu Fy], each one is a N x 1 array 

# TODO prepare xdata and ydata from TireForceMatrix
xdata = TireForceMatrix[:, 1:3]
ydata = TireForceMatrix[:, end]

p0      = [1.7, 9.5]; # Initial Guess of [B, C]

#TODO Fill in function similar to the above example
fit     = curve_fit(magicFormula,xdata,ydata,p0) 

B       = round(fit.param[1]; digits = 4)
C       = round(fit.param[2]; digits = 3)
println("B coefficient is: " ,B, "  C Coefficient is: ", C)



