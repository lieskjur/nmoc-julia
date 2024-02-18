using JuMP
using Ipopt

model = JuMP.Model(Ipopt.Optimizer)

@variable(model, x[1:2] >= 0)
@objective(model, Max, x[1]*x[2])
@constraint(model, 2*(x[1] + x[2]) <= 100)

JuMP.optimize!(model)

println("x = ", value.(x))
println("f(x) = ", objective_value(model))

