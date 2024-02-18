using JuMP
using Ipopt

model = JuMP.Model(Ipopt.Optimizer)

@variable(model, x[1:3] >= 0)
@objective(model, Min, 2*(x[1]*x[2] + x[2]*x[3] + x[3]*x[1]))
@constraint(model, x[1]*x[2]*x[3] == 1)

JuMP.optimize!(model)

println("[a,b,c] = ", value.(x))
println("f(a,b,c) = ", objective_value(model))

