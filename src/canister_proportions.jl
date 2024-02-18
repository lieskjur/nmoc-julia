using JuMP
using Ipopt

model = JuMP.Model(Ipopt.Optimizer)

@variables(model, begin
	r >= 0
	h >= 0
end)

@objective(model, Min, 2 * pi * (r^2 + r*h))
@constraint(model, pi * r^2 * h == 0.5)

JuMP.optimize!(model)

println("r = ", value(r))
println("h = ", value(h))
println("f(r,h) = ", objective_value(model))
